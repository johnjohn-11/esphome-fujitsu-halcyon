#include "esphome-fujitsu-halcyon.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <string>
#include <type_traits>

#include <esphome/core/helpers.h>
#include <soc/uart_reg.h>

namespace esphome::fujitsu_general_airstage_h_controller {

static const char* TAG = "fujitsu_halcyon";

// If we are receiving from the bus but have not been handed a transmit token
// within this window, control commands cannot be delivered (the unit is
// effectively read-only). Warn the user once with guidance.
static constexpr uint32_t TX_TOKEN_TIMEOUT_MS = 15000;

constexpr std::array ControllerName = { "Primary", "Secondary", "Undocumented" };

void FujitsuHalcyonController::loop() {
    this->controller->process_uart_data();
    this->check_sensor_timeout_();
}

// Push the effective use-sensor state to the unit: the switch's intent, masked by
// whether the external reading is usable. Returns false if the unit refused it
// (feature not supported, or locked and ignore_lock is off).
bool FujitsuHalcyonController::apply_use_sensor_() {
    if (this->use_sensor_switch_ == nullptr || !this->use_sensor_applied_)
        return false;

    return this->controller->use_sensor(this->use_sensor_switch_->state && this->sensor_usable_(), this->ignore_lock_);
}

void FujitsuHalcyonController::check_sensor_timeout_() {
    if (this->sensor_timeout_ms_ == 0 || this->temperature_sensor_ == nullptr || this->temperature_stale_)
        return;

    if (millis() - this->last_valid_temperature_ms_ < this->sensor_timeout_ms_)
        return;

    // Also reached when no reading has ever arrived since boot (the timer starts
    // in setup), so a switch left on with a dead sensor is reported too.
    this->temperature_stale_ = true;

    if (this->use_sensor_switch_ != nullptr && this->use_sensor_switch_->state) {
        ESP_LOGW(TAG, "No valid reading from the temperature sensor for %u s, the unit uses its own sensor until readings resume",
            static_cast<unsigned>(this->sensor_timeout_ms_ / 1000));
        this->apply_use_sensor_();
    } else {
        ESP_LOGD(TAG, "No valid reading from the temperature sensor for %u s", static_cast<unsigned>(this->sensor_timeout_ms_ / 1000));
    }
}

void FujitsuHalcyonController::setup() {
    const auto uart_num = static_cast<uart_port_t>(static_cast<uart::IDFUARTComponent*>(this->parent_)->get_hw_serial_number());

    // Currently no way to do this in IDFUARTComponent YAML configuration without setting the flow control pin.
    // Using RTS is not needed, but the side effect of suppressing input during output is, as the LIN chip provides loopback.
    if (auto err = uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX); err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART mode: %s", esp_err_to_name(err));
        this->mark_failed();
        return;
    }

    // On most current ESP-IDF targets, uart_ll_set_mode_rs485_half_duplex() also sets
    // UART_DL0_EN and UART_DL1_EN, each of which extends the stop bit by one bit time.
    // At the 500 baud of this bus that is 4 ms per byte and 28 ms per 8 byte frame.
    // Indoor units tolerate the malformed timing; OEM wired controllers do not, and
    // drop this component from the token ring after the initial handshake.
    // Reported upstream in espressif/esp-idf#12568, fix proposed in #12569, not merged.
    REG_CLR_BIT(UART_RS485_CONF_REG(uart_num), UART_DL0_EN_M | UART_DL1_EN_M);

    this->controller = new fujitsu_general::airstage::h::Controller(
        this->controller_address_,
        {
            .Config = [this](const fujitsu_general::airstage::h::Config& data){ this->update_from_device(data); },
            .Error  = [this](const fujitsu_general::airstage::h::Packet& data){ this->update_from_device(data); },
            .ZoneConfig = [this](const fujitsu_general::airstage::h::ZoneConfig& data){ this->update_from_device(data); },
            .Function = [this](const fujitsu_general::airstage::h::Function& data){ this->update_from_device(data); },
            .ControllerConfig = [this](const uint8_t address, const fujitsu_general::airstage::h::Config& data){ this->update_from_controller(address, data); },
            .InitializationStage = [this](const fujitsu_general::airstage::h::InitializationStageEnum stage){
                this->on_initialization_stage(stage);
            },
            .AvailableBytes = [this]() -> size_t {
                return this->available();
            },
            .ReadBytes  = [this](uint8_t *buf, size_t length){
                this->read_array(buf, length);
                this->received_bytes_ = true;
                this->log_buffer("RX", buf, length);
            },
            .WriteBytes = [this](const uint8_t *buf, size_t length){
                this->transmitted_ = true;
                this->write_array(buf, length);
                this->log_buffer("TX", buf, length);
            }
        }
    );

    // Apply user-supplied feature overrides from YAML. features_override_ was
    // initialized to DefaultFeatures and individually mutated by any setters
    // called from to_code(); fields the user did not specify still hold their
    // DefaultFeatures value. Must be applied before the first packet is processed;
    // setup() runs before loop() so this is safe.
    this->controller->set_features(this->features_override_);
    this->controller->set_autoconf(this->autoconf_);

    this->connected_sensor_->publish_initial_state(false);

    // Diagnostic for the common "reads state but cannot control" failure mode:
    // if the bus is delivering packets but this controller is never granted a
    // transmit token, warn once with actionable guidance. Secondary controllers
    // (controller_address > 0) only get to register during the preceding
    // controller's power-on window.
    this->set_timeout(TX_TOKEN_TIMEOUT_MS, [this]() {
        if (this->received_bytes_ && !this->transmitted_)
            ESP_LOGW(TAG,
                "Receiving data but no transmit token after %u s - control commands will have no effect. "
                "If controller_address > 0, power this device on before (or with) the preceding "
                "controller(s) so it can register for the token. See the README Troubleshooting section.",
                static_cast<unsigned>(TX_TOKEN_TIMEOUT_MS / 1000));
    });

    // Use the specified sensor for this component's reported temperature. The
    // value must be in Celsius. Convert in YAML (see README) if your source is
    // Fahrenheit. Auto-detecting the unit is unreliable because
    // unit_of_measurement is lost when importing a Home Assistant sensor.
    if (this->temperature_sensor_ != nullptr) {
        this->temperature_sensor_->add_on_state_callback([this](float state) {
            this->current_temperature = state;
            this->publish_state();

            if (std::isfinite(state)) {
                this->last_valid_temperature_ms_ = millis();

                // First valid reading, or readings resuming after a timeout: the
                // external sensor is usable again, so hand it back to the unit if
                // the switch is on.
                if (!this->sensor_usable_()) {
                    if (this->temperature_stale_)
                        ESP_LOGI(TAG, "Temperature sensor readings resumed");
                    this->temperature_valid_ = true;
                    this->temperature_stale_ = false;
                    this->apply_use_sensor_();
                }

                // Send this temperature to the Fujitsu IU
                this->controller->set_current_temperature(state);
            }
        });

        // Start the freshness timer now so a sensor that never delivers is
        // detected too, not only one that stops after a first reading.
        this->last_valid_temperature_ms_ = millis();
        this->current_temperature = this->temperature_sensor_->state;
        if (std::isfinite(this->current_temperature)) {
            this->temperature_valid_ = true;
            this->controller->set_current_temperature(this->current_temperature);
        }
    }

    if (this->humidity_sensor_ != nullptr) {
        this->humidity_sensor_->add_on_state_callback([this](float state) {
            this->current_humidity = state;
            this->publish_state();
        });

        this->current_humidity = this->humidity_sensor_->state;
    }

    // Read the restored use_sensor state now (the switch is not a Component, so it
    // is not restored on its own). It is applied to the unit later, once sensor
    // switching is confirmed, in on_initialization_stage().
    if (this->use_sensor_switch_ != nullptr)
        this->pending_use_sensor_ = this->use_sensor_switch_->get_initial_state_with_restore_mode();
}

void FujitsuHalcyonController::on_initialization_stage(const fujitsu_general::airstage::h::InitializationStageEnum stage) {
    using fujitsu_general::airstage::h::InitializationStageEnum;
    using stage_t = std::underlying_type_t<InitializationStageEnum>;

    // Update initialization stage sensor with a readable label plus progress.
    static constexpr std::array<const char*, 8> STAGE_LABELS = {
        "Detecting features",   // DetectFeatureSupport
        "Requesting features",  // FeatureRequestTx
        "Waiting for features",  // FeatureRequestRx
        "Requesting zones",     // ZoneRequestEnabled
        "Finding controllers",  // FindNextControllerTx
        "Finding controllers",  // FindNextControllerRx
        "Reading zones",        // ZoneRequestActive
        "Complete",             // Complete
    };
    auto index = static_cast<stage_t>(stage);
    const char* label = index < STAGE_LABELS.size() ? STAGE_LABELS[index] : "Unknown";
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%s (%u/%u)", label, index, static_cast<stage_t>(InitializationStageEnum::Complete));
    this->initialization_sensor_->publish_state(buf);
    ESP_LOGD(TAG, "Initialization stage: %s", buf);

    // Update connected sensor
    this->connected_sensor_->publish_state(stage == InitializationStageEnum::Complete);

    // Everything below depends on features being known
    if (stage <= InitializationStageEnum::FeatureRequestRx)
        return;

    // Publish feature-dependent entity state now that features are known. The
    // entities are declared statically in YAML and created only when present, so
    // there is no set_internal() toggling here.
    auto& features = this->controller->get_features();

    // Publish supported features as a human-readable diagnostic string.
    {
        char buf[255];
        std::snprintf(buf, sizeof(buf), "Mode: %s%s%s%s%s | Fan: %s%s%s%s%s" "%s%s%s%s%s%s%s",
            features.Mode.Auto ? " Auto" : "",
            features.Mode.Heat ? " Heat" : "",
            features.Mode.Cool ? " Cool" : "",
            features.Mode.Dry  ? " Dry"  : "",
            features.Mode.Fan  ? " Fan"  : "",

            features.FanSpeed.Auto   ? " Auto"   : "",
            features.FanSpeed.High   ? " High"   : "",
            features.FanSpeed.Medium ? " Medium" : "",
            features.FanSpeed.Low    ? " Low"    : "",
            features.FanSpeed.Quiet  ? " Quiet"  : "",

            features.EconomyMode       ? " | Economy"            : "",
            features.FilterTimer       ? " | Filter Timer"       : "",
            features.SensorSwitching   ? " | Sensor Switching"   : "",
            features.Maintenance       ? " | Maintenance"        : "",
            features.VerticalLouvers   ? " | Vertical Louvers"   : "",
            features.HorizontalLouvers ? " | Horizontal Louvers" : "",
            features.Zones             ? " | Zones"              : ""
        );
        this->supported_features_sensor_->publish_state(buf);
    }

    if (features.SensorSwitching && this->temperature_sensor_ != nullptr && this->use_sensor_switch_ != nullptr) {
        if (!this->use_sensor_applied_) {
            // Sensor switching is confirmed, so a write is no longer rejected.
            // Apply the state restored in setup() once, then reflect it in HA. If
            // the unit refuses (locked), show the switch off so HA matches reality.
            bool state = this->pending_use_sensor_.value_or(false);
            this->use_sensor_switch_->state = state;
            this->use_sensor_applied_ = true;
            if (!this->apply_use_sensor_() && state) {
                ESP_LOGW(TAG, "Unit refused the restored use_sensor state (locked?), leaving the switch off");
                state = false;
            }
            this->use_sensor_switch_->publish_state(state);
        } else {
            this->use_sensor_switch_->publish_state(this->use_sensor_switch_->state);
        }
    }

    if (features.FilterTimer && this->filter_sensor_ != nullptr && this->filter_sensor_->has_state())
        this->filter_sensor_->publish_state(this->filter_sensor_->state);

    // Publish zone state now that zones are known.
    if (features.Zones) {
        auto& zones = this->controller->get_zones();

        for (size_t i = 0; i < this->zone_switches_.size(); i++)
            if (zones.EnabledZones[i] && this->zone_switches_[i] != nullptr)
                this->zone_switches_[i]->publish_state(this->zone_switches_[i]->state);

        if (this->zone_group_day_switch_ != nullptr)
            this->zone_group_day_switch_->publish_state(this->zone_group_day_switch_->state);
        if (this->zone_group_night_switch_ != nullptr)
            this->zone_group_night_switch_->publish_state(this->zone_group_night_switch_->state);
    }

    // Warn once, at completion, if the user declared a feature entity that the
    // unit does not actually report. These entities were opted into from YAML.
    if (stage == InitializationStageEnum::Complete) {
        if (this->use_sensor_declared_) {
            if (!features.SensorSwitching)
                ESP_LOGW(TAG, "use_sensor declared but this unit does not report sensor switching support, the switch will have no effect");
            else if (this->temperature_sensor_ == nullptr)
                ESP_LOGW(TAG, "use_sensor declared but no temperature_sensor_id is configured, the switch will have no effect");
        }
        if (this->filter_entity_declared_ && !features.FilterTimer)
            ESP_LOGW(TAG, "filter_timer_expired/reset_filter_timer declared but this unit does not report a filter timer");
        if (this->louver_v_declared_ && !features.VerticalLouvers)
            ESP_LOGW(TAG, "advance_vertical_louver declared but this unit does not report vertical louvers");
        if (this->louver_h_declared_ && !features.HorizontalLouvers)
            ESP_LOGW(TAG, "advance_horizontal_louver declared but this unit does not report horizontal louvers");
        if (this->zones_declared_ && !features.Zones)
            ESP_LOGW(TAG, "zone_* declared but this unit does not report zone support");
    }
}

void FujitsuHalcyonController::log_buffer(const char* dir, const uint8_t* buf, size_t length) {
    // Frames are at most Packet::FrameSize bytes; clamp so the fixed-size pretty
    // buffer below can never overflow. Sizing the buffer from this compile-time
    // constant (rather than tbuf.size()) also avoids a non-standard VLA.
    length = std::min(length, static_cast<size_t>(fujitsu_general::airstage::h::Packet::FrameSize));

    auto tbuf = std::vector<uint8_t>(buf, buf + length);
    for (auto &b : tbuf)
        b ^= 0xFF;

#if defined(USE_TZSP)
    this->tzsp_send(tbuf);
#endif

    char pretty_buf[esphome::format_hex_pretty_size(fujitsu_general::airstage::h::Packet::FrameSize)];
    esphome::format_hex_pretty_to(pretty_buf, tbuf, ' ');
    ESP_LOGD(TAG, "%s: %s", dir, pretty_buf);
}

void FujitsuHalcyonController::dump_config() {
    LOG_CLIMATE("", "FujitsuHalcyonController", this);
    ESP_LOGCONFIG(TAG, "  Controller Address: %u (%s)", this->controller_address_, ControllerName[std::clamp(static_cast<size_t>(this->controller_address_), 0u, ControllerName.size() - 1)]);
    ESP_LOGCONFIG(TAG, "  Remote Temperature Controller Address: %u (%s)", this->temperature_controller_address_, ControllerName[std::clamp(static_cast<size_t>(this->temperature_controller_address_), 0u, ControllerName.size() - 1)]);
    LOG_SENSOR("  ", "Remote Temperature Controller Sensor", this->remote_sensor_);
    LOG_SENSOR("  ", "Temperature Sensor", this->temperature_sensor_);
    LOG_SENSOR("  ", "Humidity Sensor", this->humidity_sensor_);
    ESP_LOGCONFIG(TAG, "  Ignore Lock: %s", this->ignore_lock_ ? "YES" : "NO");
    if (this->temperature_sensor_ != nullptr)
        ESP_LOGCONFIG(TAG, "  Sensor Timeout: %u s%s", static_cast<unsigned>(this->sensor_timeout_ms_ / 1000), this->sensor_timeout_ms_ ? "" : " (disabled)");
    ESP_LOGCONFIG(TAG, "  Standby Mode: %s", this->standby_sensor_->state ? "ACTIVE" : "NORMAL");

    if (this->controller != nullptr && this->controller->is_initialized()) {
        auto& features = this->controller->get_features();

        ESP_LOGCONFIG(TAG, "  Additional Features:%s", features.FilterTimer || features.Maintenance || features.SensorSwitching || features.Zones ? "" : " NONE");
        if (features.FilterTimer)
            ESP_LOGCONFIG(TAG, "    - Filter Timer");
        if (features.Maintenance)
            ESP_LOGCONFIG(TAG, "    - Maintenance");
        if (features.SensorSwitching)
            ESP_LOGCONFIG(TAG, "    - Sensor Switching");
        if (features.Zones) {
            auto& zones = this->controller->get_zones();

            // Build a comma-separated list of enabled zones
            char buf[3 * fujitsu_general::airstage::h::MaxZone + 1];
            int offset = 0;
            for (size_t i = 0; i < zones.EnabledZones.size() && offset < static_cast<int>(sizeof(buf)); i++)
                if (zones.EnabledZones[i])
                    offset += std::snprintf(buf + offset, sizeof(buf) - offset, "%u, ", i + 1);
            buf[offset ? offset - 2 : 0] = '\0';

            ESP_LOGCONFIG(TAG, "    - Zones: %s", buf[0] ? buf : "NONE");
            ESP_LOGCONFIG(TAG, "        Common Zone: %s", zones.ZoneCommon ? "YES" : "NO");
        }

        if (features.FilterTimer && this->filter_sensor_ != nullptr)
            ESP_LOGCONFIG(TAG, "  Filter Timer: %s", this->filter_sensor_->state ? "EXPIRED" : "OK");
        if (features.SensorSwitching && this->use_sensor_switch_ != nullptr)
            ESP_LOGCONFIG(TAG, "  Use Temperature Sensor: %s", this->use_sensor_switch_->state ? "YES" : "NO");
    }

#if defined(USE_TZSP)
    LOG_TZSP("  ", this);
#endif

    this->dump_traits_(TAG);
}

climate::ClimateTraits FujitsuHalcyonController::traits() {
    using namespace climate;

    auto traits = ClimateTraits();

    // Target temperature / Setpoint
    // The setpoint is whole degrees, but the current temperature has half-degree
    // resolution. Set the two steps separately so Home Assistant does not round
    // the displayed current temperature to whole degrees.
    traits.set_visual_target_temperature_step(1);
    traits.set_visual_current_temperature_step(0.5);
    traits.set_visual_min_temperature(fujitsu_general::airstage::h::MinSetpoint);
    traits.set_visual_max_temperature(fujitsu_general::airstage::h::MaxSetpoint);

    // controller is null if setup() failed early; return the basic temperature
    // traits so the entity still registers rather than dereferencing a nullptr.
    if (this->controller == nullptr)
        return traits;

    auto& features = this->controller->get_features();

    // Current temperature. A source exists if an external sensor is configured,
    // or if we read temperature from a different controller on the bus.
    if (this->temperature_sensor_ != nullptr ||
        this->temperature_controller_address_ != this->controller_address_)
        traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

    // Current humidity
    if (this->humidity_sensor_ != nullptr)
        traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY);

    // Mode
    if (features.Mode.Auto)
        traits.add_supported_mode(ClimateMode::CLIMATE_MODE_HEAT_COOL);
    if (features.Mode.Heat)
        traits.add_supported_mode(ClimateMode::CLIMATE_MODE_HEAT);
    if (features.Mode.Fan)
        traits.add_supported_mode(ClimateMode::CLIMATE_MODE_FAN_ONLY);
    if (features.Mode.Dry)
        traits.add_supported_mode(ClimateMode::CLIMATE_MODE_DRY);
    if (features.Mode.Cool)
        traits.add_supported_mode(ClimateMode::CLIMATE_MODE_COOL);

    // Fan mode / speed
    if (features.FanSpeed.Quiet)
        traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_QUIET);
    if (features.FanSpeed.Low)
        traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_LOW);
    if (features.FanSpeed.Medium)
        traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_MEDIUM);
    if (features.FanSpeed.High)
        traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_HIGH);
    if (features.FanSpeed.Auto)
        traits.add_supported_fan_mode(ClimateFanMode::CLIMATE_FAN_AUTO);

    // Economy mode
    if (features.EconomyMode)
    {
        traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);
        traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_ECO);
    }

    // Swing
    if (features.HorizontalLouvers || features.VerticalLouvers) {
        traits.add_supported_swing_mode(ClimateSwingMode::CLIMATE_SWING_OFF);
        if (features.HorizontalLouvers)
            traits.add_supported_swing_mode(ClimateSwingMode::CLIMATE_SWING_HORIZONTAL);
        if (features.VerticalLouvers)
            traits.add_supported_swing_mode(ClimateSwingMode::CLIMATE_SWING_VERTICAL);
        if (features.HorizontalLouvers && features.VerticalLouvers)
            traits.add_supported_swing_mode(ClimateSwingMode::CLIMATE_SWING_BOTH);
    }

    return traits;
}

void FujitsuHalcyonController::control(const climate::ClimateCall& call) {
    using climate::ClimateMode;
    using climate::ClimatePreset;
    using climate::ClimateSwingMode;

    // Target temperature / Setpoint
    if (call.get_target_temperature().has_value())
        this->controller->set_setpoint(std::lround(call.get_target_temperature().value()), this->ignore_lock_);

    // Economy mode
    if (call.get_preset().has_value())
        this->controller->set_economy(call.get_preset().value() == ClimatePreset::CLIMATE_PRESET_ECO, this->ignore_lock_);

    // Fan mode / speed
    if (call.get_fan_mode().has_value())
        this->controller->set_fan_speed(climate_fan_mode_to_fan_speed(call.get_fan_mode().value()), this->ignore_lock_);

    // Mode / enabled
    if (call.get_mode().has_value()) {
        if (call.get_mode().value() == ClimateMode::CLIMATE_MODE_OFF)
            this->controller->set_enabled(false, this->ignore_lock_);
        else {
            this->controller->set_enabled(true, this->ignore_lock_);
            this->controller->set_mode(climate_mode_to_mode(call.get_mode().value()), this->ignore_lock_);
        }
    }

    // Swing mode
    if (call.get_swing_mode().has_value()) {
        const auto swing_mode = climate_swing_mode_to_swing_mode(call.get_swing_mode().value());
        this->controller->set_horizontal_swing(swing_mode.first, this->ignore_lock_);
        this->controller->set_vertical_swing(swing_mode.second, this->ignore_lock_);
    }

    this->publish_state();
}

void FujitsuHalcyonController::update_from_device(const fujitsu_general::airstage::h::Config& data) {
    using climate::ClimateFanMode;
    using climate::ClimateMode;
    using climate::ClimatePreset;
    using climate::ClimateSwingMode;

    auto need_to_publish = false;

    // Error sensor (binary)
    if (!this->error_sensor_->has_state())
        this->error_sensor_->publish_state(data.IndoorUnit.Error);

    // Error sensor (text)
    if (!this->error_code_sensor_->has_state() && !data.IndoorUnit.Error)
        this->error_code_sensor_->publish_state("");

    // Standby mode sensor
    // This can indicate defrosting, performing oil recovery, waiting for other units to complete....
    if (!this->standby_sensor_->has_state() || data.IndoorUnit.StandbyMode != this->standby_sensor_->state)
        this->standby_sensor_->publish_state(data.IndoorUnit.StandbyMode);

    // Filter sensor
    if (this->filter_sensor_ != nullptr && this->controller->get_features().FilterTimer && (!this->filter_sensor_->has_state() || data.IndoorUnit.FilterTimerExpired != this->filter_sensor_->state))
        this->filter_sensor_->publish_state(data.IndoorUnit.FilterTimerExpired);

    // Target temperature / Setpoint
    if (data.Setpoint != this->target_temperature) {
        this->target_temperature = data.Setpoint;
        need_to_publish = true;
    }

    // Economy mode
    if (data.Economy != (this->preset == ClimatePreset::CLIMATE_PRESET_ECO)) {
        this->preset = data.Economy ? ClimatePreset::CLIMATE_PRESET_ECO : ClimatePreset::CLIMATE_PRESET_NONE;
        need_to_publish = true;
    }

    // Fan mode / speed
    const auto fan_mode = fan_speed_to_climate_fan_mode(data.FanSpeed);
    if (fan_mode != this->fan_mode) {
        this->fan_mode = fan_mode;
        need_to_publish = true;
    }

    // Mode / enabled
    const auto mode = data.Enabled ? mode_to_climate_mode(data.Mode) : ClimateMode::CLIMATE_MODE_OFF;
    if (mode != this->mode) {
        this->mode = mode;
        need_to_publish = true;
    }

    // Swing mode
    const auto swing_mode = swing_mode_to_climate_swing_mode(data.SwingHorizontal, data.SwingVertical);
    if (swing_mode != this->swing_mode) {
        this->swing_mode = swing_mode;
        need_to_publish = true;
    }

    if (need_to_publish)
        this->publish_state();
}

void FujitsuHalcyonController::update_from_device(const fujitsu_general::airstage::h::ZoneConfig& data) {
    for (size_t i = 0; i < this->zone_switches_.size(); i++)
        if (this->zone_switches_[i] != nullptr)
            this->zone_switches_[i]->publish_state(data.ActiveZones[i]);

    if (this->zone_group_day_switch_ != nullptr)
        this->zone_group_day_switch_->publish_state(data.ActiveZoneGroups.Day);
    if (this->zone_group_night_switch_ != nullptr)
        this->zone_group_night_switch_->publish_state(data.ActiveZoneGroups.Night);
}

void FujitsuHalcyonController::update_from_device(const fujitsu_general::airstage::h::Packet& data) {
    using fujitsu_general::airstage::h::PacketTypeEnum;

    // Error packet
    if (data.Type == PacketTypeEnum::Error)
    {
        const bool has_error = data.Error.ErrorCode != 0;

        // Error sensor (boolean)
        if (has_error != this->error_sensor_->state)
            this->error_sensor_->publish_state(has_error);

        // Error sensor (text): "AA BB[.CCC]" (source address + error code + extended).
        // Build the desired string first, then publish only if it differs from the
        // current value. Comparing the full string (rather than just error/no-error)
        // means a fault changing from one non-zero code to another still refreshes.
        std::string error_text;
        if (has_error)
        {
            const auto error_bytes = std::to_array<uint8_t>({ data.SourceAddress, data.Error.ErrorCode });
            const auto error_buf_len = esphome::format_hex_pretty_size(error_bytes.size());
            constexpr auto extended_error_buf_len = 4;

            char error_buf[error_buf_len + extended_error_buf_len];
            esphome::format_hex_pretty_to(error_buf, error_bytes, ' ');

            if (data.Error.ErrorCodeExtended)
                std::snprintf(error_buf + error_buf_len - 1, extended_error_buf_len + 1, ".%u", data.Error.ErrorCodeExtended);

            // NOTE: Error codes D? appear to be remapped to J?, but maybe not in all cases?
            if ((data.Error.ErrorCode & 0xF0) == 0xD0)
                error_buf[3] = 'J';

            error_text = error_buf;
        }

        if (error_text != this->error_code_sensor_->get_raw_state())
            this->error_code_sensor_->publish_state(error_text);
    }
}

void FujitsuHalcyonController::update_from_device(const fujitsu_general::airstage::h::Function& data) {
    this->function_number_->publish_state(data.Function);
    this->function_value_number_->publish_state(data.Value);
    this->function_unit_number_->publish_state(data.Unit);
}

void FujitsuHalcyonController::update_from_controller(const uint8_t address, const fujitsu_general::airstage::h::Config& data) {
    if (address == this->temperature_controller_address_ && data.Controller.Temperature) {
        const float temperature = data.Controller.Temperature;
        // When no external sensor is configured, the bus controller temperature is
        // this component's current temperature. This does not depend on the
        // remote_sensor entity, which is only created when declared.
        if (this->temperature_sensor_ == nullptr && temperature != this->current_temperature) {
            this->current_temperature = temperature;
            this->publish_state();
        }
        // Publish it to the remote_sensor entity as well, if it was declared.
        if (this->remote_sensor_ != nullptr && temperature != this->remote_sensor_->get_raw_state())
            this->remote_sensor_->publish_state(temperature);
    }
}

constexpr climate::ClimateMode FujitsuHalcyonController::mode_to_climate_mode(const fujitsu_general::airstage::h::ModeEnum mode) {
    using climate::ClimateMode;
    using FujitsuMode = fujitsu_general::airstage::h::ModeEnum;

    switch (mode) {
        case FujitsuMode::Fan:  return ClimateMode::CLIMATE_MODE_FAN_ONLY;
        case FujitsuMode::Dry:  return ClimateMode::CLIMATE_MODE_DRY;
        case FujitsuMode::Cool: return ClimateMode::CLIMATE_MODE_COOL;
        case FujitsuMode::Heat: return ClimateMode::CLIMATE_MODE_HEAT;
        case FujitsuMode::Auto: return ClimateMode::CLIMATE_MODE_HEAT_COOL;

        // Should not get to this point
        default: return ClimateMode::CLIMATE_MODE_FAN_ONLY;
    }
}

constexpr climate::ClimateFanMode FujitsuHalcyonController::fan_speed_to_climate_fan_mode(const fujitsu_general::airstage::h::FanSpeedEnum fan_speed) {
    using climate::ClimateFanMode;
    using FujitsuFanMode = fujitsu_general::airstage::h::FanSpeedEnum;

    switch (fan_speed) {
        case FujitsuFanMode::Auto:   return ClimateFanMode::CLIMATE_FAN_AUTO;
        case FujitsuFanMode::Quiet:  return ClimateFanMode::CLIMATE_FAN_QUIET;
        case FujitsuFanMode::Low:    return ClimateFanMode::CLIMATE_FAN_LOW;
        case FujitsuFanMode::Medium: return ClimateFanMode::CLIMATE_FAN_MEDIUM;
        case FujitsuFanMode::High:   return ClimateFanMode::CLIMATE_FAN_HIGH;

        // Should not get to this point
        default: return ClimateFanMode::CLIMATE_FAN_AUTO;
    }
}

constexpr climate::ClimateSwingMode FujitsuHalcyonController::swing_mode_to_climate_swing_mode(bool horizontal, bool vertical) {
    using climate::ClimateSwingMode;

    if (horizontal && vertical)
        return ClimateSwingMode::CLIMATE_SWING_BOTH;
    else if (horizontal)
        return ClimateSwingMode::CLIMATE_SWING_HORIZONTAL;
    else if (vertical)
        return ClimateSwingMode::CLIMATE_SWING_VERTICAL;
    else
        return ClimateSwingMode::CLIMATE_SWING_OFF;
}

constexpr fujitsu_general::airstage::h::ModeEnum FujitsuHalcyonController::climate_mode_to_mode(climate::ClimateMode mode) {
    using climate::ClimateMode;
    using FujitsuMode = fujitsu_general::airstage::h::ModeEnum;

    switch (mode) {
        case ClimateMode::CLIMATE_MODE_HEAT_COOL: return FujitsuMode::Auto;
        case ClimateMode::CLIMATE_MODE_COOL:      return FujitsuMode::Cool;
        case ClimateMode::CLIMATE_MODE_HEAT:      return FujitsuMode::Heat;
        case ClimateMode::CLIMATE_MODE_FAN_ONLY:  return FujitsuMode::Fan;
        case ClimateMode::CLIMATE_MODE_DRY:       return FujitsuMode::Dry;

        // Should not get to this point if traits is respected
        default: return FujitsuMode::Fan;
    }
}

constexpr fujitsu_general::airstage::h::FanSpeedEnum FujitsuHalcyonController::climate_fan_mode_to_fan_speed(climate::ClimateFanMode fan_speed) {
    using climate::ClimateFanMode;
    using FujitsuFanMode = fujitsu_general::airstage::h::FanSpeedEnum;

    switch (fan_speed) {
        case ClimateFanMode::CLIMATE_FAN_AUTO:   return FujitsuFanMode::Auto;
        case ClimateFanMode::CLIMATE_FAN_LOW:    return FujitsuFanMode::Low;
        case ClimateFanMode::CLIMATE_FAN_MEDIUM: return FujitsuFanMode::Medium;
        case ClimateFanMode::CLIMATE_FAN_HIGH:   return FujitsuFanMode::High;
        case ClimateFanMode::CLIMATE_FAN_QUIET:  return FujitsuFanMode::Quiet;

        // Should not get to this point if traits is respected
        default: return FujitsuFanMode::Auto;
    }
}

constexpr std::pair<bool, bool> FujitsuHalcyonController::climate_swing_mode_to_swing_mode(climate::ClimateSwingMode swing_mode) {
    using climate::ClimateSwingMode;
    using SwingMode = std::pair<bool, bool>;

    switch (swing_mode) {
        case ClimateSwingMode::CLIMATE_SWING_OFF:        return SwingMode(false, false);
        case ClimateSwingMode::CLIMATE_SWING_BOTH:       return SwingMode(true, true);
        case ClimateSwingMode::CLIMATE_SWING_VERTICAL:   return SwingMode(false, true);
        case ClimateSwingMode::CLIMATE_SWING_HORIZONTAL: return SwingMode(true, false);

        // Should not get to this point
        default: return SwingMode(false, false);
    }
}

}
