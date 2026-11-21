#pragma once

#include <cstdint>

namespace drivers {

/**
 * @brief One telemetry reading cycle, whatever the source device provides.
 *
 * Each sensor fills only the channels its hardware produced and sets the
 * matching valid flags; consumers (telemetry task) print per channel.
 */
struct EnvironmentalSample {
    float temperature_c{0.0f};
    float humidity_pct{0.0f};
    float pressure_hpa{0.0f};

    bool valid_temperature{false};
    bool valid_humidity{false};
    bool valid_pressure{false};
};

/** @brief Why an init() attempt failed, for diagnostics and retry logic. */
enum class InitStatus : uint8_t {
    Ok,
    BusError,           ///< I2C transaction failed (device absent or bus fault)
    WhoAmIMismatch,     ///< device answered but with an unexpected identity
    InvalidCalibration, ///< degenerate calibration coefficients (division guard)
};

/**
 * @brief Maps a driver init failure to a human-readable reason.
 *
 * Pure and host-testable: used by the retry loops to log attempt outcomes.
 */
inline const char* init_status_text(InitStatus status) noexcept {
    switch (status) {
        case InitStatus::Ok:                 return "ok";
        case InitStatus::BusError:           return "bus error";
        case InitStatus::WhoAmIMismatch:     return "WHO_AM_I mismatch";
        case InitStatus::InvalidCalibration: return "invalid calibration";
    }
    return "unknown";
}


/**
 * @brief Common interface for the I2C environmental sensors on the bus.
 *
 * One implementation per physical device — Hts221Sensor, Lps22hbSensor, ... —
 * never one per measurement channel: a device's channels share its identity
 * check, calibration image, configuration register, and burst read, so a
 * split would duplicate device management and break the channel consistency
 * the burst read guarantees.
 *
 * Consumers (sensor_task, POST) depend only on this interface, so adding a
 * sensor never changes their code. Streaming devices (e.g. the PDM
 * microphones) measure continuously rather than on demand and belong in a
 * separate interface instead of an extension of this one.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    ISensor(const ISensor&) = delete;
    ISensor& operator=(const ISensor&) = delete;

    /** @brief 7-bit I2C address, for diagnostics and the POST probe. */
    [[nodiscard]] virtual uint8_t i2c_address() const noexcept = 0;

    /** @brief Brings the device up: identity check, calibration load, configuration. */
    virtual InitStatus init() noexcept = 0;

    /**
     * @brief Reads one sample.
     *
     * @return Sample with the channels this sensor provides and only the
     *         matching valid flags set; all flags false on a bus failure.
     */
    virtual EnvironmentalSample read() noexcept = 0;

protected:
    ISensor() = default;
};

} // namespace drivers
