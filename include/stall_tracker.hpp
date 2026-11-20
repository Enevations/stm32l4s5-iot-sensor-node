#pragma once

#include <cstdint>

namespace telemetry {

/**
 * @brief Tracks consecutive dropped samples while the telemetry queue is full.
 *
 * Pure policy logic for sensor_task's bounded-send behavior (finding #6):
 * the counter grows while sends fail and resets the moment a sample is
 * enqueued again, so the recovery log can report how many samples were lost
 * during the stall. Deliberately free of FreeRTOS and HAL dependencies so it
 * compiles and tests on the host.
 */
class StallTracker {
public:
    /** @brief Number of consecutive dropped samples since the last success. */
    [[nodiscard]] std::uint32_t consecutive_drops() const noexcept { return dropped_; }

    /** @brief Records a dropped sample. @return the new consecutive total. */
    std::uint32_t note_dropped() noexcept { return ++dropped_; }

    /** @brief Records an enqueued sample; resets the consecutive count. */
    void note_enqueued() noexcept { dropped_ = 0; }

private:
    std::uint32_t dropped_{0};
};

} // namespace telemetry
