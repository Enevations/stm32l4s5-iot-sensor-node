#pragma once

/** @file board_state.hpp
 *  @brief Hardware-fault state shared by boot code and tasks.
 *
 * The state is a single atomic flag: heartbeat() blinks fast while it is
 * set, and every subsystem that raised it clears it only when the condition
 * it flagged has recovered (so one subsystem's recovery never masks another
 * subsystem's fault).
 */

namespace bsp {

/** @brief Raises the hardware-fault flag (heartbeat blinks fast). */
void set_hardware_fault() noexcept;

/** @brief Clears the hardware-fault flag. Call only when the condition this
 *  caller flagged has recovered. */
void clear_hardware_fault() noexcept;

/** @brief Returns true while the hardware-fault flag is raised. */
[[nodiscard]] bool is_hardware_fault_active() noexcept;

} // namespace bsp
