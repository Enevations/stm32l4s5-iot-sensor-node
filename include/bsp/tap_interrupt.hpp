#pragma once

/** @file tap_interrupt.hpp
 *  @brief EXTI interrupt on PD11 for the LSM6DSL INT1 output.
 *
 * The LSM6DSL drives INT1 high for the Quiet window (~29 ms) each time its
 * hardware double-tap recognizer fires (MD1_CFG routes only the double-tap
 * event to INT1, active-high push-pull). This module maps PD11 to EXTI line
 * 11 with a rising-edge interrupt; the ISR hands the pulse to a task via a
 * FreeRTOS direct-to-task notification, so the tap task can sleep at 0% CPU
 * between gestures and do the (blocking, polled) I2C read in task context.
 */

#include "FreeRTOS.h"
#include "task.h"

namespace bsp {

/**
 * @brief Configures PD11 as EXTI rising-edge input and enables the
 *        EXTI15_10 interrupt, which then gives one direct-to-task
 *        notification per INT1 pulse.
 *
 * The interrupt priority is set to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 * so the ISR may use the FromISR kernel calls.
 *
 * @param notify_task Handle of the task to notify on each INT1 pulse
 *        (typically the tap task's own handle via
 *        xTaskGetCurrentTaskHandle()).
 */
void tap_interrupt_init(TaskHandle_t notify_task) noexcept;

} // namespace bsp
