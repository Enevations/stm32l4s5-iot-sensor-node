#pragma once

/** @file rtos.hpp
 *  @brief Small C++ RAII wrappers over selected FreeRTOS kernel APIs.
 */

#include <cstdint>
#include <type_traits>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

namespace rtos {

/** @brief RAII owner for a dynamically created FreeRTOS task. */
class Task {
public:
    /** Task entry-point signature used with xTaskCreate. */
    using EntryFn = void (*)(void*);

    /**
     * @brief Creates a task.
     * @param name Task name, limited by configMAX_TASK_NAME_LEN.
     * @param stack_words Stack depth in words, not bytes.
     * @param priority FreeRTOS task priority.
     * @param fn Task entry function.
     * @param context Optional value passed to the entry function.
     */
    Task(const char* name, uint16_t stack_words, UBaseType_t priority, EntryFn fn, void* context = nullptr) noexcept {
        xTaskCreate(fn, name, stack_words, context, priority, &handle_);
    }

    /** Deletes the owned task, if creation succeeded. */
    ~Task() noexcept {
        if (handle_ != nullptr) {
            vTaskDelete(handle_);
        }
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    /** @return Native FreeRTOS task handle. */
    [[nodiscard]] TaskHandle_t handle() const noexcept { return handle_; }

private:
    TaskHandle_t handle_{nullptr};
};

template <typename T>
/** @brief Dynamically created FreeRTOS queue for trivially copyable values. */
class Queue {
    static_assert(std::is_trivially_copyable_v<T>, "Queue items must be trivially copyable.");

public:
    /** @brief Creates a queue with the requested item capacity. */
    explicit Queue(UBaseType_t capacity) noexcept {
        handle_ = xQueueCreate(capacity, sizeof(T));
    }

    /** Deletes the owned queue. */
    ~Queue() noexcept {
        if (handle_ != nullptr) {
            vQueueDelete(handle_);
        }
    }

    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    /**
     * @brief Sends a copy of an item.
     * @param item Item to copy into the queue.
     * @param ticks_to_wait Maximum wait in RTOS ticks.
     * @return true if the item was queued.
     */
    bool send(const T& item, TickType_t ticks_to_wait = portMAX_DELAY) noexcept {
        return xQueueSend(handle_, &item, ticks_to_wait) == pdTRUE;
    }

    /**
     * @brief Receives an item.
     * @param[out] out_item Receives the queued value.
     * @param ticks_to_wait Maximum wait in RTOS ticks.
     * @return true if an item was received.
     */
    bool receive(T& out_item, TickType_t ticks_to_wait = portMAX_DELAY) noexcept {
        return xQueueReceive(handle_, &out_item, ticks_to_wait) == pdTRUE;
    }

    /** @return Native FreeRTOS queue handle. */
    [[nodiscard]] QueueHandle_t handle() const noexcept { return handle_; }

private:
    QueueHandle_t handle_{nullptr};
};

class Mutex {
public:
    /** @brief Creates a FreeRTOS mutex. */
    Mutex() noexcept {
        handle_ = xSemaphoreCreateMutex();
    }

    /** Deletes the owned mutex. */
    ~Mutex() noexcept {
        if (handle_ != nullptr) {
            vSemaphoreDelete(handle_);
        }
    }

    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /**
     * @brief Waits to acquire the mutex.
     * @param ticks_to_wait Maximum wait in RTOS ticks.
     * @return true when the mutex was acquired.
     */
    bool lock(TickType_t ticks_to_wait = portMAX_DELAY) noexcept {
        return xSemaphoreTake(handle_, ticks_to_wait) == pdTRUE;
    }

    /** Releases the mutex. */
    void unlock() noexcept {
        xSemaphoreGive(handle_);
    }

    /** @return Native FreeRTOS mutex handle. */
    [[nodiscard]] SemaphoreHandle_t handle() const noexcept { return handle_; }

private:
    SemaphoreHandle_t handle_{nullptr};
};

/**
 * @brief RAII lock that guards a resource only while the scheduler is running.
 *
 * Before the scheduler starts, only main() executes, so there is no contention
 * to guard against — and acquiring a FreeRTOS mutex in that window relies on
 * kernel-internal pre-scheduler behavior (priority inheritance against
 * pxCurrentTCB). The guard therefore proceeds unlocked when the scheduler has
 * not started and locks with portMAX_DELAY once it runs. Not usable while the
 * scheduler is suspended.
 */
class SchedulerLock {
public:
    /** @brief Locks the mutex unless the scheduler has not started yet. */
    explicit SchedulerLock(Mutex& mutex) noexcept : mutex_(mutex) {
        if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
            return;
        }
        proceed_ = mutex_.lock(portMAX_DELAY);
        held_ = proceed_;
    }

    /** @brief Releases the mutex, if it was acquired. */
    ~SchedulerLock() noexcept {
        if (held_) {
            mutex_.unlock();
        }
    }

    SchedulerLock(const SchedulerLock&) = delete;
    SchedulerLock& operator=(const SchedulerLock&) = delete;

    /** @return true when the protected call may proceed. */
    explicit operator bool() const noexcept { return proceed_; }

private:
    Mutex& mutex_;
    bool proceed_{true};
    bool held_{false};
};

class LockGuard {
public:
    /** @brief Acquires the mutex for the lifetime of this guard. */
    explicit LockGuard(Mutex& mutex) noexcept : mutex_(mutex) {
        mutex_.lock();
    }

    /** Releases the guarded mutex. */
    ~LockGuard() noexcept {
        mutex_.unlock();
    }

    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    Mutex& mutex_;
};

/**
 * @brief Suspends the current task.
 * @param ms Delay duration in milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

} // namespace rtos