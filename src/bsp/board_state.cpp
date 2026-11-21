/** @file board_state.cpp
 *  @brief Atomic hardware-fault flag shared by boot code and tasks.
 */

#include "bsp/board_state.hpp"

#include <atomic>

namespace bsp {

namespace {

enum class BoardState : uint8_t {
    Healthy = 0,
    HardwareFault = 1,
};

/// Cross-task flag; written with release and read with acquire so a fault
/// raised by one task is visible to the heartbeat task without other locks.
inline std::atomic<BoardState> g_board_state{BoardState::Healthy};

} // namespace

void set_hardware_fault() noexcept {
    g_board_state.store(BoardState::HardwareFault, std::memory_order_release);
}

void clear_hardware_fault() noexcept {
    g_board_state.store(BoardState::Healthy, std::memory_order_release);
}

bool is_hardware_fault_active() noexcept {
    return g_board_state.load(std::memory_order_acquire) == BoardState::HardwareFault;
}

} // namespace bsp
