#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "i2c_interface.hpp"

namespace test {

/**
 * @brief Single emulated I2C device: register file plus presence flag.
 *
 * Register reads use only the 7-bit address portion, so multi-byte reads with
 * the auto-increment bit (0x80) set behave like the real device.
 */
struct FakeI2cDevice {
    uint8_t address{0};
    bool present{false};
    uint8_t regs[128]{};

    uint8_t& at(uint8_t reg) { return regs[reg & 0x7FU]; }
    const uint8_t& at(uint8_t reg) const { return regs[reg & 0x7FU]; }
};

/**
 * @brief Bus fake for unit tests: canned registers and simulated NACKs.
 *
 * Drivers and POST logic run against this fake exactly as they run against
 * bsp::I2cBus on the target, without any HAL involvement.
 */
class FakeI2cBus final : public bsp::II2cBus {
public:
    static constexpr std::size_t kMaxDevices = 8;

    /** @brief Adds (or reconfigures) a present device and clears its registers. */
    FakeI2cDevice& add_device(uint8_t address) {
        FakeI2cDevice* slot = find(address);
        if (slot == nullptr) {
            slot = &devices_[device_count_++];
        }
        *slot = FakeI2cDevice{};
        slot->address = address;
        slot->present = true;
        return *slot;
    }

    /** @brief Marks a device absent so it NACKs every transaction. */
    void remove_device(uint8_t address) {
        FakeI2cDevice* slot = find(address);
        if (slot != nullptr) {
            slot->present = false;
        }
    }

    bool is_device_ready(uint8_t dev_addr_7bit, uint32_t, uint32_t) noexcept override {
        const FakeI2cDevice* slot = find(dev_addr_7bit);
        return slot != nullptr && slot->present;
    }

    bool write_reg(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t value, uint32_t) noexcept override {
        FakeI2cDevice* slot = find(dev_addr_7bit);
        if (slot == nullptr || !slot->present) {
            return false;
        }
        slot->at(reg_addr) = value;
        return true;
    }

    bool read_reg(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t& out_value, uint32_t) noexcept override {
        const FakeI2cDevice* slot = find(dev_addr_7bit);
        if (slot == nullptr || !slot->present) {
            return false;
        }
        out_value = slot->at(reg_addr);
        return true;
    }

    bool read_regs(uint8_t dev_addr_7bit,
                   uint8_t start_reg_addr,
                   uint8_t* buffer,
                   std::size_t length,
                   uint32_t) noexcept override {
        const FakeI2cDevice* slot = find(dev_addr_7bit);
        if (slot == nullptr || !slot->present) {
            return false;
        }
        const uint8_t base = start_reg_addr & 0x7FU;
        for (std::size_t i = 0; i < length; ++i) {
            buffer[i] = slot->regs[(base + i) & 0x7FU];
        }
        return true;
    }

private:
    FakeI2cDevice* find(uint8_t address) {
        for (std::size_t i = 0; i < device_count_; ++i) {
            if (devices_[i].address == address) {
                return &devices_[i];
            }
        }
        return nullptr;
    }

    const FakeI2cDevice* find(uint8_t address) const {
        for (std::size_t i = 0; i < device_count_; ++i) {
            if (devices_[i].address == address) {
                return &devices_[i];
            }
        }
        return nullptr;
    }

    FakeI2cDevice devices_[kMaxDevices]{};
    std::size_t device_count_{0};
};

} // namespace test
