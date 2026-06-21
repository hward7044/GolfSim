#pragma once
#include <cstdint>

// =============================================================================
// OV9281 I2C Register Map (Public Subset)
// Source: Linux kernel ov9281.c drivers (Rockchip, Raspberry Pi)
// =============================================================================
namespace OV9281Reg {

    // --- Exposure Control (AEC Manual) ---
    constexpr uint16_t EXPOSURE_HI   = 0x3500;  // Exposure[19:16]
    constexpr uint16_t EXPOSURE_MID  = 0x3501;  // Exposure[15:8]
    constexpr uint16_t EXPOSURE_LO   = 0x3502;  // Exposure[7:0]

    // --- Gain Control (AGC Manual) ---
    constexpr uint16_t GAIN_HI       = 0x3508;
    constexpr uint16_t GAIN_LO       = 0x3509;

    // --- Frame Timing (VTS) ---
    constexpr uint16_t VTS_HI        = 0x380E;
    constexpr uint16_t VTS_LO        = 0x380F;

    // --- Strobe / VSYNC Control ---
    constexpr uint16_t VSYNC_DELAY_1 = 0x4314;
    constexpr uint16_t VSYNC_DELAY_2 = 0x4315;
    constexpr uint16_t VSYNC_DELAY_3 = 0x4316;
    constexpr uint16_t VSYNC_WIDTH_1 = 0x4311;
    constexpr uint16_t VSYNC_WIDTH_2 = 0x4312;

    // --- Streaming Control ---
    constexpr uint16_t STREAM_ON     = 0x0100;  // Write 0x01 to start, 0x00 to stop

} // namespace OV9281Reg
