#pragma once
#include "utils/logger.h"
#include "emulation/cpu.h"
#include "emulation/memorybus.h"
#include <array>
#include <cstdint>

namespace Emulation::Graphics
{
    class PPU
    {
    public:
        PPU();
        void Clock();
        void LoadCHRProgram(const std::vector<uint8_t>& chrRom);
        const uint32_t* GetScreenBuffer() const;
        bool IsFrameComplete();

        static PPU& Instance()
        {
            static PPU INSTANCE;
            return INSTANCE;
        }

    private:
        MemoryBus& memoryBus;
        static constexpr size_t VRAM_SIZE = 0x2000; // 8 KB VRAM

        // PPU memory (VRAM)
        std::array<uint8_t, VRAM_SIZE> vram;
        std::array<uint8_t, 32> paletteTable;
        std::array<uint8_t, 0x100> palette;
        std::array<uint8_t, 256> oam;
        std::array<uint32_t, 256 * 240> screenBuffer;

        // Internal state variables
        uint16_t vramAddr;
        uint16_t tempAddr;
        uint8_t fineX;
        uint8_t writeToggle;
        uint8_t buffer;
        int scanline;
        int cycle;

        bool frameComplete;

        // Helper methods for reading/writing PPU registers
        uint8_t ReadPPURegister(uint16_t addr) const;
        void WritePPURegister(uint16_t addr, uint8_t value);

        uint8_t ReadVRAM(uint16_t addr) const;
        void WriteVRAM(uint16_t addr, uint8_t value);

        // PPU register addresses
        static constexpr uint16_t PPUCTRL = 0x2000;
        static constexpr uint16_t PPUMASK = 0x2001;
        static constexpr uint16_t PPUSTATUS = 0x2002;
        static constexpr uint16_t OAMADDR = 0x2003;
        static constexpr uint16_t OAMDATA = 0x2004;
        static constexpr uint16_t PPUSCROLL = 0x2005;
        static constexpr uint16_t PPUADDR = 0x2006;
        static constexpr uint16_t PPUDATA = 0x2007;

        void IncrementVRAMAddr();
        void HandleSpriteEvaluation();
        void RenderPixel();
        uint8_t FetchBackgroundPixel(uint8_t x, uint8_t y);
        uint8_t FetchSpritePixel(uint8_t x, uint8_t y);
    };
}