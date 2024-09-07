#pragma once
#include "utils/logger.h"
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

        bool triggeredNMI = false;
        bool frameComplete = false;

        int scanLine = 0;
        int dot = 0;

        static PPU& Instance()
        {
            static PPU INSTANCE;
            return INSTANCE;
        }

    private:
        MemoryBus& memoryBus;

        static constexpr size_t VRAM_SIZE = 0x4000; // 16 KB VRAM

        // PPU register addresses
        static constexpr uint16_t PPUCTRL = 0;
        static constexpr uint16_t PPUMASK = 1;
        static constexpr uint16_t PPUSTATUS = 2;
        static constexpr uint16_t OAMADDR = 3;
        static constexpr uint16_t OAMDATA = 4;
        static constexpr uint16_t PPUSCROLL = 5;
        static constexpr uint16_t PPUADDR = 6;
        static constexpr uint16_t PPUDATA = 7;

        // PPU memory (VRAM)
        std::array<uint8_t, VRAM_SIZE> vram;
        std::array<uint32_t, 256 * 240> screenBuffer;

        std::array<uint8_t, 256> primaryOam;


        uint32_t palette[64] = {
            4283716692, 4278197876, 4278718608, 4281335944, 4282646628, 4284219440, 4283696128, 4282128384,
            4280297984, 4278729216, 4278206464, 4278205440, 4278202940, 4278190080, 4278190080, 4278190080,
            4288190104, 4278734020, 4281348844, 4284227300, 4287108272, 4288681060, 4288160288, 4286069760,
            4283718144, 4280840704, 4278746112, 4278220328, 4278216312, 4278190080, 4278190080, 4278190080,
            4293717740, 4283210476, 4286086380, 4289749740, 4293154028, 4293679284, 4293683812, 4292118560,
            4288719360, 4285842432, 4283224096, 4281912428, 4281906380, 4282137660, 4278190080, 4278190080,
            4293717740, 4289252588, 4290559212, 4292129516, 4293701356, 4293701332, 4293702832, 4293182608,
            4291613304, 4290043512, 4289258128, 4288209588, 4288730852, 4288717472, 4278190080, 4278190080 };

        // Memory Mapped Registers

        //$2000 PPUCTRL
        union {
            struct
            {
                unsigned nametableSelect : 2;
                unsigned vramIncrementMode : 1;
                unsigned spriteTileSelect : 1;
                unsigned bgTileSelect : 1;
                unsigned spriteHeight : 1;
                unsigned ppuMasterSlave : 1;
                unsigned nmiEnable : 1;
            };

            uint8_t val = 0;
        } ppuCtrl;

        //$2001 PPUMASK
        union {
            struct
            {
                unsigned greyScale : 1;
                unsigned showBgLeftColumn : 1;
                unsigned showSpriteLeftColumn : 1;
                unsigned showBg : 1;
                unsigned showSprites : 1;
                unsigned emphasizeRed : 1;
                unsigned emphasizeGreen : 1;
                unsigned emphasizeBlue : 1;
            };

            uint8_t val = 0;
        } ppuMask;

        //$2002 PPUSTATUS
        union {
            struct
            {
                unsigned leastSignificantBits : 5;
                unsigned spriteOverflow : 1;
                unsigned spriteZeroHit : 1;
                unsigned vBlank : 1;
            };

            uint8_t val = 0;
        } ppuStatus;

        uint8_t oamAddr = 0;    //$2003
        uint8_t oamData = 0;    //$2004
        uint8_t oamDma = 0;  //$4014

        uint8_t readBuffer = 0;

        //Internal Registers
        
        // 15-bit registers (v and t)
        uint16_t vramAddr = 0;  // Current VRAM address (15 bits)
        uint16_t tempAddr = 0;  // Temporary VRAM address (15 bits)

        // 3-bit register (x)
        uint8_t fineX = 0;   // Fine X scroll (3 bits)

        // 1-bit flag (w)
        bool writeToggle = false;      // Write toggle (1 bit)

        // New members for background rendering
        uint16_t bgShiftRegister[2] = { 0 };  // Background pattern table shift registers
        uint8_t bgAttributeShiftRegister[2] = { 0 };  // Background attribute shift registers
        uint8_t bgNextTileId = 0;
        uint8_t bgNextTileAttribute = 0;
        uint8_t bgNextTileLsb = 0;
        uint8_t bgNextTileMsb = 0;

        uint8_t ReadVRAM(uint16_t addr) const;
        void WriteVRAM(uint16_t addr, uint8_t value);

        uint16_t MirrorAddress(uint16_t addr) const;

        // Clocking
        void PreRender();
        void Rendering();
        void Vblank();

        void FetchTiles();
        void EmitPixel();

        uint16_t GetBackgroundPatternAddress(uint8_t tileId);
        uint16_t GetAttributeAddress();
        uint8_t GetColorFromPaletteRam(uint8_t palette, uint8_t pixel);

        void CopyHorizontal();
        void CopyVertical();

        void PPUWriteCallback(uint16_t address, uint8_t value);
        uint8_t PPUReadCallback(uint16_t address);

        void IncrementVRAMAddr();
        bool ForcedBlanking();
    };
}