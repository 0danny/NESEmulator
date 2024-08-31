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
        void EmitPixel();
        void LoadCHRProgram(const std::vector<uint8_t>& chrRom);
        const uint32_t* GetScreenBuffer() const;
        bool IsFrameComplete();
        void PPUWriteCallback(uint16_t address, uint8_t value);
        uint8_t PPUReadCallback(uint16_t address);

        bool triggeredNMI = false;

        static PPU& Instance()
        {
            static PPU INSTANCE;
            return INSTANCE;
        }

    private:
        MemoryBus& memoryBus;
        static constexpr size_t VRAM_SIZE = 0x4000; // 16 KB VRAM

        // PPU memory (VRAM)
        std::array<uint8_t, VRAM_SIZE> vram;
        std::array<uint8_t, 32> paletteTable;
        std::array<uint8_t, 256> oam;
        std::array<uint32_t, 256 * 240> screenBuffer;

        // Internal state variables
        uint8_t ppuStatusCpy = 0;

        uint16_t vramAddr;
        uint16_t tempAddr;
        uint8_t fineX;
        bool writeToggle;

        uint8_t ppuReadBuffer = 0;
        uint8_t ppuReadBufferCpy = 0;

        int scanLine;
        int dot;
        int pixelIndex = 0;
        bool odd = false;

        bool frameComplete;

        uint32_t palette[64] = {
            4283716692, 4278197876, 4278718608, 4281335944, 4282646628, 4284219440, 4283696128, 4282128384,
            4280297984, 4278729216, 4278206464, 4278205440, 4278202940, 4278190080, 4278190080, 4278190080,
            4288190104, 4278734020, 4281348844, 4284227300, 4287108272, 4288681060, 4288160288, 4286069760,
            4283718144, 4280840704, 4278746112, 4278220328, 4278216312, 4278190080, 4278190080, 4278190080,
            4293717740, 4283210476, 4286086380, 4289749740, 4293154028, 4293679284, 4293683812, 4292118560,
            4288719360, 4285842432, 4283224096, 4281912428, 4281906380, 4282137660, 4278190080, 4278190080,
            4293717740, 4289252588, 4290559212, 4292129516, 4293701356, 4293701332, 4293702832, 4293182608,
            4291613304, 4290043512, 4289258128, 4288209588, 4288730852, 4288717472, 4278190080, 4278190080 };

        //$2000 PPUCTRL
        union {
            struct
            {
                unsigned baseNametableAddress : 2;
                unsigned vramAddressIncrement : 1;
                unsigned spritePatternTableAddress : 1;
                unsigned bgPatternTableAddress : 1;
                unsigned spriteSize : 1;
                unsigned ppuMasterSlaveSelect : 1;
                unsigned generateNMI : 1;
            };

            uint8_t val;
        } ppuCtrl;

        //$2001 PPUMASK
        union {
            struct
            {
                unsigned greyScale : 1;
                unsigned showBgLeftmost8 : 1;
                unsigned showSpritesLeftmost8 : 1;
                unsigned showBg : 1;
                unsigned showSprites : 1;
                unsigned emphasizeRed : 1;
                unsigned emphasizeGreen : 1;
                unsigned emphasizeBlue : 1;
            };

            uint8_t val;
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

            uint8_t val;
        } ppuStatus;

        uint8_t oamAddr = 0;    //$2003
        uint8_t oamData = 0;    //$2004
        uint8_t ppuScroll = 0;  //$2005

        uint8_t ntbyte, attrbyte, patternlow, patternhigh;
        uint16_t bgShiftRegLo;
        uint16_t bgShiftRegHi;
        uint16_t attrShiftReg1;
        uint16_t attrShiftReg2;
        uint8_t quadrant_num;

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

        void FetchTiles();

        void XIncrement();
        void YIncrement();

        bool IsRenderingDisabled();
        void CopyVerticalBits();
        void CopyHorizontalBits();
        void ReloadShiftersAndShift();
    };
}