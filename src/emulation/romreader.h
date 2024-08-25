#pragma once

#include <stdint.h>
#include <string>
#include <iostream>
#include "utils/logger.h"

namespace Emulation
{
    struct iNESHeader
    {
        char signature[4];      // "NES" followed by 0x1A
        uint8_t prgRomSize;     // Size of PRG ROM in 16 KB units
        uint8_t chrRomSize;     // Size of CHR ROM in 8 KB units
        uint8_t flags6;         // Flags 6: Mapper, mirroring, battery, trainer
        uint8_t flags7;         // Flags 7: Mapper, VS/PlayChoice, NES 2.0
        uint8_t prgRamSize;     // Size of PRG RAM in 8 KB units (usually unused)
        uint8_t flags9;         // Flags 9: TV system (NTSC/PAL)
        uint8_t flags10;        // Flags 10: TV system, PRG RAM, unofficial uses
        uint8_t padding[5];     // Unused padding bytes (should be zero)
    };

	class RomReader
	{
	public:
		RomReader();

        bool LoadRom(const std::string filepath);
        const iNESHeader& GetHeader() const;
        void PrintHeader() const;
        const std::vector<uint8_t>& GetPRGRom() const;

        static RomReader& Instance()
        {
            static RomReader INSTANCE;
            return INSTANCE;
        }

    private:
        iNESHeader header;       // Struct to hold the iNES header info
        std::vector<uint8_t> prgRom; // PRG ROM data
        std::vector<uint8_t> chrRom; // CHR ROM data
        std::string romName;

        bool ReadHeader(std::ifstream& romFile);
	};
}