#include "romreader.h"

namespace Emulation
{
	RomReader::RomReader() : header(iNESHeader()) { }

	bool RomReader::LoadRom(const std::string filePath)
	{
		// Check if the file path exists.
		std::ifstream romFile(filePath, std::ios::binary);

		if (!romFile)
		{
			Utils::Logger::Error("Failed to open ROM file - ", filePath);
			return false;
		}

		// Read the header.
		if (!ReadHeader(romFile))
		{
			Utils::Logger::Error("ROM does not contain magic signature NES.");
			return false;
		}

		// Get the rom name by splitting at last /
		romName = filePath.substr(filePath.find_last_of('/', filePath.length()) + 1, filePath.length());

		// Look for the enum for the mapper.
		int mapperNum = ((header.flags7 & 0xF0) | (header.flags6 >> 4));

		if (mapperNum < 0 && mapperNum > mappers->size())
		{
			Utils::Logger::Error("Mapper number ", mapperNum, " not supported.");
			return false;
		}

		Utils::Logger::Info("Using memory mapper ", mappers[mapperNum]);

		// Load PRG ROM data

		/*
			PRG ROM stands for "Program Read-Only Memory." 
			This is the portion of the ROM that contains the game’s program code and data. 
			It holds all the instructions that the CPU (6502 processor) executes during the game.
		*/

		prgRom.resize(header.prgRomSize * 16 * 1024);
		romFile.read(reinterpret_cast<char*>(prgRom.data()), prgRom.size());

		// Load CHR ROM data

		/*
			CHR ROM stands for "Character Read-Only Memory." 
			This part of the ROM contains the graphical data used by the PPU (Picture Processing Unit) to render the game’s visuals.
		*/

		if (header.chrRomSize > 0)
		{
			chrRom.resize(header.chrRomSize * 8 * 1024);
			romFile.read(reinterpret_cast<char*>(chrRom.data()), chrRom.size());
		}

		return true;
	}

	const std::vector<uint8_t>& RomReader::GetPRGRom() const
	{
		return prgRom;
	}

	const iNESHeader& RomReader::GetHeader() const
	{
		return header;
	}

	bool RomReader::ReadHeader(std::ifstream& romFile)
	{
		romFile.read(reinterpret_cast<char*>(&header), sizeof(header));

		// Validate the header signature
		if (header.signature[0] != 'N' || header.signature[1] != 'E' ||
			header.signature[2] != 'S' || header.signature[3] != 0x1A)
		{
			return false;
		}

		return true;
	}

	void RomReader::PrintHeader() const
	{
		Utils::Logger::Info("----- iNES Header Information -----");
		Utils::Logger::Info("Rom Name: ", romName);
		Utils::Logger::Info("Signature: ", header.signature[0], header.signature[1], header.signature[2],
			" (0x", std::hex, std::uppercase, int(header.signature[3]), ")");
		Utils::Logger::Info("PRG ROM Size: ", std::dec, int(header.prgRomSize), " x 16 KB");
		Utils::Logger::Info("CHR ROM Size: ", std::dec, int(header.chrRomSize), " x 8 KB");
		Utils::Logger::Info("Mapper Number: ", ((header.flags7 & 0xF0) | (header.flags6 >> 4)));
		Utils::Logger::Info("Mirroring: ", ((header.flags6 & 0x01) ? "Vertical" : "Horizontal"));
		Utils::Logger::Info("Battery-backed RAM: ", ((header.flags6 & 0x02) ? "Yes" : "No"));
		Utils::Logger::Info("Trainer: ", ((header.flags6 & 0x04) ? "Yes" : "No"));
		Utils::Logger::Info("Four-screen VRAM: ", ((header.flags6 & 0x08) ? "Yes" : "No"));
		Utils::Logger::Info("PRG RAM Size: ", std::dec, int(header.prgRamSize), " x 8 KB");
		Utils::Logger::Info("TV System: ", ((header.flags9 & 0x01) ? "PAL" : "NTSC"));
		Utils::Logger::Info("-----------------------------------");
	}
}