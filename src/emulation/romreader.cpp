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
		std::cout << "----- iNES Header Information -----" << std::endl;
		std::cout << "Rom Name: " << romName << std::endl;
		std::cout << "Signature: " << header.signature[0] << header.signature[1] << header.signature[2] << " (0x"
			<< std::hex << std::uppercase << int(header.signature[3]) << ")" << std::endl;
		std::cout << "PRG ROM Size: " << std::dec << int(header.prgRomSize) << " x 16 KB" << std::endl;
		std::cout << "CHR ROM Size: " << std::dec << int(header.chrRomSize) << " x 8 KB" << std::endl;
		std::cout << "Mapper Number: " << ((header.flags7 & 0xF0) | (header.flags6 >> 4)) << std::endl;
		std::cout << "Mirroring: " << ((header.flags6 & 0x01) ? "Vertical" : "Horizontal") << std::endl;
		std::cout << "Battery-backed RAM: " << ((header.flags6 & 0x02) ? "Yes" : "No") << std::endl;
		std::cout << "Trainer: " << ((header.flags6 & 0x04) ? "Yes" : "No") << std::endl;
		std::cout << "Four-screen VRAM: " << ((header.flags6 & 0x08) ? "Yes" : "No") << std::endl;
		std::cout << "PRG RAM Size: " << std::dec << int(header.prgRamSize) << " x 8 KB" << std::endl;
		std::cout << "TV System: " << ((header.flags9 & 0x01) ? "PAL" : "NTSC") << std::endl;
		std::cout << "-----------------------------------" << std::endl;
	}
}