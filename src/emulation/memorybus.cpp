#include "memorybus.h"

namespace Emulation
{
	MemoryBus::MemoryBus() : exceptHandler(Utils::ExceptHandler::Instance())
	{
		ram.fill(0); // Initialize RAM with zeros
	}

	uint8_t MemoryBus::Read(uint16_t address, bool isCPU) const
	{
		//Check if the read is inside bounds (uint cant be negative, but we check anyways lmao)
		if (address < 0 || address > ram.size() - 1)
		{
			exceptHandler.ThrowException("Attempted memory read outside bounds.", Utils::Logger::Uint16ToHex(address));

			return 0;
		}
		else
		{
			if (isCPU && (address >= 0x2000 && address <= 0x2007))
			{
				ppuRegisterCallback(address, 0, true);
			}

			std::lock_guard<std::mutex> lock(mutex);
			return ram[address];
		}
	}

	void MemoryBus::Write(uint16_t address, uint8_t value, bool isCPU)
	{
		// Check if the write is inside bounds
		if (address < 0 || address > ram.size() - 1)
		{
			exceptHandler.ThrowException("Attempted memory write outside bounds.", Utils::Logger::Uint16ToHex(address));
		}
		else
		{
			if (isCPU && (address >= 0x2000 && address <= 0x2007))
			{
				ppuRegisterCallback(address, value, false);
			}

			std::lock_guard<std::mutex> lock(mutex);
			ram[address] = value;
		}
	}

	uint16_t MemoryBus::ReadWord(uint16_t address, bool isCPU) const
	{
		if (address < 0 || address > ram.size() - 1)
		{
			exceptHandler.ThrowException("Attempted memory read word outside bounds.", Utils::Logger::Uint16ToHex(address));

			return 0;
		}
		else
		{
			if (isCPU && (address >= 0x2000 && address <= 0x2007))
			{
				ppuRegisterCallback(address, 0, true);
			}

			std::lock_guard<std::mutex> lock(mutex);
			return static_cast<uint16_t>(ram[address]) | (static_cast<uint16_t>(ram[address + 1]) << 8);
		}
	}

	void MemoryBus::WriteWord(uint16_t address, uint16_t value, bool isCPU)
	{
		// Check if the write is inside bounds
		if (address < 0 || address > ram.size() - 2)  // Check if there's enough space for both bytes
		{
			exceptHandler.ThrowException("Attempted memory write word outside bounds.", Utils::Logger::Uint16ToHex(address));
		}
		else
		{
			if (isCPU && (address >= 0x2000 && address <= 0x2007))
			{
				ppuRegisterCallback(address, value, false);
			}

			std::lock_guard<std::mutex> lock(mutex);
			ram[address] = value & 0xFF;
			ram[address + 1] = (value >> 8) & 0xFF;
		}
	}

	void MemoryBus::LoadRawProgram(const uint8_t* program, uint16_t programSize, uint16_t loadAddress)
	{
		for (uint16_t i = 0; i < programSize; ++i)
		{
			ram[loadAddress + i] = program[i];
		}

		// Set the reset vector to point to the program's start address
		WriteWord(0xFFFC, loadAddress);
	}

	void MemoryBus::LoadPrgProgram(const std::vector<uint8_t>& prgRom)
	{
		// Assume PRG ROM is always loaded at 0x8000
		size_t prgSize = prgRom.size();

		if (prgSize == 16384) // 16 KB PRG ROM
		{
			Utils::Logger::Info("16 KB ROM, Mirroring...");

			// Load the PRG ROM into both 0x8000-0xBFFF and 0xC000-0xFFFF
			std::copy(prgRom.begin(), prgRom.end(), ram.begin() + 0x8000);
			std::copy(prgRom.begin(), prgRom.end(), ram.begin() + 0xC000);
		}
		else if (prgSize == 32768) // 32 KB PRG ROM
		{
			// Load the PRG ROM into 0x8000-0xFFFF
			std::copy(prgRom.begin(), prgRom.end(), ram.begin() + 0x8000);

			Utils::Logger::Info("32 KB ROM, Loading...");
		}
		else
		{
			Utils::Logger::Error("Unexpected PRG ROM size!");
		}
	}

	void MemoryBus::RegisterPPURegisterCallback(PPURegisterCallback callback)
	{
		ppuRegisterCallback = std::move(callback);
	}
}