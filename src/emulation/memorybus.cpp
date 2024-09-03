#include "memorybus.h"

namespace Emulation
{
	MemoryBus::MemoryBus() :
		exceptHandler(Utils::ExceptHandler::Instance()),
		controller(Controller::Instance())
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
			//PPU registers are mirrored every 8 bytes until 0x4000.
			if (isCPU && (address >= 0x2000 && address <= 0x4000))
			{
				return ppuReadCallback(address);
			}

			if (address == 0x4016)
			{
				return controller.Read();
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
			if (isCPU && (address >= 0x2000 && address <= 0x4000))
			{
				ppuWriteCallback(address, value);
				return;
			}

			if (address == 0x4016)
			{
				controller.Write(value);
				return;
			}

			std::lock_guard<std::mutex> lock(mutex);
			ram[address] = value;
		}
	}

	uint16_t MemoryBus::ReadWord(uint16_t address, bool isCPU) const
	{
		uint16_t lowByte = Read(address, isCPU);
		uint16_t highByte = Read(address + 1, isCPU);

		return (highByte << 8) | lowByte;
	}


	void MemoryBus::WriteWord(uint16_t address, uint16_t value, bool isCPU)
	{
		Write(address, value & 0xFF, isCPU);         // Write the lower byte
		Write(address + 1, (value >> 8) & 0xFF, isCPU);  // Write the upper byte
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

	void MemoryBus::RegisterPPUWriteCallback(PPUWriteCallback callback)
	{
		ppuWriteCallback = std::move(callback);
	}

	void MemoryBus::RegisterPPUReadCallback(PPUReadCallback callback)
	{
		ppuReadCallback = std::move(callback);
	}
}