#pragma once

#include <array>
#include <mutex>
#include <cstdint>

#include "utils/excepthandler.h"

namespace Emulation
{
	class MemoryBus 
	{
	private:
		static constexpr size_t RAM_SIZE = 65536; // 64KB of RAM
		std::array<uint8_t, RAM_SIZE> ram;
		mutable std::mutex mutex;

		//Singletons
		Utils::ExceptHandler& exceptHandler;

		MemoryBus();

	public:
		// Singleton access
		static MemoryBus& Instance() 
		{
			static MemoryBus INSTANCE;
			return INSTANCE;
		}

		// Prevent copying and assignment
		MemoryBus(const MemoryBus&) = delete;
		MemoryBus& operator=(const MemoryBus&) = delete;

		// Thread-safe read method
		uint8_t Read(uint16_t address) const;

		// Thread-safe write method
		void Write(uint16_t address, uint8_t value);

		// Thread-safe read word method (16-bit)
		uint16_t ReadWord(uint16_t address) const;

		// Thread-safe write word method (16-bit)
		void WriteWord(uint16_t address, uint16_t value);

		void LoadRawProgram(const uint8_t* program, uint16_t programSize, uint16_t loadAddress);
		void LoadPrgProgram(const std::vector<uint8_t>& prgRom);
	};
}

