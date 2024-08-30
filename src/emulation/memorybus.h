#pragma once

#include <array>
#include <mutex>
#include <cstdint>
#include <functional>

#include "utils/excepthandler.h"
#include "emulation/controller.h"


namespace Emulation
{
	using PPUWriteCallback = std::function<void(uint16_t address, uint8_t value)>;
	using PPUReadCallback = std::function<uint8_t(uint16_t address)>;

	class MemoryBus 
	{
	private:
		PPUWriteCallback ppuWriteCallback;
		PPUReadCallback ppuReadCallback;

		static constexpr size_t RAM_SIZE = 65536; // 64KB of RAM
		std::array<uint8_t, RAM_SIZE> ram;
		mutable std::mutex mutex;

		//Singletons
		Utils::ExceptHandler& exceptHandler;
		Controller& controller;

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
		uint8_t Read(uint16_t address, bool isCPU = true) const;
		uint16_t ReadWord(uint16_t address, bool isCPU = true) const;

		void Write(uint16_t address, uint8_t value, bool isCPU = true);
		void WriteWord(uint16_t address, uint16_t value, bool isCPU = true);

		void LoadRawProgram(const uint8_t* program, uint16_t programSize, uint16_t loadAddress);
		void LoadPrgProgram(const std::vector<uint8_t>& prgRom);

		void RegisterPPUWriteCallback(PPUWriteCallback callback);
		void RegisterPPUReadCallback(PPUReadCallback callback);
	};
}

