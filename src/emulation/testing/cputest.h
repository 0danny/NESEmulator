#pragma once

#include "utils/logger.h"
#include "emulation/romreader.h"
#include "emulation/memorybus.h"
#include "emulation/cpu.h"
#include "utils/excepthandler.h"
#include "emulation/graphics/ppu.h"

#include <string>

namespace Emulation::Testing
{
	struct CPUState 
	{
		uint16_t PC;
		uint8_t A, X, Y, SP;
		uint8_t P;
		int Cycles;
		uint16_t scanLine;
		uint16_t dot;
	};

	class CPUTest
	{
	private:
		CPUTest();

		RomReader& romReader;
		MemoryBus& memoryBus;
		Graphics::PPU& ppu;
		Utils::ExceptHandler& exceptHandler;
		CPU& cpu;

		std::vector<CPUState> ParseTestLog(const std::string& filename);

		void TestStates(const std::vector<CPUState>& expectedStates);

	public:
		// Singleton access
		static CPUTest& Instance()
		{
			static CPUTest INSTANCE;
			return INSTANCE;
		}

		// Prevent copying and assignment
		CPUTest(const CPUTest&) = delete;
		CPUTest& operator=(const CPUTest&) = delete;

		void StartTest();
	};
}

