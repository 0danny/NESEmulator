#pragma once

#include "utils/logger.h"
#include "emulation/cpu.h"
#include "emulation/romreader.h"
#include "emulation/graphics/renderer.h"
#include "emulation/graphics/ppu.h"
#include "monitor/window.h"
#include "utils/excepthandler.h"
#include "emulation/memorybus.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <QThread>

using namespace Emulation;

namespace Core
{
	class Emulator
	{
	private:
		CPU& cpu;
		RomReader& romReader;
		Graphics::Renderer& renderer;
		Graphics::PPU& ppu;
		Monitor::Window& monitor;
		Utils::ExceptHandler& exceptHandler;
		MemoryBus& memoryBus;

		Emulator();

		std::thread clockThread;

	public:
		int Start(int argc, char* argv[]);
		void CreateClock();
		void Loop();

		static Emulator& Instance()
		{
			static Emulator INSTANCE;
			return INSTANCE;
		}
	};
}

