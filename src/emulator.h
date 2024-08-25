#pragma once

#include "utils/logger.h"
#include "emulation/cpu.h"
#include "emulation/romreader.h"
#include "emulation/graphics/renderer.h"

#include <thread>
#include <atomic>
#include <chrono>

namespace Core
{
	class Emulator
	{
	private:
		Emulator();

	public:
		int Start();
		void RunCPU();

		static Emulator& Instance()
		{
			static Emulator INSTANCE;
			return INSTANCE;
		}
	};
}

