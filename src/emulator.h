#pragma once

#include "utils/logger.h"
#include "emulation/cpu.h"
#include "emulation/romreader.h"
#include "emulation/graphics/renderer.h"
#include "monitor/window.h"

#include <thread>
#include <atomic>
#include <chrono>
#include <QThread>

namespace Core
{
	class Emulator
	{
	private:
		Emulator();

	public:
		int Start(int argc, char* argv[]);

		static Emulator& Instance()
		{
			static Emulator INSTANCE;
			return INSTANCE;
		}
	};
}

