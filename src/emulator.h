#pragma once

#include "utils/logger.h"
#include "emulation/cpu.h"

namespace Core
{
	class Emulator
	{
	private:
		Emulator();

	public:
		int Start();

		static Emulator& Instance()
		{
			static Emulator INSTANCE;
			return INSTANCE;
		}
	};
}

