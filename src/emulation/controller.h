#pragma once

#include "utils/logger.h"

#include <SDL.h>
#undef main

namespace Emulation
{
	struct ControllerState 
	{
		uint8_t buttons[8];  // A, B, Select, Start, Up, Down, Left, Right
		uint8_t shiftRegister;  // For reading the button states
	};

	class Controller
	{
	public:
		Controller();

		void HandleInput();

		void Write(uint8_t data);

		uint8_t Read();

		static Controller& Instance()
		{
			static Controller INSTANCE;
			return INSTANCE;
		}

	private:
		SDL_Event event;
		ControllerState controller1;
		uint8_t strobe = 0;
	};
}