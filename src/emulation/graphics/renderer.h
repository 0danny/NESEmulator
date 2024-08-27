#pragma once

#include "utils/logger.h"

#include <SDL.h>
#undef main

namespace Emulation::Graphics
{
	class Renderer
	{
	public:
		Renderer();

		bool InitSDL();
		void RenderFrame(const uint32_t* pixelBuffer);
		void Cleanup();
		void Loop();
		void StartThread();

		static Renderer& Instance()
		{
			static Renderer INSTANCE;
			return INSTANCE;
		}

	private:
		static constexpr int SCREEN_WIDTH = 256;
		static constexpr int SCREEN_HEIGHT = 240;

		SDL_Window* window = nullptr;
		SDL_Renderer* renderer = nullptr;
		SDL_Texture* texture = nullptr;

		std::thread rendererThread;

		uint32_t pixelBuffer[SCREEN_WIDTH * SCREEN_HEIGHT] = { 0 };
	};
}