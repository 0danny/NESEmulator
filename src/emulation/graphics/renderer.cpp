#include "renderer.h"

namespace Emulation::Graphics
{
	Renderer::Renderer() { }

	bool Renderer::InitSDL()
	{
        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
            Utils::Logger::Error("SDL could not initialize! SDL_Error - ", SDL_GetError());
            return false;
        }

        window = SDL_CreateWindow("NES Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
        if (window == nullptr)
        {
            Utils::Logger::Error("Window could not be created! SDL_Error - ", SDL_GetError());
            return false;
        }

        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if (renderer == nullptr)
        {
            Utils::Logger::Error("Renderer could not be created! SDL_Error - ", SDL_GetError());
            return false;
        }

        texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);
        if (texture == nullptr)
        {
            Utils::Logger::Error("Texture could not be created! SDL_Error - ", SDL_GetError());
            return false;
        }

        return true;
	}

    void Renderer::StartThread()
    {
        // Start CPU thread
        rendererThread = std::thread(&Renderer::Loop, this);

        Utils::Logger::Info("Renderer thread started - ", rendererThread.get_id());
    }

    void Renderer::Loop()
    {
        bool quit = false;
        SDL_Event e;

        while (!quit)
        {
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
            }

            for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
            {
                pixelBuffer[i] = 0xFF0000FF;  // Example: fill the screen with blue.
            }

            RenderFrame(pixelBuffer);

            SDL_Delay(16);
        }
    }

    void Renderer::RenderFrame(uint32_t* pixelBuffer)
    {
        void* pixels;
        int pitch;

        if (SDL_LockTexture(texture, nullptr, &pixels, &pitch) != 0)
        {
            Utils::Logger::Error("Unable to lock texture - ", SDL_GetError());
            return;
        }

        memcpy(pixels, pixelBuffer, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint32_t));

        SDL_UnlockTexture(texture);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    void Renderer::Cleanup()
    {
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
}