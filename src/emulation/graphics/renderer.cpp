#include "renderer.h"

namespace Emulation::Graphics
{
    Renderer::Renderer() : scaleFactor(3) { }

	bool Renderer::InitSDL()
	{
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)
        {
            Utils::Logger::Error("SDL could not initialize! SDL_Error - ", SDL_GetError());
            return false;
        }

        int scaledWidth = SCREEN_WIDTH * scaleFactor;
        int scaledHeight = SCREEN_HEIGHT * scaleFactor;

        window = SDL_CreateWindow("NES Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, scaledWidth, scaledHeight, SDL_WINDOW_SHOWN);
        if (window == nullptr)
        {
            Utils::Logger::Error("Window could not be created! SDL_Error - ", SDL_GetError());
            return false;
        }

        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (renderer == nullptr)
        {
            Utils::Logger::Error("Renderer could not be created! SDL_Error - ", SDL_GetError());
            return false;
        }

        texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, SCREEN_WIDTH, SCREEN_HEIGHT);
        if (texture == nullptr)
        {
            Utils::Logger::Error("Texture could not be created! SDL_Error - ", SDL_GetError());
            return false;
        }

        return true;
	}

    void Renderer::RenderFrame(const uint32_t* pixelBuffer)
    {
        SDL_Rect srcRect = { 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT };
        SDL_Rect destRect = { 0, 0, SCREEN_WIDTH * scaleFactor, SCREEN_HEIGHT * scaleFactor };

        SDL_UpdateTexture(texture, NULL, pixelBuffer, SCREEN_WIDTH * sizeof(Uint32));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, &srcRect, &destRect);
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