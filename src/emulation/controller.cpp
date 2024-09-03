#include "controller.h"

namespace Emulation
{
    Controller::Controller() 
    { }

	void Controller::HandleInput()
	{
        while (SDL_PollEvent(&event)) 
        {
            switch (event.type) 
            {

            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) 
                {
                case SDLK_a: controller1.buttons[0] = 1; break; // A button
                case SDLK_s: controller1.buttons[1] = 1; break; // B button
                case SDLK_RETURN: controller1.buttons[2] = 1; break; // Select button
                case SDLK_SPACE: controller1.buttons[3] = 1; break; // Start button
                case SDLK_UP: controller1.buttons[4] = 1; break; // Up button
                case SDLK_DOWN: controller1.buttons[5] = 1; break; // Down button
                case SDLK_LEFT: controller1.buttons[6] = 1; break; // Left button
                case SDLK_RIGHT: controller1.buttons[7] = 1; break; // Right button
                }
                break;

            case SDL_KEYUP:
                switch (event.key.keysym.sym) 
                {
                case SDLK_a: controller1.buttons[0] = 0; break; // A button
                case SDLK_s: controller1.buttons[1] = 0; break; // B button
                case SDLK_RETURN: controller1.buttons[2] = 0; break; // Select button
                case SDLK_SPACE: controller1.buttons[3] = 0; break; // Start button
                case SDLK_UP: controller1.buttons[4] = 0; break; // Up button
                case SDLK_DOWN: controller1.buttons[5] = 0; break; // Down button
                case SDLK_LEFT: controller1.buttons[6] = 0; break; // Left button
                case SDLK_RIGHT: controller1.buttons[7] = 0; break; // Right button
                }
                break;

            }
        }
	}

    void Controller::Write(uint8_t data) 
    {
        //Utils::Logger::Info("Write called.");

        strobe = data & 1;

        if (strobe) 
        {
            controller1.shiftRegister = 
                (controller1.buttons[7] << 7) |
                (controller1.buttons[6] << 6) |
                (controller1.buttons[5] << 5) |
                (controller1.buttons[4] << 4) |
                (controller1.buttons[3] << 3) |
                (controller1.buttons[2] << 2) |
                (controller1.buttons[1] << 1) |
                controller1.buttons[0];
        }
    }

    // Function to handle reads from 0x4016
    uint8_t Controller::Read() 
    {
        //Utils::Logger::Info("Read called.");

        uint8_t result = controller1.shiftRegister & 1;
        if (!strobe) 
        {
            controller1.shiftRegister >>= 1;  // Shift to the next button
        }

        return result;
    }
}
