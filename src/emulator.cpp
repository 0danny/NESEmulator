#include "emulator.h"

namespace Core
{
	Emulator::Emulator() { } 

	int Emulator::Start(int argc, char* argv[])
	{
        Utils::Logger::Info("The emulator is starting...");

        auto& cpu = Emulation::CPU::Instance();
        auto& romReader = Emulation::RomReader::Instance();
        auto& renderer = Emulation::Graphics::Renderer::Instance();
        auto& monitor = Monitor::Window::Instance();

        if (!renderer.InitSDL())
        {
            Utils::Logger::Error("Failed to initialize SDL!");
            return -1;
        }

        // Load ROM first.
        if (romReader.LoadRom("games/donkeykongjr.nes"))
        {
            //Show header of loaded ROM.
            romReader.PrintHeader();

            //Load the rom into memory.
            cpu.LoadPrgProgram(romReader.GetPRGRom());

            //Start CPU & Renderer threads.
            cpu.StartThread();
            renderer.StartThread();

            //Create monitor window.
            monitor.Create(argc, argv);
            monitor.AddControls(romReader.GetPRGRom());
            monitor.Run();

            //Cleanup
            cpu.Cleanup();
            renderer.Cleanup();
        }

        return 0;
	}
}