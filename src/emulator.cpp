#include "emulator.h"

namespace Core
{
	Emulator::Emulator() : 
        cpu(CPU::Instance()), 
        romReader(RomReader::Instance()), 
        renderer(Graphics::Renderer::Instance()), 
        ppu(Graphics::PPU::Instance()), 
        monitor(Monitor::Window::Instance()),
        exceptHandler(Utils::ExceptHandler::Instance()),
        memoryBus(MemoryBus::Instance())
    {

    } 

	int Emulator::Start(int argc, char* argv[])
	{
        Utils::Logger::Info("The emulator is starting...");

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
            memoryBus.LoadPrgProgram(romReader.GetPRGRom());
            cpu.Reset(); //Reset the CPU to ensure we load the correct vector.

            ppu.LoadCHRProgram(romReader.GetCHRRom());

            //Start CPU & Renderer threads.
            CreateClock();
            renderer.StartThread();

            //Create monitor window.
            monitor.Create(argc, argv);
            monitor.AddControls(romReader.GetPRGRom());
            monitor.Run();

            //Cleanup
            clockThread.join();
            renderer.Cleanup();
        }

        return 0;
	}

    void Emulator::CreateClock()
    {
        // Start Clock thread
        clockThread = std::thread(&Emulator::Loop, this);

        Utils::Logger::Info("Clock thread started - ", clockThread.get_id());
    }

    void Emulator::Loop()
    {
        while (!exceptHandler.HasException())
        {
            cpu.Clock();

            // Clock the PPU three times for every CPU cycle
            for (int i = 0; i < 3; ++i)
            {
                ppu.Clock();
            }

            if (ppu.IsFrameComplete())
            {
                renderer.RenderFrame(ppu.GetScreenBuffer());
            }

            //std::this_thread::sleep_for(std::chrono::microseconds(1));
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        Utils::Logger::Error("FATAL - Fell out of clock loop.");
    }
}