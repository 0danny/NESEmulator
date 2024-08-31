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
        memoryBus(MemoryBus::Instance()),
        cpuTest(Testing::CPUTest::Instance()),
        controller(Controller::Instance())
    {
        testMode = true;
        showMonitor = true;
    } 

	int Emulator::Start(int argc, char* argv[])
	{
        Utils::Logger::Info("The emulator is starting...");

        if (testMode)
        {
            Utils::Logger::Info("Emulator has been started in test mode.");
            cpuTest.StartTest();
            return 0;
        }

        if (!renderer.InitSDL())
        {
            Utils::Logger::Error("Failed to initialize SDL!");
            return -1;
        }

        // Load ROM first.
        if (romReader.LoadRom("games/pong.nes"))
        {
            //Show header of loaded ROM.
            romReader.PrintHeader();

            //Load the rom into memory.
            memoryBus.LoadPrgProgram(romReader.GetPRGRom());
            ppu.LoadCHRProgram(romReader.GetCHRRom());

            cpu.Reset(); //Reset the CPU to ensure we load the correct vector.

            // Start Clock thread
            clockThread = std::thread(&Emulator::Loop, this);
            Utils::Logger::Info("Clock thread started - ", clockThread.get_id());

            if (showMonitor)
            {
                //Create monitor window.
                monitor.Create(argc, argv);
                monitor.AddControls(romReader.GetPRGRom());
                monitor.Run();
            }

            //Cleanup
            clockThread.join();
            renderer.Cleanup();
        }

        return 0;
	}

    void Emulator::Loop()
    {
        while (!exceptHandler.HasException())
        {
            cpu.Clock();

            if (ppu.IsFrameComplete())
            {
                renderer.RenderFrame(ppu.GetScreenBuffer());

                //Check for input.
                controller.HandleInput();
            }
        }

        Utils::Logger::Error("FATAL - Fell out of clock loop.");
    }
}