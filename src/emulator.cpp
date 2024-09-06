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
        testMode = false;
        showMonitor = false;
    } 

	int Emulator::Start(int argc, char* argv[])
	{
        Utils::Logger::Info("The emulator is starting..., [Monitor? - ", showMonitor, "] ");

        if (testMode)
        {
            Utils::Logger::Info("Emulator has been started in test mode.");
            cpuTest.StartTest();
            return 0;
        }

        //games/donkeykongjr.nes
        //games/test/palette_ram.nes
        //Load ROM first.
        if (romReader.LoadRom("games/pong.nes"))
        {
            //Show header of loaded ROM.
            romReader.PrintHeader();

            //Load the rom into memory.
            memoryBus.LoadPrgProgram(romReader.GetPRGRom());
            ppu.LoadCHRProgram(romReader.GetCHRRom());

            //Reset the CPU to ensure we load the correct vector.
            cpu.Reset();

            //If we are using the monitor, we have to start the clock in a seperate thread.
            if (showMonitor)
            {
                // Start Clock thread
                clockThread = std::thread(&Emulator::Loop, this);

                //Create monitor window.
                monitor.Create(argc, argv);
            }
            else
            {
                if (!renderer.InitSDL())
                {
                    Utils::Logger::Error("Failed to initialize SDL!");
                    return -1;
                }

                Loop();
            }

            Cleanup();
        }

        return 0;
	}

    void Emulator::Cleanup()
    {
        Utils::Logger::Info("Cleaning up emulator....");

        //Cleanup
        if (showMonitor)
            clockThread.join();

        renderer.Cleanup();
    }

    void Emulator::Loop()
    {
        Utils::Logger::Info("Clock thread started - ", clockThread.get_id());

        while (!exceptHandler.HasException())
        {
            cpu.Clock();

            if (ppu.frameComplete)
            {
                ppu.frameComplete = false;
                
                renderer.RenderFrame(ppu.GetScreenBuffer());

                controller.HandleInput();
            }

            //std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        Utils::Logger::Error("FATAL - Fell out of clock loop.");
    }
}