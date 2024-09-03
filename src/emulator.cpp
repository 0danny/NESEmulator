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
        if (romReader.LoadRom("games/donkeykongjr.nes"))
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

    #include <chrono>
    #include <thread>

    void Emulator::Loop()
    {
        while (!exceptHandler.HasException())
        {
            cpu.Clock();

            if (ppu.frameComplete)
            {
                ppu.frameComplete = false;

                
                    //Utils::Logger::Debug("Not in vblank, rendering. (Vblank: ", ppu.ppuStatus.vBlank, ")");

                renderer.RenderFrame(ppu.GetScreenBuffer());
                
                
                Utils::Logger::Debug("Frame processed , [Scanline ", ppu.scanLine, " , Dot ", ppu.dot, "]");

                    

                //Check for input.
                controller.HandleInput();
            }

            //std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        Utils::Logger::Error("FATAL - Fell out of clock loop.");
    }
}