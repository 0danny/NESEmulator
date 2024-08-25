#include "emulator.h"

namespace Core
{
	Emulator::Emulator() { } 

	int Emulator::Start()
	{
        Utils::Logger::Info("The emulator is starting...");

        auto& cpu = Emulation::CPU::Instance();
        auto& romReader = Emulation::RomReader::Instance();
        auto& renderer = Emulation::Graphics::Renderer::Instance();

        if (!renderer.InitSDL())
        {
            Utils::Logger::Error("Failed to initialize SDL!");
            return -1;
        }

        // Load ROM first.
        if (romReader.LoadRom("games/donkeykong.nes"))
        {
            romReader.PrintHeader();
        }

        cpu.LoadPrgProgram(romReader.GetPRGRom());

        // Start CPU thread
        std::thread cpuThread(&Emulator::RunCPU, this);

        // Run renderer in main thread
        renderer.Loop();

        cpuThread.join();

        renderer.Cleanup();

        return 0;
	}

    void Emulator::RunCPU()
    {
        Utils::Logger::Info("Launching 6502 CPU...");

        auto& cpu = Emulation::CPU::Instance();

        while (cpu.IsClocking())
        {
            cpu.Clock();

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}