#include "cputest.h"

namespace Emulation::Testing
{
	CPUTest::CPUTest() : 
        romReader(RomReader::Instance()), 
        memoryBus(MemoryBus::Instance()),
        cpu(CPU::Instance()),
        exceptHandler(Utils::ExceptHandler::Instance())
    { 
        Utils::Logger::Info("CPU Test init.");
    }

    std::vector<CPUState> CPUTest::ParseTestLog(const std::string& filename) 
    {
        //From https://github.com/wpmed92/MedNES/blob/master/Test/CPUTest.cpp <-- Full credits

        std::vector<CPUState> states;
        std::ifstream file(filename);
        std::string line;

        while (std::getline(file, line)) 
        {
            CPUState state;

            std::stringstream lexerStream;

            //pc
            for (int i = 0; i < 4; i++) {
                lexerStream << line[i];
            }

            char* p;
            state.PC = (uint16_t)strtol(lexerStream.str().c_str(), &p, 16);
            lexerStream.str("");

            //accumulator
            lexerStream << line[50] << line[51];
            state.A = (uint8_t)strtol(lexerStream.str().c_str(), &p, 16);
            lexerStream.str("");

            //xregister
            lexerStream << line[55] << line[56];
            state.X = (uint8_t)strtol(lexerStream.str().c_str(), &p, 16);;
            lexerStream.str("");

            //yregister
            lexerStream << line[60] << line[61];
            state.Y = (uint8_t)strtol(lexerStream.str().c_str(), &p, 16);;
            lexerStream.str("");

            //statusregister
            lexerStream << line[65] << line[66];
            state.P = (uint8_t)strtol(lexerStream.str().c_str(), &p, 16);
            lexerStream.str("");

            //stackpointer
            lexerStream << line[71] << line[72];
            state.SP = (uint8_t)strtol(lexerStream.str().c_str(), &p, 16);;
            lexerStream.str("");

            //cycle
            for (int i = 90; i < line.length(); i++) {
                lexerStream << line[i];
            }

            state.CYC = (int)strtol(lexerStream.str().c_str(), &p, 10);;
            lexerStream.str("");

            states.push_back(state);
        }

        return states;
    }

    void CPUTest::StartTest()
    {
        if (romReader.LoadRom("games/nestest.nes"))
        {
            romReader.PrintHeader();
            memoryBus.LoadPrgProgram(romReader.GetPRGRom());
            cpu.Reset();

            // Set the initial PC to C000 for the automated test
            cpu.PC = 0xC000;

            // Parse the nestest log
            auto expectedStates = ParseTestLog("nestest.log");

            CPUState firstState = expectedStates.at(0);

            // Print first line of test log to verify.
            Utils::Logger::Info(
                "[Line 1] PC: ", Utils::Logger::Uint16ToHex(firstState.PC),
                " A: ", Utils::Logger::Uint8ToHex(firstState.A),
                " X: ", Utils::Logger::Uint8ToHex(firstState.X),
                " Y: ", Utils::Logger::Uint8ToHex(firstState.Y),
                " SP: ", Utils::Logger::Uint8ToHex(firstState.SP),
                " P: ", Utils::Logger::Uint8ToHex(firstState.P),
                " CYC: ", firstState.CYC
            );

            // Run the test
            TestStates(expectedStates);
        }
    }

    void CPUTest::TestStates(const std::vector<CPUState>& expectedStates) 
    {
        bool problem = false;

        for (const auto& expectedState : expectedStates) 
        {
            if (!exceptHandler.HasException())
            {
                // Compare CPU state with expected state
                bool stateMatch =
                    cpu.PC == expectedState.PC &&
                    cpu.A == expectedState.A &&
                    cpu.X == expectedState.X &&
                    cpu.Y == expectedState.Y &&
                    cpu.SP == expectedState.SP &&
                    cpu.P == expectedState.P &&
                    cpu.cycles == expectedState.CYC;

                if (!stateMatch) {
                    if (cpu.PC != expectedState.PC)
                    {
                        Utils::Logger::Error("The PC is mismatched, ours: ", static_cast<int>(cpu.PC), " | nestest: ", static_cast<int>(expectedState.PC));
                    }

                    if (cpu.A != expectedState.A)
                    {
                        Utils::Logger::Error("The A is mismatched, ours: ", static_cast<int>(cpu.A), " | nestest: ", static_cast<int>(expectedState.A));
                    }

                    if (cpu.X != expectedState.X)
                    {
                        Utils::Logger::Error("The X is mismatched, ours: ", static_cast<int>(cpu.X), " | nestest: ", static_cast<int>(expectedState.X));
                    }

                    if (cpu.Y != expectedState.Y)
                    {
                        Utils::Logger::Error("The Y is mismatched, ours: ", static_cast<int>(cpu.Y), " | nestest: ", static_cast<int>(expectedState.Y));
                    }

                    if (cpu.SP != expectedState.SP)
                    {
                        Utils::Logger::Error("The SP is mismatched, ours: ", static_cast<int>(cpu.SP), " | nestest: ", static_cast<int>(expectedState.SP));
                    }

                    if (cpu.P != expectedState.P)
                    {
                        Utils::Logger::Error("The P (Status) is mismatched, ours: ", static_cast<int>(cpu.P), " | nestest: ", static_cast<int>(expectedState.P));
                    }

                    if (cpu.cycles != expectedState.CYC)
                    {
                        Utils::Logger::Error("The Cycles is mismatched, ours: ", static_cast<int>(cpu.cycles), " | nestest: ", static_cast<int>(expectedState.CYC));
                    }

                    std::cout << std::endl;

                    std::cout << "Mismatch at PC: " << std::hex << expectedState.PC << std::endl;
                    std::cout << "Expected: "
                        << "PC:" << static_cast<int>(expectedState.PC)
                        << " A:" << static_cast<int>(expectedState.A)
                        << " X:" << static_cast<int>(expectedState.X)
                        << " Y:" << static_cast<int>(expectedState.Y)
                        << " P:" << static_cast<int>(expectedState.P)
                        << " SP:" << static_cast<int>(expectedState.SP)
                        << " CYC:" << expectedState.CYC << std::endl;
                    std::cout << "Actual:   "
                        << "PC:" << static_cast<int>(cpu.PC)
                        << " A:" << static_cast<int>(cpu.A)
                        << " X:" << static_cast<int>(cpu.X)
                        << " Y:" << static_cast<int>(cpu.Y)
                        << " P:" << static_cast<int>(cpu.P)
                        << " SP:" << static_cast<int>(cpu.SP)
                        << " CYC:" << cpu.cycles << std::endl;
                    problem = true;

                    break;
                }

                // Execute one instruction
                cpu.Clock();
            }
            else
            {
                Utils::Logger::Error("Exception in testing loop.");
                problem = true;
                break;
            }
        }

        if(!problem)
            Utils::Logger::Info("CONGRATS, YOU HAVE PASSED NESTEST!!!!!");
    }
}