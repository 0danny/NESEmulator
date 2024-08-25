#include "emulator.h"

namespace Core
{
	Emulator::Emulator() { } 

	int Emulator::Start()
	{
		Utils::Logger::Info("The emulator is starting...");

		auto& cpu = Emulation::CPU::Instance();
		auto& romReader = Emulation::RomReader::Instance();

		// Example Program
		uint8_t program[] = 
		{
			0xA2, 0x00,       // LDX #$00 - Load 0 into X
			0x8E, 0x00, 0x02, // STX $0200 - Store the value of X at 0x200 in Memory
			0xE8,             // INX - Increment X
			0x4C, 0x02, 0x80  // JMP $8002 - Jump to the STX
		};

		uint8_t program2[] = {
			//0xA9, 0x10,       // 8000: LDA #$10  - Load 0x10 into A register
			0x85, 0x10,       // 8002: STA $10   - Store A into memory location $0010
			0xA2, 0x05,       // 8004: LDX #$05  - Load 0x05 into X register
			0x8E, 0x00, 0x02, // 8006: STX $0200 - Store X at memory location $0200
			0xE8,             // 8009: INX       - Increment X by 1
			0x6D, 0x13, 0x00, // 800A: ADC $0010 - Add value at $0010 to A (A = A + $0010 + Carry)
			0xF0, 0x03,       // 800D: BEQ $8012 - Branch to $8012 if the result is zero
			0x4C, 0x09, 0x80, // 800F: JMP $8009 - Jump back to $8009 (INX instruction)
			0xEA,             // 8012: NOP       - No operation (does nothing, takes 2 cycles)
			0xE8,             // 8013: INX       - Increment X by 1
			0xF0, 0xF3,       // 8014: BEQ $8009 - Branch to $8009 if X is zero
			0x00              // 8016: BRK       - Break (forces an interrupt)
		};

		uint8_t program3[] = {
			0xA9, 0x3C,       // 8000: LDA #$3C  - Load 0x3C into A register
			0x85, 0x10,       // 8002: STA $0010 - Store A into memory location $0010
			0xA2, 0x0F,       // 8004: LDX #$0F  - Load 0x0F into X register
			0x8E, 0x00, 0x02, // 8006: STX $0200 - Store X at memory location $0200
			0xA2, 0x04,       // 8009: LDX #$04  - Load 0x04 into X register
			0x20, 0x18, 0x80, // 800B: JSR $8018 - Jump to subroutine at $8018
			0xAD, 0x10, 0x00, // 800E: LDA $0010 - Load the value at $0010 into A
			0x29, 0x0F,       // 8011: AND #$0F  - Perform bitwise AND on A with 0x0F
			0xD0, 0x05,       // 8013: BNE $801A - Branch to $801A if the Zero flag is not set
			0x00,             // 8015: BRK       - Break (forces an interrupt)
			0xEA,             // 8016: NOP       - No operation
			0xEA,             // 8017: NOP       - No operation
			0x60,             // 8018: RTS       - Return from subroutine

			// Subroutine: Increment X 3 times
			0xE8,             // 8019: INX       - Increment X by 1 (X becomes X+1)
			0xE8,             // 801A: INX       - Increment X by 1 (X becomes X+2)
			0xE8,             // 801B: INX       - Increment X by 1 (X becomes X+3)
			0x60              // 801C: RTS       - Return from subroutine
		};

		// Load ROM first.
		if (romReader.LoadRom("games/donkeykong.nes"))
		{
			romReader.PrintHeader();
		}
		
		cpu.LoadPrgProgram(romReader.GetPRGRom());
		cpu.Run();

		return 0;
	}
}