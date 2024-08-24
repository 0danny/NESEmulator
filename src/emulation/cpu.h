#pragma once

#include <stdint.h>
#include <array>
#include <conio.h>

#include "utils/logger.h"

#define LOBYTE(w)           ((uint8_t)(w))
#define HIBYTE(w)           ((uint8_t)(((uint16_t)(w) >> 8) & 0xFF))
#define LOWORD(l)           ((uint16_t)(l))
#define HIWORD(l)           ((uint16_t)(((uint32_t)(l) >> 16) & 0xFFFF))

#define CARRY_FLAG     0x01  // Carry flag (bit 0)
#define ZERO_FLAG      0x02  // Zero flag (bit 1)
#define INTERRUPT_FLAG 0x04  // Interrupt disable flag (bit 2)
#define DECIMAL_FLAG   0x08  // Decimal mode flag (bit 3)
#define BREAK_FLAG     0x10  // Break command flag (bit 4)
#define UNUSED_FLAG    0x20  // Unused (bit 5, always set to 1)
#define OVERFLOW_FLAG  0x40  // Overflow flag (bit 6)
#define NEGATIVE_FLAG  0x80  // Negative flag (bit 7)

namespace Emulation
{
	class CPU
	{
	private:
		uint8_t A = 0;   // Accumulator
		uint8_t X = 0;   // X Register
		uint8_t Y = 0;   // Y Register
		uint8_t SP = 0; // Stack Pointer (Located between 0x0100 and 0x01FF) 1 byte.
		uint16_t PC = 0; // Program Counter
		uint8_t P = 0; // Status Register

		uint16_t StackStart = 0x0100;

		std::array<uint8_t, 65536> memory = { 0 };

		int cycles = 0;

		CPU();

		// Memory Functions
		uint8_t Fetch();
		uint16_t FetchWord();

		uint8_t Read(uint16_t addr);
		uint16_t ReadWord(uint16_t addr);

		void Write(uint16_t addr, uint8_t value);
		void WriteWord(uint16_t addr, uint16_t value);

		// Op Code Implementations

		//Single Opcodes
		void BEQ(uint8_t offset);
		void BNE(uint8_t offset);
		void JSR(uint16_t address);
		void INX();
		void BRK();
		void NOP();
		void RTS();

		//LDX
		void LDX_Imm(uint8_t value);

		//LDA
		void LDA_Imm(uint8_t value);
		void LDA_Abs(uint16_t address);

		//STX
		void STX_Abs(uint16_t address);
		void STA_ZP(uint8_t address);

		//ADC
		void ADC_Abs(uint16_t address);

		//JMP
		void JMP_Abs(uint16_t address);

		//AND
		void AND_Imm(uint8_t value);

		// Status Register (P) Functions
		void SetCarryFlag(bool value);
		void SetZeroFlag(bool value);
		void SetInterruptFlag(bool value);
		void SetDecimalFlag(bool value);
		void SetBreakFlag(bool value);
		void SetOverflowFlag(bool value);
		void SetNegativeFlag(bool value);

		void PushStack(uint8_t value);
		void PushStackWord(uint16_t value);

		uint8_t PullStack();
		uint16_t PullStackWord();

		void Reset();
		void Execute();

		void PrintState();

	public:

		void LoadProgram(const uint8_t* program, uint16_t programSize, uint16_t loadAddress);
		void Run();

		static CPU& Instance()
		{
			static CPU INSTANCE;
			return INSTANCE;
		}
	};
}

