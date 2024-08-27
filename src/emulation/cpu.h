#pragma once

#include <stdint.h>
#include <array>
#include <conio.h>

#include "utils/logger.h"
#include "utils/excepthandler.h"
#include "emulation/graphics/ppu.h"
#include "emulation/memorybus.h"

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

#define PAGE_NUMBER 0xFF00
#define PAGE_POSITION 0x00FF

namespace Emulation
{
	class CPU
	{
	public:
		using OpcodeHandler = void(CPU::*)();

		CPU();

		void Reset();
		void Clock();

		void RequestNMI();

		static CPU& Instance()
		{
			static CPU INSTANCE;
			return INSTANCE;
		}

	private:
		MemoryBus& memoryBus;
		Utils::ExceptHandler& exceptHandler;

		uint8_t A = 0;   // Accumulator
		uint8_t X = 0;   // X Register
		uint8_t Y = 0;   // Y Register
		uint8_t SP = 0;  // Stack Pointer (Located between 0x0100 and 0x01FF) 1 byte.
		uint16_t PC = 0; // Program Counter
		uint8_t P = 0;   // Status Register

		uint16_t StackStart = 0x0100;

		int cycles = 0;

		OpcodeHandler opcodeTable[256];

		std::thread cpuThread;

		// Memory Functions
		uint8_t Fetch();
		uint16_t FetchWord();

		// Op Code Implementations

		//Branch OpCodes
		void BEQ();
		void BNE();
		void BVS();
		void BCC();
		void BCS();
		void BPL();

		//Single Opcodes
		void JSR();
		void INX();
		void BRK();
		void RTS();
		void DEC();
		void DEY();
		void DEX();
		void NMI();

		//NOPs
		void NOP();
		void NOP_Implied();
		void NOP_Imm();
		void NOP_ZP();
		void NOP_ZPX();
		void NOP_Abs();
		void NOP_AbsX();

		//ASL
		void ASL_A();

		//Bit Test
		void BIT_Abs();

		//Transfer Indexs
		void TXS();
		void TSX();
		void TXA();
		void TAX();
		void TAY();
		void TYA();

		void LAS();

		//Flag OpCodes
		void CLD();
		void CLC();
		void SEI();

		//LDX
		void LDX_Imm();
		void LDX_Abs();

		//LDA
		void LDA_Imm();
		void LDA_Abs();
		void LDA_ZP();

		void LDY_Imm();
		void LDY_ZP();

		//STX
		void STX_Abs();

		//STA
		void STA_ZP();
		void STA_Abs();
		void STA_AbsY();
		void STA_AbsX();
		void STA_IndirectY();

		void STY_ZP();

		//ADC
		void ADC_Imm();
		void ADC_Abs();

		//JMP
		void JMP_Abs();

		void CMP_Imm();

		// Push Stack OpCodes
		void PHA();
		void PHP();
		void PLA();
		void PLP();

		//AND
		void AND_Imm();

		// Status Register (P) Functions
		void SetCarryFlag(bool value);
		void SetZeroFlag(bool value);
		void SetInterruptFlag(bool value);
		void SetDecimalFlag(bool value);
		void SetBreakFlag(bool value);
		void SetOverflowFlag(bool value);
		void SetNegativeFlag(bool value);
		void SetUnusedFlag(bool value);

		void PushStack(uint8_t value);
		void PushStackWord(uint16_t value);

		void ExceptionWrapper(std::string reason, std::string error);

		uint8_t PullStack();
		uint16_t PullStackWord();

		void InitializeOpcodes();

		void PrintState();
	};
}

