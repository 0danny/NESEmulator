#pragma once

#include <stdint.h>
#include <array>
#include <conio.h>

#include "utils/logger.h"
#include "utils/excepthandler.h"
#include "emulation/graphics/ppu.h"
#include "emulation/memorybus.h"
#include "monitor/window.h"

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
		CPU();

		void Reset();
		void Clock();
		void RequestNMI();

		uint8_t A = 0;   // Accumulator
		uint8_t X = 0;   // X Register
		uint8_t Y = 0;   // Y Register
		uint8_t SP = 0;  // Stack Pointer (Located between 0x0100 and 0x01FF) 1 byte.
		uint16_t PC = 0; // Program Counter
		uint8_t P = 0;   // Status Register
		int cycles = 0;

		static CPU& Instance()
		{
			static CPU INSTANCE;
			return INSTANCE;
		}

	private:
		MemoryBus& memoryBus;
		Utils::ExceptHandler& exceptHandler;

		uint16_t StackStart = 0x0100;

		enum class AddressingMode 
		{
			Implied,
			Accumulator,
			Immediate,
			ZeroPage,
			ZeroPageX,
			ZeroPageY,
			Absolute,
			AbsoluteX,
			AbsoluteY,
			Indirect,
			IndirectX,
			IndirectY
		};

		// Opcode table
		std::array<std::function<void()>, 256> opcodeTable;

		// Memory Functions
		uint8_t Fetch();
		uint16_t FetchWord();

		// Push Stack OpCodes
		void PHA();
		void PHP();
		void PLA();
		void PLP();

		//Interrupt/Return Addy OpCodes
		void RTS();
		void RTI();
		void BRK();
		void NMI();
		void JSR();

		//Instruction Operations
		void LD(AddressingMode mode, uint8_t& reg, int cycles);
		void ST(AddressingMode mode, uint8_t& reg, int cycles);
		void CMP(AddressingMode mode, uint8_t& reg, int cycles);
		void ADC_SBC(AddressingMode mode, int cycles, bool isSubtraction);
		void AND(AddressingMode mode, int cycles);
		void NOP(AddressingMode mode, int cycles);
		void INC(AddressingMode mode, int cycles);
		void ORA(AddressingMode mode, int cycles);
		void RLA(AddressingMode mode, int cycles);
		void JMP(AddressingMode mode);
		void Branch(bool condition);
		void BIT(AddressingMode mode, int cycles);
		void EOR(AddressingMode mode, int cycles);
		void FlagOperation(uint8_t flag, bool setFlag);
		void Transfer(uint8_t& dest, uint8_t& src, bool setFlags);
		void RementMemory(AddressingMode mode, bool increment, int cycles);
		void RementRegister(uint8_t& reg, bool increment);
		void Rotate(AddressingMode mode, bool rotateLeft, int cycles);
		void Shift(AddressingMode mode, bool shiftLeft, int cycles);

		//Illegal OpCodes (Unused)
		void SLO_ZPX();
		void LAS();

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
		void CountOpCodes();
		uint16_t GetOperandAddress(AddressingMode mode, bool* pageBoundaryCrossed);

		void PrintState();
	};
}

