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
		using OpcodeHandler = void(CPU::*)();

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

		OpcodeHandler opcodeTable[256];

		std::thread cpuThread;

		// Memory Functions
		uint8_t Fetch();
		uint16_t FetchWord();

		uint8_t FetchIndirect(uint16_t& effectiveAddr, uint8_t reg);

		// Op Code Implementations

		//Branch OpCodes
		void BEQ();
		void BNE();
		void BMI();
		void BVS();
		void BVC();
		void BCC();
		void BCS();
		void BPL();

		//Single Opcodes
		void JSR();
		void INX();
		void INY();
		void BRK();
		void RTS();
		void DEY();
		void DEX();
		void NMI();

		//Dec
		void DEC_ZP();
		void DEC_Abs();

		//NOPs
		void NOP();
		void NOP_Implied();
		void NOP_Imm();
		void NOP_ZP();
		void NOP_ZPX();
		void NOP_Abs();
		void NOP_AbsX();

		void INC_ZP();

		void INC_Abs();

		//ORA
		void ORA_Imm();
		void ORA_Abs();
		void ORA_ZP();
		void ORA_IndirectX();

		void ORA_IndirectY();

		//RLA
		void RLA_IndirectX();
		void RLA_IndirectY();

		void ROR_A();
		void ROR_ZP();
		void ROR_Abs();
		void ROL_ZP();
		void ROL_Abs();
		void ROL_A();

		//ASL
		void ASL_A();
		void ASL_Abs();

		void ASL_ZP();

		//LSR
		void LSR_A();
		void LSR_ZP();

		void LSR_Abs();

		//Bit Test
		void BIT_Abs();
		void BIT_ZP();

		//Transfer Indexs
		void TXS();
		void TSX();
		void TXA();
		void TAX();
		void TAY();
		void TYA();

		void LAS();

		void RTI();

		//Flag OpCodes
		void CLD();
		void SED();
		void CLC();
		void SEC();
		void CLV();
		void SEI();

		//LDX
		void LDX_Imm();
		void LDX_ZP();
		void LDX_Abs();

		//LDA
		void LDA_Imm();
		void LDA_Abs();
		void LDA_ZP();
		void LDA_IndirectX();
		void LDA_IndirectY();

		//LDY
		void LDY_Imm();
		void LDY_ZP();
		void LDY_Abs();

		//STX
		void STX_Abs();
		void STX_ZP();

		//STA
		void STA_ZP();
		void STA_Abs();
		void STA_AbsY();
		void STA_AbsX();
		void STA_IndirectY();
		void STA_IndirectX();

		//STY
		void STY_ZP();
		void STY_Abs();

		//SBC
		void SBC_Imm();
		void SBC_Abs();
		void SBC_ZP();
		void SBC_IndirectX();

		//ADC
		void ADC_Imm();
		void ADC_Abs();
		void ADC_ZP();
		void ADC_IndirectX();
		void ADC_IndirectY();

		//EOR
		void EOR_Imm();
		void EOR_Abs();
		void EOR_ZP();
		void EOR_IndirectX();

		void SLO_ZPX();

		//JMP
		void JMP_Abs();
		void JMP_Indirect();

		//CMP
		void CMP_Imm();
		void CMP_Abs();
		void CMP_ZP();
		void CMP_IndirectX();
		void CPX_Imm();
		void CPX_ZP();
		void CPX_Abs();
		void CPY_Imm();
		void CPY_ZP();
		void CPY_Abs();

		void CompareOp(uint8_t operand, uint8_t reg);

		// Push Stack OpCodes
		void PHA();
		void PHP();
		void PLA();
		void PLP();

		//AND
		void AND_Imm();
		void AND_ZP();
		void AND_Abs();
		void AND_IndirectX();

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

