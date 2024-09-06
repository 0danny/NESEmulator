#include "cpu.h"

namespace Emulation
{
	CPU::CPU() :
		memoryBus(MemoryBus::Instance()),
		exceptHandler(Utils::ExceptHandler::Instance()),
		ppu(Graphics::PPU::Instance()),
		pcCallback(nullptr)
	{
		InitializeOpcodes();
		Reset();
	}

	void CPU::Reset()
	{
		A = X = Y = 0;

		SP = 0xFD;
		P = 0x24;
		PC = memoryBus.ReadWord(0xFFFC);  // Reset vector
		Utils::Logger::Info("Reset Vector - ", Utils::Logger::Uint16ToHex(PC));

		cycles = 7;
	}

	void CPU::Clock()
	{
		uint8_t opcode = Fetch();

		auto& executeInstruction = opcodeTable[opcode];

		if (executeInstruction)
		{
			//Utils::Logger::Info("[", Utils::Logger::Uint16ToHex(PC), "] OpCode (", Utils::Logger::Uint8ToHex(opcode), ")");
			executeInstruction();

			if (ppu.triggeredNMI)
			{
				NMI();

				ppu.triggeredNMI = false;
			}
		}
		else
		{
			ExceptionWrapper("OpCode Not Implemented", Utils::Logger::Uint8ToHex(opcode));
		}
	}

	void CPU::InitializeOpcodes()
	{
		// Clear the table
		opcodeTable.fill(nullptr);

		// Populate the table with opcode handlers

		//JMP
		opcodeTable[0x4C] = [this]() { JMP(AddressingMode::Absolute); };
		opcodeTable[0x6C] = [this]() { JMP(AddressingMode::Indirect); };

		//LDX
		opcodeTable[0xA2] = [this]() { LD(AddressingMode::Immediate, X); };
		opcodeTable[0xA6] = [this]() { LD(AddressingMode::ZeroPage, X); };
		opcodeTable[0xB6] = [this]() { LD(AddressingMode::ZeroPageY, X); };
		opcodeTable[0xAE] = [this]() { LD(AddressingMode::Absolute, X); };
		opcodeTable[0xBE] = [this]() { LD(AddressingMode::AbsoluteY, X); };

		//LDA
		opcodeTable[0xA9] = [this]() { LD(AddressingMode::Immediate, A); };
		opcodeTable[0xA5] = [this]() { LD(AddressingMode::ZeroPage, A); };
		opcodeTable[0xB5] = [this]() { LD(AddressingMode::ZeroPageX, A); };
		opcodeTable[0xAD] = [this]() { LD(AddressingMode::Absolute, A); };
		opcodeTable[0xBD] = [this]() { LD(AddressingMode::AbsoluteX, A); };
		opcodeTable[0xB9] = [this]() { LD(AddressingMode::AbsoluteY, A); };
		opcodeTable[0xA1] = [this]() { LD(AddressingMode::IndirectX, A); };
		opcodeTable[0xB1] = [this]() { LD(AddressingMode::IndirectY, A); };

		//LDY
		opcodeTable[0xA0] = [this]() { LD(AddressingMode::Immediate, Y); };
		opcodeTable[0xA4] = [this]() { LD(AddressingMode::ZeroPage, Y); };
		opcodeTable[0xB4] = [this]() { LD(AddressingMode::ZeroPageX, Y); };
		opcodeTable[0xAC] = [this]() { LD(AddressingMode::Absolute, Y); };
		opcodeTable[0xBC] = [this]() { LD(AddressingMode::AbsoluteX, Y); };

		//STX
		opcodeTable[0x8E] = [this]() { ST(AddressingMode::Absolute, X); };
		opcodeTable[0x86] = [this]() { ST(AddressingMode::ZeroPage, X); };
		opcodeTable[0x96] = [this]() { ST(AddressingMode::ZeroPageY, X); };

		//STA
		opcodeTable[0x85] = [this]() { ST(AddressingMode::ZeroPage, A); };
		opcodeTable[0x95] = [this]() { ST(AddressingMode::ZeroPageX, A); };
		opcodeTable[0x8D] = [this]() { ST(AddressingMode::Absolute, A); };
		opcodeTable[0x9D] = [this]() { ST(AddressingMode::AbsoluteX, A); };
		opcodeTable[0x99] = [this]() { ST(AddressingMode::AbsoluteY, A); };
		opcodeTable[0x81] = [this]() { ST(AddressingMode::IndirectX, A); };
		opcodeTable[0x91] = [this]() { ST(AddressingMode::IndirectY, A); };

		//STY
		opcodeTable[0x8C] = [this]() { ST(AddressingMode::Absolute, Y); };
		opcodeTable[0x84] = [this]() { ST(AddressingMode::ZeroPage, Y); };
		opcodeTable[0x94] = [this]() { ST(AddressingMode::ZeroPageX, Y); };

		//SBC
		opcodeTable[0xE9] = [this]() { ADC_SBC(AddressingMode::Immediate, true); };
		opcodeTable[0xE5] = [this]() { ADC_SBC(AddressingMode::ZeroPage, true); };
		opcodeTable[0xF5] = [this]() { ADC_SBC(AddressingMode::ZeroPageX, true); };
		opcodeTable[0xED] = [this]() { ADC_SBC(AddressingMode::Absolute, true); };
		opcodeTable[0xFD] = [this]() { ADC_SBC(AddressingMode::AbsoluteX, true); };
		opcodeTable[0xF9] = [this]() { ADC_SBC(AddressingMode::AbsoluteY, true); };
		opcodeTable[0xE1] = [this]() { ADC_SBC(AddressingMode::IndirectX, true); };
		opcodeTable[0xF1] = [this]() { ADC_SBC(AddressingMode::IndirectY, true); };

		//ADC
		opcodeTable[0x69] = [this]() { ADC_SBC(AddressingMode::Immediate, false); };
		opcodeTable[0x65] = [this]() { ADC_SBC(AddressingMode::ZeroPage, false); };
		opcodeTable[0x75] = [this]() { ADC_SBC(AddressingMode::ZeroPageX, false); };
		opcodeTable[0x6D] = [this]() { ADC_SBC(AddressingMode::Absolute, false); };
		opcodeTable[0x7D] = [this]() { ADC_SBC(AddressingMode::AbsoluteX, false); };
		opcodeTable[0x79] = [this]() { ADC_SBC(AddressingMode::AbsoluteY, false); };
		opcodeTable[0x61] = [this]() { ADC_SBC(AddressingMode::IndirectX, false); };
		opcodeTable[0x71] = [this]() { ADC_SBC(AddressingMode::IndirectY, false); };

		//INC
		opcodeTable[0xE6] = [this]() { INC(AddressingMode::ZeroPage); };
		opcodeTable[0xF6] = [this]() { INC(AddressingMode::ZeroPageX); };
		opcodeTable[0xEE] = [this]() { INC(AddressingMode::Absolute); };
		opcodeTable[0xFE] = [this]() { INC(AddressingMode::AbsoluteX); };

		//AND
		opcodeTable[0x29] = [this]() { AND(AddressingMode::Immediate); };
		opcodeTable[0x25] = [this]() { AND(AddressingMode::ZeroPage); };
		opcodeTable[0x35] = [this]() { AND(AddressingMode::ZeroPageX); };
		opcodeTable[0x2D] = [this]() { AND(AddressingMode::Absolute); };
		opcodeTable[0x3D] = [this]() { AND(AddressingMode::AbsoluteX); };
		opcodeTable[0x39] = [this]() { AND(AddressingMode::AbsoluteY); };
		opcodeTable[0x21] = [this]() { AND(AddressingMode::IndirectX); };
		opcodeTable[0x31] = [this]() { AND(AddressingMode::IndirectY); };

		//Return Address/Interrupt OpCodes
		opcodeTable[0x00] = [this]() { BRK(); };
		opcodeTable[0x20] = [this]() { JSR(); };
		opcodeTable[0x60] = [this]() { RTS(); };
		opcodeTable[0x40] = [this]() { RTI(); };

		//Compare OpCodes
		opcodeTable[0xC9] = [this]() { CMP(AddressingMode::Immediate, A); };
		opcodeTable[0xC5] = [this]() { CMP(AddressingMode::ZeroPage, A); };
		opcodeTable[0xD5] = [this]() { CMP(AddressingMode::ZeroPageX, A); };
		opcodeTable[0xCD] = [this]() { CMP(AddressingMode::Absolute, A); };
		opcodeTable[0xDD] = [this]() { CMP(AddressingMode::AbsoluteX, A); };
		opcodeTable[0xD9] = [this]() { CMP(AddressingMode::AbsoluteY, A); };
		opcodeTable[0xC1] = [this]() { CMP(AddressingMode::IndirectX, A); };
		opcodeTable[0xD1] = [this]() { CMP(AddressingMode::IndirectY, A); };

		opcodeTable[0xE0] = [this]() { CMP(AddressingMode::Immediate, X); };
		opcodeTable[0xE4] = [this]() { CMP(AddressingMode::ZeroPage, X); };
		opcodeTable[0xEC] = [this]() { CMP(AddressingMode::Absolute, X); };

		opcodeTable[0xC0] = [this]() { CMP(AddressingMode::Immediate, Y); };
		opcodeTable[0xC4] = [this]() { CMP(AddressingMode::ZeroPage, Y); };
		opcodeTable[0xCC] = [this]() { CMP(AddressingMode::Absolute, Y); };

		//Branch Opcodes
		opcodeTable[0xF0] = [this]() { Branch((P & ZERO_FLAG) != 0); };
		opcodeTable[0xD0] = [this]() { Branch((P & ZERO_FLAG) == 0); };

		opcodeTable[0x30] = [this]() { Branch((P & NEGATIVE_FLAG) != 0); };
		opcodeTable[0x10] = [this]() { Branch((P & NEGATIVE_FLAG) == 0); };

		opcodeTable[0x70] = [this]() { Branch((P & OVERFLOW_FLAG) != 0); };
		opcodeTable[0x50] = [this]() { Branch((P & OVERFLOW_FLAG) == 0); };

		opcodeTable[0xB0] = [this]() { Branch((P & CARRY_FLAG) != 0); };
		opcodeTable[0x90] = [this]() { Branch((P & CARRY_FLAG) == 0); };

		//Flag OpCodes
		opcodeTable[0xD8] = [this]() { FlagOperation(DECIMAL_FLAG, 0); };
		opcodeTable[0x18] = [this]() { FlagOperation(CARRY_FLAG, 0); };
		opcodeTable[0xB8] = [this]() { FlagOperation(OVERFLOW_FLAG, 0); };

		opcodeTable[0x78] = [this]() { FlagOperation(INTERRUPT_FLAG, 1); };
		opcodeTable[0x38] = [this]() { FlagOperation(CARRY_FLAG, 1); };
		opcodeTable[0xF8] = [this]() { FlagOperation(DECIMAL_FLAG, 1); };

		//Bit Test
		opcodeTable[0x2C] = [this]() { BIT(AddressingMode::Absolute); };
		opcodeTable[0x24] = [this]() { BIT(AddressingMode::ZeroPage); };

		//Push - Pull Stack OpCodes
		opcodeTable[0x08] = [this]() { PHP(); };
		opcodeTable[0x48] = [this]() { PHA(); };
		opcodeTable[0x68] = [this]() { PLA(); };
		opcodeTable[0x28] = [this]() { PLP(); };

		//ORA
		opcodeTable[0x09] = [this]() { ORA(AddressingMode::Immediate); };
		opcodeTable[0x05] = [this]() { ORA(AddressingMode::ZeroPage); };
		opcodeTable[0x15] = [this]() { ORA(AddressingMode::ZeroPageX); };
		opcodeTable[0x0D] = [this]() { ORA(AddressingMode::Absolute); };
		opcodeTable[0x1D] = [this]() { ORA(AddressingMode::AbsoluteX); };
		opcodeTable[0x19] = [this]() { ORA(AddressingMode::AbsoluteY); };
		opcodeTable[0x01] = [this]() { ORA(AddressingMode::IndirectX); };
		opcodeTable[0x11] = [this]() { ORA(AddressingMode::IndirectY); };

		//EOR
		opcodeTable[0x49] = [this]() { EOR(AddressingMode::Immediate); };
		opcodeTable[0x45] = [this]() { EOR(AddressingMode::ZeroPage); };
		opcodeTable[0x55] = [this]() { EOR(AddressingMode::ZeroPageX); };
		opcodeTable[0x4D] = [this]() { EOR(AddressingMode::Absolute); };
		opcodeTable[0x5D] = [this]() { EOR(AddressingMode::AbsoluteX); };
		opcodeTable[0x59] = [this]() { EOR(AddressingMode::AbsoluteY); };
		opcodeTable[0x41] = [this]() { EOR(AddressingMode::IndirectX); };
		opcodeTable[0x51] = [this]() { EOR(AddressingMode::IndirectY); };

		//Transfer Indexs
		opcodeTable[0xAA] = [this]() { Transfer(X, A, true); };
		opcodeTable[0xA8] = [this]() { Transfer(Y, A, true); };
		opcodeTable[0xBA] = [this]() { Transfer(X, SP, true); };
		opcodeTable[0x8A] = [this]() { Transfer(A, X, true); };
		opcodeTable[0x9A] = [this]() { Transfer(SP, X, false); };
		opcodeTable[0x98] = [this]() { Transfer(A, Y, true); };

		//Rement OpCodes
		opcodeTable[0xE8] = [this]() { RementRegister(X, true); };
		opcodeTable[0xC8] = [this]() { RementRegister(Y, true); };

		opcodeTable[0xCA] = [this]() { RementRegister(X, false); };
		opcodeTable[0x88] = [this]() { RementRegister(Y, false); };

		//Shift OpCodes (ASL, LSR)
		opcodeTable[0x0A] = [this]() { Shift(AddressingMode::Accumulator, true); };
		opcodeTable[0x06] = [this]() { Shift(AddressingMode::ZeroPage, true); };
		opcodeTable[0x16] = [this]() { Shift(AddressingMode::ZeroPageX, true); };
		opcodeTable[0x0E] = [this]() { Shift(AddressingMode::Absolute, true); };
		opcodeTable[0x1E] = [this]() { Shift(AddressingMode::AbsoluteX, true); };

		opcodeTable[0x4A] = [this]() { Shift(AddressingMode::Accumulator, false); };
		opcodeTable[0x46] = [this]() { Shift(AddressingMode::ZeroPage, false); };
		opcodeTable[0x56] = [this]() { Shift(AddressingMode::ZeroPageX, false); };
		opcodeTable[0x4E] = [this]() { Shift(AddressingMode::Absolute, false); };
		opcodeTable[0x5E] = [this]() { Shift(AddressingMode::AbsoluteX, false); };

		//Rotate OpCodes (ROR, ROL)
		opcodeTable[0x6A] = [this]() { Rotate(AddressingMode::Accumulator, false); };
		opcodeTable[0x66] = [this]() { Rotate(AddressingMode::ZeroPage, false); };
		opcodeTable[0x76] = [this]() { Rotate(AddressingMode::ZeroPageX, false); };
		opcodeTable[0x6E] = [this]() { Rotate(AddressingMode::Absolute, false); };
		opcodeTable[0x7E] = [this]() { Rotate(AddressingMode::AbsoluteX, false); };

		opcodeTable[0x2A] = [this]() { Rotate(AddressingMode::Accumulator, true); };
		opcodeTable[0x26] = [this]() { Rotate(AddressingMode::ZeroPage, true); };
		opcodeTable[0x36] = [this]() { Rotate(AddressingMode::ZeroPageX, true); };
		opcodeTable[0x2E] = [this]() { Rotate(AddressingMode::Absolute, true); };
		opcodeTable[0x3E] = [this]() { Rotate(AddressingMode::AbsoluteX, true); };

		//opcodeTable[0x27] = [this]() { RLA(AddressingMode::ZeroPage, 5); };
		//opcodeTable[0x37] = [this]() { RLA(AddressingMode::ZeroPageX, 6); };
		//opcodeTable[0x2F] = [this]() { RLA(AddressingMode::Absolute, 6); };
		//opcodeTable[0x3F] = [this]() { RLA(AddressingMode::AbsoluteX, 7); };
		//opcodeTable[0x3B] = [this]() { RLA(AddressingMode::AbsoluteY, 7); };
		//opcodeTable[0x23] = [this]() { RLA(AddressingMode::IndirectX, 8); };
		//opcodeTable[0x33] = [this]() { RLA(AddressingMode::IndirectY, 8); };

		//Decrement Memory Opcodes (DEC)
		opcodeTable[0xC6] = [this]() { RementMemory(AddressingMode::ZeroPage, false); };
		opcodeTable[0xD6] = [this]() { RementMemory(AddressingMode::ZeroPageX, false); };
		opcodeTable[0xCE] = [this]() { RementMemory(AddressingMode::Absolute, false); };
		opcodeTable[0xDE] = [this]() { RementMemory(AddressingMode::AbsoluteX, false); };

		//NOP Instructions
		opcodeTable[0xEA] = [this]() { NOP(AddressingMode::Implied); };
		opcodeTable[0x1A] = [this]() { NOP(AddressingMode::Implied); };
		opcodeTable[0x3A] = [this]() { NOP(AddressingMode::Implied); };
		opcodeTable[0x5A] = [this]() { NOP(AddressingMode::Implied); };
		opcodeTable[0x7A] = [this]() { NOP(AddressingMode::Implied); };
		opcodeTable[0xDA] = [this]() { NOP(AddressingMode::Implied); };
		opcodeTable[0xFA] = [this]() { NOP(AddressingMode::Implied); };

		opcodeTable[0x80] = [this]() { NOP(AddressingMode::Immediate); };
		opcodeTable[0x82] = [this]() { NOP(AddressingMode::Immediate); };
		opcodeTable[0x89] = [this]() { NOP(AddressingMode::Immediate); };
		opcodeTable[0xC2] = [this]() { NOP(AddressingMode::Immediate); };
		opcodeTable[0xE2] = [this]() { NOP(AddressingMode::Immediate); };

		opcodeTable[0x04] = [this]() { NOP(AddressingMode::ZeroPage); };
		opcodeTable[0x44] = [this]() { NOP(AddressingMode::ZeroPage); };
		opcodeTable[0x64] = [this]() { NOP(AddressingMode::ZeroPage); };
		opcodeTable[0x14] = [this]() { NOP(AddressingMode::ZeroPageX); };
		opcodeTable[0x34] = [this]() { NOP(AddressingMode::ZeroPageX); };
		opcodeTable[0x54] = [this]() { NOP(AddressingMode::ZeroPageX); };
		opcodeTable[0x74] = [this]() { NOP(AddressingMode::ZeroPageX); };
		opcodeTable[0xD4] = [this]() { NOP(AddressingMode::ZeroPageX); };
		opcodeTable[0xF4] = [this]() { NOP(AddressingMode::ZeroPageX); };

		opcodeTable[0x0C] = [this]() { NOP(AddressingMode::Absolute); };
		opcodeTable[0x1C] = [this]() { NOP(AddressingMode::AbsoluteX); };
		opcodeTable[0x3C] = [this]() { NOP(AddressingMode::AbsoluteX); };
		opcodeTable[0x5C] = [this]() { NOP(AddressingMode::AbsoluteX); };
		opcodeTable[0x7C] = [this]() { NOP(AddressingMode::AbsoluteX); };
		opcodeTable[0xDC] = [this]() { NOP(AddressingMode::AbsoluteX); };
		opcodeTable[0xFC] = [this]() { NOP(AddressingMode::AbsoluteX); };

		//Illegal Opcodes
		//opcodeTable[0xBB] = &CPU::LAS;
		//opcodeTable[0x17] = &CPU::SLO_ZPX;

		CountOpCodes();
	}

	uint8_t CPU::Read(uint16_t address)
	{
		uint8_t result = memoryBus.Read(address);
		AddCycle();
		return result;
	}

	uint16_t CPU::ReadWord(uint16_t address)
	{
		uint16_t result = memoryBus.ReadWord(address);

		AddCycle();
		AddCycle();


		return result;
	}

	void CPU::Write(uint16_t address, uint8_t value)
	{
		memoryBus.Write(address, value);
		AddCycle();
	}

	void CPU::CountOpCodes()
	{
		//Count OpCodes
		int numOpcodes = 0;

		for (int i = 0; i < 0xFF; i++)
		{
			if (opcodeTable[i] != nullptr)
			{
				numOpcodes++;
			}
		}

		float opCodePercent = floor((static_cast<float>(numOpcodes) / 0xFF) * 100);

		Utils::Logger::Info("Implemented opcodes ", numOpcodes, " out of ", 0xFF, " (", opCodePercent, "%)");
	}

	void CPU::AddCycle()
	{
		ppu.Clock();
		ppu.Clock();
		ppu.Clock();

		this->cycles++;
	}

	uint16_t CPU::GetOperandAddress(AddressingMode mode, bool addCycle = false)
	{
		switch (mode)
		{
		case AddressingMode::Implied:
		case AddressingMode::Accumulator:
			return 0;

		case AddressingMode::Immediate:
			return Fetch();

		case AddressingMode::ZeroPage:
			return Fetch();

		case AddressingMode::ZeroPageX:
			AddCycle();
			return (Fetch() + X) & 0xFF;

		case AddressingMode::ZeroPageY:
			AddCycle();
			return (Fetch() + Y) & 0xFF;

		case AddressingMode::Absolute:
			return FetchWord();

		case AddressingMode::AbsoluteX: {
			uint16_t base = FetchWord();
			uint16_t effective = base + X;

			if (((base & PAGE_NUMBER) != (effective & PAGE_NUMBER)) || addCycle)
				AddCycle();

			return effective;
		}

		case AddressingMode::AbsoluteY: {
			uint16_t base = FetchWord();
			uint16_t effective = base + Y;

			if (((base & PAGE_NUMBER) != (effective & PAGE_NUMBER)) || addCycle)
				AddCycle();

			return effective;
		}

		case AddressingMode::Indirect: {
			uint16_t base = FetchWord();
			uint8_t lo = Read(base);
			uint8_t hi = Read((base & 0xFF00) | ((base + 1) & 0x00FF));
			return (hi << 8) | lo;
		}

		case AddressingMode::IndirectX: {
			uint8_t zp = (Fetch() + X) & 0xFF;
			AddCycle();
			uint16_t effective = Read(zp);
			effective |= (Read((zp + 1) & 0xFF) << 8);
			return effective;
		}

		case AddressingMode::IndirectY: {
			uint8_t zp = Fetch();
			uint16_t base = Read(zp);
			base |= (Read((zp + 1) & 0xFF) << 8);

			uint16_t effective = base + Y;

			if (((base & PAGE_NUMBER) != (effective & PAGE_NUMBER)) || addCycle)
				AddCycle();

			return effective;
		}


		default:
			ExceptionWrapper("Unsupported addressing mode.", "");
			return 0;
		}
	}

	void CPU::ExceptionWrapper(std::string reason, std::string error)
	{
		exceptHandler.ThrowException(reason, error);
		PrintState();
	}

#pragma region Single OpCodes

	void CPU::NMI()
	{
		// 7 Cycles
		PushStackWord(PC);

		SetBreakFlag(false);
		PushStack(P);

		SetInterruptFlag(true);

		PC = ReadWord(0xFFFA);

		AddCycle();
	}

	void CPU::BRK()
	{
		PC += 2;

		PushStackWord(PC);

		SetBreakFlag(true);

		PushStack(P);

		SetInterruptFlag(true);

		PC = ReadWord(0xFFFE);

		//Utils::Logger::Warning("CPU ENTERED INTERRUPT HANDLER at ", Utils::Logger::Uint16ToHex(PC));

		AddCycle();
	}

	void CPU::JSR()
	{
		//Push the return address to the stack, +1 to get the return address.
		PushStackWord(PC + 1);

		//Utils::Logger::Info("Pushing PC onto stack - ", Utils::Logger::Uint16ToHex(PC + 1));

		//Preform normal JMP
		JMP(AddressingMode::Absolute);

		AddCycle();
	}

	void CPU::RTS()
	{
		//Get the return addy from stack
		uint16_t ret = PullStackWord();

		PC = ret + 1;

		AddCycle();
		AddCycle();
		AddCycle();
	}

	void CPU::RTI()
	{
		// Pull the status register from the stack
		P = PullStack();
		SetBreakFlag(false);  // Clear the Break flag
		SetUnusedFlag(true);  // Ensure the unused flag is always set to 1

		// Pull the program counter (PC) from the stack
		uint16_t pcLow = PullStack();
		uint16_t pcHigh = PullStack();
		PC = (pcHigh << 8) | pcLow;

		AddCycle();
		AddCycle();
	}


#pragma endregion

#pragma region Illegal OpCodes

	void CPU::SLO_ZPX()
	{
		// Fetch the zero page address, then add X to it (with wrapping around)
		uint8_t zeroPageAddr = Fetch();
		uint8_t addr = zeroPageAddr + X;

		// Read the value from the zero page address
		uint8_t value = Read(addr);

		// Perform an ASL (Arithmetic Shift Left) operation on the value
		SetCarryFlag(value & 0x80);  // Set the carry flag if the high bit is set
		value <<= 1;

		// Write the shifted value back to the zero page address
		Write(addr, value);

		// Perform an OR operation between the shifted value and the accumulator
		A |= value;

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 6;  // SLO (Zero Page,X) takes 6 cycles
	}

	void CPU::LAS()
	{
		uint16_t baseAddr = FetchWord();
		uint16_t addr = baseAddr + Y;

		uint8_t result = Read(addr) & SP;

		A = result;
		X = result;
		SP = result;

		// Set the Zero and Negative flags based on the result
		SetZeroFlag(result == 0);
		SetNegativeFlag(result & NEGATIVE_FLAG);

		cycles += 4;

		if ((baseAddr & PAGE_NUMBER) != (addr & PAGE_NUMBER))
		{
			cycles++;
		}
	}

#pragma endregion

#pragma region Push/Pull Stack OpCodes

	void CPU::PHA()
	{
		PushStack(A);

		AddCycle();
	}

	void CPU::PHP()
	{
		uint8_t status = P;

		SetBreakFlag(true);
		SetUnusedFlag(true); // Should always be 1 anyway.

		PushStack(P);

		P = status;

		AddCycle();
	}

	void CPU::PLA()
	{
		A = PullStack();

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		AddCycle();
		AddCycle();
	}

	void CPU::PLP()
	{
		P = PullStack();

		SetBreakFlag(false);
		SetUnusedFlag(true);

		AddCycle();
		AddCycle();
	}

#pragma endregion

	void CPU::RementMemory(AddressingMode mode, bool increment)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t value = Read(address);

		value = increment ? (value + 1) : (value - 1);

		Write(address, value);

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		AddCycle();

		if (mode == AddressingMode::AbsoluteX)
			AddCycle();
	}

	void CPU::RementRegister(uint8_t& reg, bool increment)
	{
		reg = increment ? (reg + 1) : (reg - 1);

		SetZeroFlag(reg == 0);
		SetNegativeFlag(reg & NEGATIVE_FLAG);

		AddCycle();
	}

	void CPU::Branch(bool condition)
	{
		int8_t offset = Fetch();

		if (condition)
		{
			uint16_t oldPC = PC;

			PC += offset;
			AddCycle();

			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
				AddCycle();
		}
	}

	void CPU::NOP(AddressingMode mode)
	{
		GetOperandAddress(mode);

		if (mode != AddressingMode::Immediate)
			AddCycle();
	}

	void CPU::INC(AddressingMode mode)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t value = Read(address);

		value += 1;

		Write(address, value);

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		AddCycle();

		if (mode == AddressingMode::AbsoluteX)
			AddCycle();
	}

	void CPU::ORA(AddressingMode mode)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t operand = (mode == AddressingMode::Immediate) ? address : Read(address);

		A |= operand;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);
	}

	void CPU::RLA(AddressingMode mode, int cycles)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t value = Read(address);

		uint8_t carryIn = (P & CARRY_FLAG) ? 0x01 : 0x00;
		SetCarryFlag(value & 0x80);
		value = (value << 1) | carryIn;

		Write(address, value);

		A &= value;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		this->cycles += cycles;
	}

	void CPU::Rotate(AddressingMode mode, bool rotateLeft)
	{
		uint8_t carryIn = (P & CARRY_FLAG) ? (rotateLeft ? 0x01 : 0x80) : 0x00;
		uint8_t value;

		if (mode == AddressingMode::Accumulator)
		{
			value = A;
			SetCarryFlag(rotateLeft ? (value & 0x80) : (value & 0x01));

			if (rotateLeft)
				value = (value << 1) | carryIn;
			else
				value = (value >> 1) | carryIn;

			A = value;
		}
		else
		{
			uint16_t address = GetOperandAddress(mode);
			value = Read(address);

			SetCarryFlag(rotateLeft ? (value & 0x80) : (value & 0x01));

			if (rotateLeft)
				value = (value << 1) | carryIn;
			else
				value = (value >> 1) | carryIn;

			Write(address, value);
		}

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		AddCycle();

		if (mode == AddressingMode::AbsoluteX)
			AddCycle();
	}

	void CPU::Shift(AddressingMode mode, bool shiftLeft)
	{
		if (mode == AddressingMode::Accumulator)
		{
			SetCarryFlag(A & (shiftLeft ? 0x80 : 0x01));

			if (shiftLeft)
				A <<= 1;
			else
				A >>= 1;

			SetZeroFlag(A == 0);
			SetNegativeFlag(shiftLeft ? (A & NEGATIVE_FLAG) : 0);
		}
		else
		{
			uint16_t address = GetOperandAddress(mode);
			uint8_t operand = Read(address);

			SetCarryFlag(operand & (shiftLeft ? 0x80 : 0x01));

			if (shiftLeft)
				operand <<= 1;
			else
				operand >>= 1;

			Write(address, operand);

			SetZeroFlag(operand == 0);
			SetNegativeFlag(shiftLeft ? (operand & NEGATIVE_FLAG) : 0);
		}

		AddCycle();

		if (mode == AddressingMode::AbsoluteX)
			AddCycle();
	}

	void CPU::BIT(AddressingMode mode)
	{
		uint8_t memValue = Read(GetOperandAddress(mode));

		uint8_t result = A & memValue;

		SetZeroFlag(result == 0);
		SetNegativeFlag(memValue & NEGATIVE_FLAG);
		SetOverflowFlag(memValue & OVERFLOW_FLAG);
	}

	void CPU::EOR(AddressingMode mode)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t operand = (mode == AddressingMode::Immediate) ? address : Read(address);

		A ^= operand;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);
	}

	void CPU::LD(AddressingMode mode, uint8_t& reg)
	{
		uint16_t address = GetOperandAddress(mode);

		if (mode == AddressingMode::Immediate)
			reg = address;
		else
			reg = Read(address);

		SetZeroFlag(reg == 0);
		SetNegativeFlag(reg & NEGATIVE_FLAG);
	}

	void CPU::ST(AddressingMode mode, uint8_t& reg)
	{
		uint16_t address = GetOperandAddress(mode, true);

		Write(address, reg);
	}

	void CPU::ADC_SBC(AddressingMode mode, bool isSubtraction)
	{
		uint16_t operandAddress = GetOperandAddress(mode);

		uint8_t operand = (mode == AddressingMode::Immediate) ? operandAddress : Read(operandAddress);

		if (isSubtraction)
		{
			operand = ~operand; // Invert the operand for subtraction
		}

		uint16_t result = A + operand + (P & CARRY_FLAG ? 1 : 0);

		// Overflow occurs if there is a sign change that shouldn't happen
		bool overflow = ((A ^ result) & (operand ^ result) & 0x80) != 0;

		SetCarryFlag(result > 0xFF);
		SetZeroFlag((result & 0xFF) == 0);
		SetOverflowFlag(overflow);
		SetNegativeFlag(result & NEGATIVE_FLAG);

		A = result & 0xFF;
	}

	void CPU::AND(AddressingMode mode)
	{
		uint16_t operandAddress = GetOperandAddress(mode);

		uint8_t operand = (mode == AddressingMode::Immediate) ? operandAddress : Read(operandAddress);

		A &= operand;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);
	}

	void CPU::JMP(AddressingMode mode)
	{
		uint16_t address = GetOperandAddress(mode);

		PC = address;
	}

	void CPU::CMP(AddressingMode mode, uint8_t& reg)
	{
		uint16_t operandAddress = GetOperandAddress(mode);

		uint8_t operand = (mode == AddressingMode::Immediate) ? operandAddress : Read(operandAddress);

		uint8_t result = reg - operand;

		SetCarryFlag(reg >= operand);
		SetZeroFlag(result == 0);
		SetNegativeFlag(result & NEGATIVE_FLAG);
	}

	void CPU::Transfer(uint8_t& dest, uint8_t& src, bool setFlags)
	{
		dest = src;

		if (setFlags)
		{
			SetNegativeFlag(dest & NEGATIVE_FLAG);
			SetZeroFlag(dest == 0);
		}

		AddCycle();
	}

	void CPU::FlagOperation(uint8_t flag, bool setFlag)
	{
		if (setFlag)
		{
			P |= flag;
		}
		else
		{
			P &= ~flag;
		}

		AddCycle();
	}

	void CPU::PushStack(uint8_t value)
	{
		Write(StackStart + SP, value);  // Store the value at the current stack pointer address
		SP--;  // Decrement the stack pointer
	}

	void CPU::PushStackWord(uint16_t value)
	{
		//We have to push the high byte first as we are growing downwards.

		// Push the high byte first
		PushStack(HIBYTE(value));
		// Then push the low byte
		PushStack(LOBYTE(value));
	}

	uint8_t CPU::PullStack()
	{
		//Read from Stack PTR
		SP++;
		uint8_t stValue = Read(StackStart + SP);
		return stValue;
	}

	uint16_t CPU::PullStackWord()
	{
		uint8_t lowByte = PullStack();  // Pull the low byte first
		uint8_t highByte = PullStack(); // Then pull the high byte
		return (highByte << 8) | lowByte;  // Combine them into a 16-bit word
	}

	// Fetch the next byte and increment the PC
	uint8_t CPU::Fetch()
	{
		uint8_t result = Read(PC++);
		return result;
	}

	// Fetch the next word (16-bit) and increment the PC by 2 bytes
	uint16_t CPU::FetchWord()
	{
		uint16_t word = ReadWord(PC);
		PC += 2;
		return word;
	}

#pragma region Status Register Functions

	void CPU::SetCarryFlag(bool value) {
		if (value) {
			P |= CARRY_FLAG;  // Set the bit
		}
		else {
			P &= ~CARRY_FLAG; // Clear the bit
		}
	}

	void CPU::SetZeroFlag(bool value) {
		if (value) {
			P |= ZERO_FLAG;
		}
		else {
			P &= ~ZERO_FLAG;
		}
	}

	void CPU::SetInterruptFlag(bool value) {
		if (value) {
			P |= INTERRUPT_FLAG;
		}
		else {
			P &= ~INTERRUPT_FLAG;
		}
	}

	void CPU::SetDecimalFlag(bool value) {
		if (value) {
			P |= DECIMAL_FLAG;
		}
		else {
			P &= ~DECIMAL_FLAG;
		}
	}

	void CPU::SetBreakFlag(bool value) {
		if (value) {
			P |= BREAK_FLAG;
		}
		else {
			P &= ~BREAK_FLAG;
		}
	}

	void CPU::SetOverflowFlag(bool value) {
		if (value) {
			P |= OVERFLOW_FLAG;
		}
		else {
			P &= ~OVERFLOW_FLAG;
		}
	}

	void CPU::SetNegativeFlag(bool value) {
		if (value) {
			P |= NEGATIVE_FLAG;
		}
		else {
			P &= ~NEGATIVE_FLAG;
		}
	}

	void CPU::SetUnusedFlag(bool value) {
		if (value) {
			P |= UNUSED_FLAG;
		}
		else {
			P &= ~UNUSED_FLAG;
		}
	}

#pragma endregion

	void CPU::PrintState()
	{
		Utils::Logger::Info("------------- CPU State -------------");
		Utils::Logger::Info("A: ", Utils::Logger::Uint8ToHex(A));
		Utils::Logger::Info("X: ", Utils::Logger::Uint8ToHex(X));
		Utils::Logger::Info("Y: ", Utils::Logger::Uint8ToHex(Y));
		Utils::Logger::Info("SP: ", Utils::Logger::Uint8ToHex(SP));
		Utils::Logger::Info("PC: ", Utils::Logger::Uint16ToHex(PC));

		Utils::Logger::Info("Status Flags (P): ", Utils::Logger::Uint8ToHex(P));
		Utils::Logger::Info("  N V - B D I Z C");
		Utils::Logger::Info("  ",
			(P & NEGATIVE_FLAG ? '1' : '0'), " ",
			(P & OVERFLOW_FLAG ? '1' : '0'), " ",
			((P & UNUSED_FLAG) ? '1' : '0'),  // Unused bit
			" ", (P & BREAK_FLAG ? '1' : '0'), " ",
			(P & DECIMAL_FLAG ? '1' : '0'), " ",
			(P & INTERRUPT_FLAG ? '1' : '0'), " ",
			(P & ZERO_FLAG ? '1' : '0'), " ",
			(P & CARRY_FLAG ? '1' : '0')
		);

		Utils::Logger::Info("Cycles: ", std::dec, cycles);

		// Display the top 4 bytes of the stack
		Utils::Logger::Info("Stack State: ");

		int count = 0;

		while (count < 4)
		{
			uint16_t currentAddress = (StackStart + SP) - count;

			uint8_t stackValue = Read(currentAddress);

			Utils::Logger::Info("  [", Utils::Logger::Uint16ToHex(currentAddress), "] = ", Utils::Logger::Uint8ToHex(stackValue));

			count++;
		}

		Utils::Logger::Info("-------------------------------------");
	}

	void CPU::RegisterPCCallback(PCCallback pcCallback)
	{
		this->pcCallback = std::move(pcCallback);
	}
}