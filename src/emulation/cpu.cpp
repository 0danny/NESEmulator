#include "cpu.h"

namespace Emulation
{
	CPU::CPU() : memoryBus(MemoryBus::Instance()), exceptHandler(Utils::ExceptHandler::Instance())
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

		auto& window = Monitor::Window::Instance();
		auto& opcodeStruct = window.opcodeLookup[opcode];

		auto& executeInstruction = opcodeTable[opcode];

		if (executeInstruction)
		{
			Utils::Logger::Info("[", Utils::Logger::Uint16ToHex(PC), "] OpCode (", Utils::Logger::Uint8ToHex(opcode), ") - ", opcodeStruct.name);

			executeInstruction();
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
		opcodeTable[0xA2] = [this]() { LD(AddressingMode::Immediate, X, 2); };
		opcodeTable[0xA6] = [this]() { LD(AddressingMode::ZeroPage, X, 3); };
		opcodeTable[0xAE] = [this]() { LD(AddressingMode::Absolute, X, 4); };

		//LDA
		opcodeTable[0xA9] = [this]() { LD(AddressingMode::Immediate, A, 2); };
		opcodeTable[0xA5] = [this]() { LD(AddressingMode::ZeroPage, A, 3); };
		opcodeTable[0xAD] = [this]() { LD(AddressingMode::Absolute, A, 4); };
		opcodeTable[0xA1] = [this]() { LD(AddressingMode::IndirectX, A, 6); };
		opcodeTable[0xB1] = [this]() { LD(AddressingMode::IndirectY, A, 5); };

		//LDY
		opcodeTable[0xA0] = [this]() { LD(AddressingMode::Immediate, Y, 2); };
		opcodeTable[0xAC] = [this]() { LD(AddressingMode::Absolute, Y, 4); };
		opcodeTable[0xA4] = [this]() { LD(AddressingMode::ZeroPage, Y, 3); };

		//STX
		opcodeTable[0x8E] = [this]() { ST(AddressingMode::Absolute, X, 4); };
		opcodeTable[0x86] = [this]() { ST(AddressingMode::ZeroPage, X, 3); };

		//STA
		opcodeTable[0x85] = [this]() { ST(AddressingMode::ZeroPage, A, 3); };
		opcodeTable[0x8D] = [this]() { ST(AddressingMode::Absolute, A, 4); };
		opcodeTable[0x9D] = [this]() { ST(AddressingMode::AbsoluteX, A, 5); };
		opcodeTable[0x99] = [this]() { ST(AddressingMode::AbsoluteY, A, 5); };
		opcodeTable[0x81] = [this]() { ST(AddressingMode::IndirectX, A, 6); };
		opcodeTable[0x91] = [this]() { ST(AddressingMode::IndirectY, A, 6); };

		//STY
		opcodeTable[0x8C] = [this]() { ST(AddressingMode::Absolute, Y, 4); };
		opcodeTable[0x84] = [this]() { ST(AddressingMode::ZeroPage, Y, 3); };

		//SBC
		opcodeTable[0xE9] = [this]() { ADC_SBC(AddressingMode::Immediate, 2, true); };
		opcodeTable[0xED] = [this]() { ADC_SBC(AddressingMode::Absolute, 4, true); };
		opcodeTable[0xE1] = [this]() { ADC_SBC(AddressingMode::IndirectX, 6, true); };
		opcodeTable[0xE5] = [this]() { ADC_SBC(AddressingMode::ZeroPage, 3, true); };

		//ADC
		opcodeTable[0x6D] = [this]() { ADC_SBC(AddressingMode::Absolute, 4, false); };
		opcodeTable[0x69] = [this]() { ADC_SBC(AddressingMode::Immediate, 2, false); };
		opcodeTable[0x65] = [this]() { ADC_SBC(AddressingMode::ZeroPage, 3, false); };
		opcodeTable[0x61] = [this]() { ADC_SBC(AddressingMode::IndirectX, 6, false); };

		//INC
		opcodeTable[0xE6] = [this]() { INC(AddressingMode::ZeroPage, 5); };
		opcodeTable[0xEE] = [this]() { INC(AddressingMode::Absolute, 6); };

		//AND
		opcodeTable[0x29] = [this]() { AND(AddressingMode::Immediate, 2); };
		opcodeTable[0x25] = [this]() { AND(AddressingMode::ZeroPage, 3); };
		opcodeTable[0x2D] = [this]() { AND(AddressingMode::Absolute, 4); };
		opcodeTable[0x21] = [this]() { AND(AddressingMode::IndirectX, 6); };
		opcodeTable[0x31] = [this]() { AND(AddressingMode::IndirectY, 5); };

		//Return Address/Interrupt OpCodes
		opcodeTable[0x00] = [this]() { BRK(); };
		opcodeTable[0x20] = [this]() { JSR(); };
		opcodeTable[0x60] = [this]() { RTS(); };
		opcodeTable[0x40] = [this]() { RTI(); };

		//Compare OpCodes
		opcodeTable[0xC9] = [this]() { CMP(AddressingMode::Immediate, A, 2); };
		opcodeTable[0xC5] = [this]() { CMP(AddressingMode::ZeroPage, A, 3); };
		opcodeTable[0xCD] = [this]() { CMP(AddressingMode::Absolute, A, 4); };
		opcodeTable[0xC1] = [this]() { CMP(AddressingMode::IndirectX, A, 6); };

		opcodeTable[0xE0] = [this]() { CMP(AddressingMode::Immediate, X, 2); };
		opcodeTable[0xE4] = [this]() { CMP(AddressingMode::ZeroPage, X, 3); };
		opcodeTable[0xEC] = [this]() { CMP(AddressingMode::Absolute, X, 4); };

		opcodeTable[0xC0] = [this]() { CMP(AddressingMode::Immediate, Y, 2); };
		opcodeTable[0xC4] = [this]() { CMP(AddressingMode::ZeroPage, Y, 3); };
		opcodeTable[0xCC] = [this]() { CMP(AddressingMode::Absolute, Y, 4); };

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
		opcodeTable[0x2C] = [this]() { BIT(AddressingMode::Absolute, 4); };
		opcodeTable[0x24] = [this]() { BIT(AddressingMode::ZeroPage, 3); };

		//Push - Pull Stack OpCodes
		opcodeTable[0x08] = [this]() { PHP(); };
		opcodeTable[0x48] = [this]() { PHA(); };
		opcodeTable[0x68] = [this]() { PLA(); };
		opcodeTable[0x28] = [this]() { PLP(); };

		//ORA
		opcodeTable[0x0D] = [this]() { ORA(AddressingMode::Absolute, 4); };
		opcodeTable[0x09] = [this]() { ORA(AddressingMode::Immediate, 2); };
		opcodeTable[0x05] = [this]() { ORA(AddressingMode::ZeroPage, 3); };
		opcodeTable[0x01] = [this]() { ORA(AddressingMode::IndirectX, 6); };
		opcodeTable[0x11] = [this]() { ORA(AddressingMode::IndirectY, 5); };

		//EOR
		opcodeTable[0x4D] = [this]() { EOR(AddressingMode::Absolute, 4); };
		opcodeTable[0x49] = [this]() { EOR(AddressingMode::Immediate, 2); };
		opcodeTable[0x41] = [this]() { EOR(AddressingMode::IndirectX, 6); };
		opcodeTable[0x45] = [this]() { EOR(AddressingMode::ZeroPage, 3); };

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

		//Shift OpCodes
		opcodeTable[0x0A] = [this]() { Shift(AddressingMode::Accumulator, true, 2); };
		opcodeTable[0x0E] = [this]() { Shift(AddressingMode::Absolute, true, 6); };
		opcodeTable[0x06] = [this]() { Shift(AddressingMode::ZeroPage, true, 5); };

		opcodeTable[0x4A] = [this]() { Shift(AddressingMode::Accumulator, false, 2); };
		opcodeTable[0x46] = [this]() { Shift(AddressingMode::ZeroPage, false, 5); };
		opcodeTable[0x4E] = [this]() { Shift(AddressingMode::Absolute, false, 6); };

		//Rotate OpCodes
		opcodeTable[0x6A] = [this]() { Rotate(AddressingMode::Accumulator, false, 2); };
		opcodeTable[0x66] = [this]() { Rotate(AddressingMode::ZeroPage, false, 5); };
		opcodeTable[0x6E] = [this]() { Rotate(AddressingMode::Absolute, false, 6); };

		opcodeTable[0x2A] = [this]() { Rotate(AddressingMode::Accumulator, true, 2); };
		opcodeTable[0x26] = [this]() { Rotate(AddressingMode::ZeroPage, true, 5); };
		opcodeTable[0x2E] = [this]() { Rotate(AddressingMode::Absolute, true, 6); };

		opcodeTable[0x23] = [this]() { RLA(AddressingMode::IndirectX, 8); };
		opcodeTable[0x33] = [this]() { RLA(AddressingMode::IndirectY, 8); };

		//Decrement Memory Opcodes
		opcodeTable[0xC6] = [this]() { RementMemory(AddressingMode::ZeroPage, false, 5); };
		opcodeTable[0xCE] = [this]() { RementMemory(AddressingMode::Absolute, false, 6); };

		//NOP Instructions
		opcodeTable[0xEA] = [this]() { NOP(AddressingMode::Implied, 2); };
		opcodeTable[0x1A] = [this]() { NOP(AddressingMode::Implied, 2); };
		opcodeTable[0x3A] = [this]() { NOP(AddressingMode::Implied, 2); };
		opcodeTable[0x5A] = [this]() { NOP(AddressingMode::Implied, 2); };
		opcodeTable[0x7A] = [this]() { NOP(AddressingMode::Implied, 2); };
		opcodeTable[0xDA] = [this]() { NOP(AddressingMode::Implied, 2); };
		opcodeTable[0xFA] = [this]() { NOP(AddressingMode::Implied, 2); };

		opcodeTable[0x80] = [this]() { NOP(AddressingMode::Immediate, 2); };
		opcodeTable[0x82] = [this]() { NOP(AddressingMode::Immediate, 2); };
		opcodeTable[0x89] = [this]() { NOP(AddressingMode::Immediate, 2); };
		opcodeTable[0xC2] = [this]() { NOP(AddressingMode::Immediate, 2); };
		opcodeTable[0xE2] = [this]() { NOP(AddressingMode::Immediate, 2); };

		opcodeTable[0x04] = [this]() { NOP(AddressingMode::ZeroPage, 3); };
		opcodeTable[0x44] = [this]() { NOP(AddressingMode::ZeroPage, 3); };
		opcodeTable[0x64] = [this]() { NOP(AddressingMode::ZeroPage, 3); };
		opcodeTable[0x14] = [this]() { NOP(AddressingMode::ZeroPageX, 4); };
		opcodeTable[0x34] = [this]() { NOP(AddressingMode::ZeroPageX, 4); };
		opcodeTable[0x54] = [this]() { NOP(AddressingMode::ZeroPageX, 4); };
		opcodeTable[0x74] = [this]() { NOP(AddressingMode::ZeroPageX, 4); };
		opcodeTable[0xD4] = [this]() { NOP(AddressingMode::ZeroPageX, 4); };
		opcodeTable[0xF4] = [this]() { NOP(AddressingMode::ZeroPageX, 4); };

		opcodeTable[0x0C] = [this]() { NOP(AddressingMode::Absolute, 4); };
		opcodeTable[0x1C] = [this]() { NOP(AddressingMode::AbsoluteX, 4); };
		opcodeTable[0x3C] = [this]() { NOP(AddressingMode::AbsoluteX, 4); };
		opcodeTable[0x5C] = [this]() { NOP(AddressingMode::AbsoluteX, 4); };
		opcodeTable[0x7C] = [this]() { NOP(AddressingMode::AbsoluteX, 4); };
		opcodeTable[0xDC] = [this]() { NOP(AddressingMode::AbsoluteX, 4); };
		opcodeTable[0xFC] = [this]() { NOP(AddressingMode::AbsoluteX, 4); };

		//Illegal Opcodes
		//opcodeTable[0xBB] = &CPU::LAS;
		//opcodeTable[0x17] = &CPU::SLO_ZPX;

		CountOpCodes();
	}

	void CPU::CountOpCodes()
	{
		//Count OpCodes
		int numOpcodes = 0;

		for (int i = 0; i < 256; i++)
		{
			if (opcodeTable[i] != nullptr)
			{
				numOpcodes++;
			}
		}

		float opCodePercent = floor((static_cast<float>(numOpcodes) / 151) * 100);

		Utils::Logger::Info("Implemented opcodes ", numOpcodes, " out of 151 (", opCodePercent, "%)");
	}

	uint16_t CPU::GetOperandAddress(AddressingMode mode, bool* pageBoundaryCrossed = nullptr)
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
			return (Fetch() + X) & 0xFF;

		case AddressingMode::ZeroPageY:
			return (Fetch() + Y) & 0xFF;

		case AddressingMode::Absolute:
			return FetchWord();

		case AddressingMode::AbsoluteX: {
			uint16_t base = FetchWord();
			uint16_t effective = base + X;
			if (pageBoundaryCrossed != nullptr)
				*pageBoundaryCrossed = (base & PAGE_NUMBER) != (effective & PAGE_NUMBER);
			return effective;
		}

		case AddressingMode::AbsoluteY: {
			uint16_t base = FetchWord();
			uint16_t effective = base + Y;
			if (pageBoundaryCrossed != nullptr)
				*pageBoundaryCrossed = (base & PAGE_NUMBER) != (effective & PAGE_NUMBER);
			return effective;
		}

		case AddressingMode::Indirect: {
			uint16_t base = FetchWord();
			uint8_t lo = memoryBus.Read(base);
			uint8_t hi = memoryBus.Read((base & 0xFF00) | ((base + 1) & 0x00FF));
			return (hi << 8) | lo;
		}

		case AddressingMode::IndirectX: {
			uint8_t zp = (Fetch() + X) & 0xFF;
			uint16_t effective = memoryBus.Read(zp) | (memoryBus.Read((zp + 1) & 0xFF) << 8);
			return effective;
		}

		case AddressingMode::IndirectY: {
			uint8_t zp = Fetch();
			uint16_t base = memoryBus.Read(zp) | (memoryBus.Read((zp + 1) & 0xFF) << 8);
			uint16_t effective = base + Y;
			if(pageBoundaryCrossed != nullptr)
				*pageBoundaryCrossed = (base & PAGE_NUMBER) != (effective & PAGE_NUMBER);
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

	void CPU::RequestNMI()
	{
		NMI();
	}

#pragma region Single OpCodes

	void CPU::NMI()
	{
		// Push the current PC onto the stack
		PushStackWord(PC);

		// Push the status register onto the stack
		SetBreakFlag(false); // Clear the Break flag before pushing
		PushStack(P);

		// Disable further interrupts
		SetInterruptFlag(true);

		// Read the NMI vector (0xFFFA-0xFFFB) and jump to that address
		PC = memoryBus.ReadWord(0xFFFA);

		// The NMI takes 7 cycles to execute
		cycles += 7;
	}

	void CPU::BRK()
	{
		// Providing an extra byte of spacing for a break mark (identifying a reason for the break.)
		PC += 2;

		//Push the return address onto the stack
		PushStackWord(PC);

		// Set break to 1 before pushing P onto the stack.
		SetBreakFlag(true);

		// Push status register onto stack.
		PushStack(P);

		// Set interrupt bit.
		SetInterruptFlag(true);

		// Fetch the interrupt vector from $FFFE/$FFFF and set the PC
		PC = memoryBus.ReadWord(0xFFFE);

		Utils::Logger::Warning("CPU ENTERED INTERRUPT HANDLER at ", Utils::Logger::Uint16ToHex(PC));

		cycles += 7;
	}

	void CPU::JSR()
	{
		//Push the return address to the stack, +1 to get the return address.
		PushStackWord(PC + 1);

		Utils::Logger::Info("Pushing PC onto stack - ", Utils::Logger::Uint16ToHex(PC + 1));

		//Preform normal JMP
		JMP(AddressingMode::Absolute);

		cycles += 3; //Accounting for JMP cycles.
	}

	void CPU::RTS()
	{
		//Get the return addy from stack
		uint16_t ret = PullStackWord();

		PC = ret + 1;

		cycles += 6;
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

		cycles += 6;  // RTI takes 6 cycles
	}


#pragma endregion

#pragma region Illegal OpCodes

	void CPU::SLO_ZPX()
	{
		// Fetch the zero page address, then add X to it (with wrapping around)
		uint8_t zeroPageAddr = Fetch();
		uint8_t addr = zeroPageAddr + X;

		// Read the value from the zero page address
		uint8_t value = memoryBus.Read(addr);

		// Perform an ASL (Arithmetic Shift Left) operation on the value
		SetCarryFlag(value & 0x80);  // Set the carry flag if the high bit is set
		value <<= 1;

		// Write the shifted value back to the zero page address
		memoryBus.Write(addr, value);

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

		uint8_t result = memoryBus.Read(addr) & SP;

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

		cycles += 3;
	}

	void CPU::PHP()
	{
		uint8_t status = P;

		SetBreakFlag(true);
		SetUnusedFlag(true); // Should always be 1 anyway.

		PushStack(P);

		P = status;

		cycles += 3;
	}

	void CPU::PLA()
	{
		A = PullStack();

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 4;
	}

	void CPU::PLP()
	{
		P = PullStack();

		SetBreakFlag(false);
		SetUnusedFlag(true);

		cycles += 4;
	}

#pragma endregion

	void CPU::RementMemory(AddressingMode mode, bool increment, int cycles)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t value = memoryBus.Read(address);

		value = increment ? (value + 1) : (value - 1);

		memoryBus.Write(address, value);

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		this->cycles += cycles;
	}

	void CPU::RementRegister(uint8_t& reg, bool increment)
	{
		reg = increment ? (reg + 1) : (reg - 1);

		SetZeroFlag(reg == 0);
		SetNegativeFlag(reg & NEGATIVE_FLAG);

		this->cycles += 2;
	}

	void CPU::Branch(bool condition)
	{
		int8_t offset = Fetch();

		if (condition)
		{
			uint16_t oldPC = PC;

			PC += offset;

			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
				this->cycles += 2;
			else
				this->cycles++;
		}

		this->cycles += 2;
	}

	void CPU::NOP(AddressingMode mode, int cycles)
	{
		if (mode != AddressingMode::Implied)
		{
			if (mode == AddressingMode::Absolute || mode == AddressingMode::AbsoluteX)
				FetchWord();
			else
				Fetch();
		}

		this->cycles += cycles;
	}

	void CPU::INC(AddressingMode mode, int cycles)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t value = memoryBus.Read(address);

		value += 1;

		memoryBus.Write(address, value);

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		this->cycles += cycles;
	}

	void CPU::ORA(AddressingMode mode, int cycles)
	{
		bool crossedBoundary = false;
		uint16_t address = GetOperandAddress(mode, &crossedBoundary);
		uint8_t operand = (mode == AddressingMode::Immediate) ? address : memoryBus.Read(address);

		A |= operand;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		if (crossedBoundary)
			this->cycles += 1;

		this->cycles += cycles;
	}

	void CPU::RLA(AddressingMode mode, int cycles)
	{
		uint16_t address = GetOperandAddress(mode);
		uint8_t value = memoryBus.Read(address);

		uint8_t carryIn = (P & CARRY_FLAG) ? 0x01 : 0x00;
		SetCarryFlag(value & 0x80);
		value = (value << 1) | carryIn;

		memoryBus.Write(address, value);

		A &= value;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		this->cycles += cycles;
	}

	void CPU::Rotate(AddressingMode mode, bool rotateLeft, int cycles)
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
			value = memoryBus.Read(address);

			SetCarryFlag(rotateLeft ? (value & 0x80) : (value & 0x01));

			if (rotateLeft)
				value = (value << 1) | carryIn;
			else
				value = (value >> 1) | carryIn;

			memoryBus.Write(address, value);
		}

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		this->cycles += cycles;
	}

	void CPU::Shift(AddressingMode mode, bool shiftLeft, int cycles)
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
			uint8_t operand = memoryBus.Read(address);

			SetCarryFlag(operand & (shiftLeft ? 0x80 : 0x01));

			if (shiftLeft)
				operand <<= 1;
			else
				operand >>= 1;

			memoryBus.Write(address, operand);

			SetZeroFlag(operand == 0);
			SetNegativeFlag(shiftLeft ? (operand & NEGATIVE_FLAG) : 0);
		}

		this->cycles += cycles;
	}

	void CPU::BIT(AddressingMode mode, int cycles)
	{
		uint8_t memValue = memoryBus.Read(GetOperandAddress(mode));

		uint8_t result = A & memValue;

		SetZeroFlag(result == 0);
		SetNegativeFlag(memValue & NEGATIVE_FLAG);
		SetOverflowFlag(memValue & OVERFLOW_FLAG);

		this->cycles += cycles;
	}

	void CPU::EOR(AddressingMode mode, int cycles)
	{
		bool crossedBoundary = false;
		uint16_t address = GetOperandAddress(mode, &crossedBoundary);
		uint8_t operand = (mode == AddressingMode::Immediate) ? address : memoryBus.Read(address);

		A ^= operand;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		if (crossedBoundary)
			this->cycles += 1;

		this->cycles += cycles;
	}

	void CPU::LD(AddressingMode mode, uint8_t& reg, int cycles)
	{
		bool crossedBoundary = false;
		uint16_t address = GetOperandAddress(mode, &crossedBoundary);

		if (mode == AddressingMode::Immediate)
			reg = address;
		else
			reg = memoryBus.Read(address);

		SetZeroFlag(reg == 0);
		SetNegativeFlag(reg & NEGATIVE_FLAG);

		if (crossedBoundary)
			this->cycles += 1;

		this->cycles += cycles;
	}

	void CPU::ST(AddressingMode mode, uint8_t& reg, int cycles)
	{
		uint16_t address = GetOperandAddress(mode);

		memoryBus.Write(address, reg);

		this->cycles += cycles;
	}

	void CPU::ADC_SBC(AddressingMode mode, int cycles, bool isSubtraction)
	{
		bool crossedBoundary = false;
		uint16_t operandAddress = GetOperandAddress(mode, &crossedBoundary);

		uint8_t operand = (mode == AddressingMode::Immediate) ? operandAddress : memoryBus.Read(operandAddress);

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

		if (crossedBoundary)
			this->cycles++;

		this->cycles += cycles;
	}

	void CPU::AND(AddressingMode mode, int cycles)
	{
		bool crossedBoundary = false;
		uint16_t operandAddress = GetOperandAddress(mode, &crossedBoundary);

		uint8_t operand = (mode == AddressingMode::Immediate) ? operandAddress : memoryBus.Read(operandAddress);

		A &= operand;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		if (crossedBoundary)
			this->cycles++;

		this->cycles += cycles;
	}

	void CPU::JMP(AddressingMode mode)
	{
		uint16_t address = GetOperandAddress(mode);

		PC = address;

		cycles += (mode == AddressingMode::Absolute) ? 3 : 5;
	}

	void CPU::CMP(AddressingMode mode, uint8_t& reg, int cycles)
	{
		bool crossedBoundary = false;
		uint16_t operandAddress = GetOperandAddress(mode, &crossedBoundary);

		uint8_t operand = (mode == AddressingMode::Immediate) ? operandAddress : memoryBus.Read(operandAddress);

		uint8_t result = reg - operand;

		SetCarryFlag(reg >= operand);
		SetZeroFlag(result == 0);
		SetNegativeFlag(result & NEGATIVE_FLAG);

		if (crossedBoundary)
			this->cycles++;

		this->cycles += cycles;
	}

	void CPU::Transfer(uint8_t& dest, uint8_t& src, bool setFlags)
	{
		dest = src;

		if (setFlags)
		{
			SetNegativeFlag(dest & NEGATIVE_FLAG);
			SetZeroFlag(dest == 0);
		}

		this->cycles += 2;
	}

	void CPU::FlagOperation(uint8_t flag, bool setFlag)
	{
		if (setFlag)
		{
			P |= flag;
		}
		else {
			P &= ~flag;
		}

		this->cycles += 2;
	}

	void CPU::PushStack(uint8_t value)
	{
		memoryBus.Write(StackStart + SP, value);  // Store the value at the current stack pointer address
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
		uint8_t stValue = memoryBus.Read(StackStart + SP);
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
		return memoryBus.Read(PC++);
	}

	// Fetch the next word (16-bit) and increment the PC by 2 bytes
	uint16_t CPU::FetchWord()
	{
		uint16_t word = memoryBus.ReadWord(PC);
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

			uint8_t stackValue = memoryBus.Read(currentAddress);

			Utils::Logger::Info("  [", Utils::Logger::Uint16ToHex(currentAddress), "] = ", Utils::Logger::Uint8ToHex(stackValue));

			count++;
		}

		Utils::Logger::Info("-------------------------------------");
	}
}