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
		P = 0x34;

		PC = memoryBus.ReadWord(0xFFFC);  // Reset vector
		Utils::Logger::Info("Reset Vector - ", Utils::Logger::Uint16ToHex(PC));

		cycles = 0;
	}

	void CPU::Clock()
	{
		uint8_t opcode = Fetch();

		Utils::Logger::Info("OpCode - ", Utils::Logger::Uint8ToHex(opcode));

		if (opcodeTable[opcode] != nullptr)
		{
			(this->*opcodeTable[opcode])();  // Call the handler
		}
		else
		{
			ExceptionWrapper("OpCode Not Implemented", Utils::Logger::Uint8ToHex(opcode));
		}
	}

	void CPU::InitializeOpcodes() 
	{
		// Clear the table
		std::fill(std::begin(opcodeTable), std::end(opcodeTable), nullptr);

		// Populate the table with opcode handlers

		//LDX
		opcodeTable[0xA2] = &CPU::LDX_Imm;
		opcodeTable[0xAE] = &CPU::LDX_Abs;

		//LDA
		opcodeTable[0xA9] = &CPU::LDA_Imm;
		opcodeTable[0xAD] = &CPU::LDA_Abs;
		opcodeTable[0xA5] = &CPU::LDA_ZP;

		//LDY
		opcodeTable[0xA0] = &CPU::LDY_Imm;
		opcodeTable[0xA4] = &CPU::LDY_ZP;

		opcodeTable[0xBB] = &CPU::LAS;

		opcodeTable[0x8E] = &CPU::STX_Abs;

		//STA
		opcodeTable[0x85] = &CPU::STA_ZP;
		opcodeTable[0x8D] = &CPU::STA_Abs;
		opcodeTable[0x99] = &CPU::STA_AbsY;
		opcodeTable[0x9D] = &CPU::STA_AbsX;

		opcodeTable[0x91] = &CPU::STA_IndirectY;

		opcodeTable[0x84] = &CPU::STY_ZP;

		//ADC
		opcodeTable[0x6D] = &CPU::ADC_Abs;
		opcodeTable[0x69] = &CPU::ADC_Imm;

		opcodeTable[0x29] = &CPU::AND_Imm;

		opcodeTable[0x4C] = &CPU::JMP_Abs;

		//ASL
		opcodeTable[0x0A] = &CPU::ASL_A;

		//Compare OpCodes
		opcodeTable[0xC9] = &CPU::CMP_Imm;

		//Flag OpCodes
		opcodeTable[0xD8] = &CPU::CLD;
		opcodeTable[0x18] = &CPU::CLC;
		opcodeTable[0x78] = &CPU::SEI;

		//Branch Opcodes
		opcodeTable[0xF0] = &CPU::BEQ;
		opcodeTable[0xD0] = &CPU::BNE;
		opcodeTable[0x10] = &CPU::BPL;
		opcodeTable[0x70] = &CPU::BVS;
		opcodeTable[0x90] = &CPU::BCC;
		opcodeTable[0xB0] = &CPU::BCS;

		//Decrement Opcodes
		opcodeTable[0xC6] = &CPU::DEC;
		opcodeTable[0x88] = &CPU::DEY;
		opcodeTable[0xCA] = &CPU::DEX;

		//Bit Test
		opcodeTable[0x2C] = &CPU::BIT_Abs;

		//Single OpCodes
		opcodeTable[0xE8] = &CPU::INX;
		opcodeTable[0x00] = &CPU::BRK;
		opcodeTable[0x20] = &CPU::JSR;
		opcodeTable[0x60] = &CPU::RTS;

		//Transfer Indexs
		opcodeTable[0xAA] = &CPU::TAX;
		opcodeTable[0xA8] = &CPU::TAY;
		opcodeTable[0xBA] = &CPU::TSX;
		opcodeTable[0x8A] = &CPU::TXA;
		opcodeTable[0x9A] = &CPU::TXS;
		opcodeTable[0x98] = &CPU::TYA;

		//Push - Pull Stack OpCodes
		opcodeTable[0x08] = &CPU::PHP;
		opcodeTable[0x48] = &CPU::PHA;
		opcodeTable[0x68] = &CPU::PLA;
		opcodeTable[0x28] = &CPU::PLP;

		//NOP Instructions
		opcodeTable[0xEA] = &CPU::NOP;
		opcodeTable[0x1A] = &CPU::NOP_Implied;
		opcodeTable[0x3A] = &CPU::NOP_Implied;
		opcodeTable[0x5A] = &CPU::NOP_Implied;
		opcodeTable[0x7A] = &CPU::NOP_Implied;
		opcodeTable[0xDA] = &CPU::NOP_Implied;
		opcodeTable[0xFA] = &CPU::NOP_Implied;

		opcodeTable[0x80] = &CPU::NOP_Imm;
		opcodeTable[0x82] = &CPU::NOP_Imm;
		opcodeTable[0x89] = &CPU::NOP_Imm;
		opcodeTable[0xC2] = &CPU::NOP_Imm;
		opcodeTable[0xE2] = &CPU::NOP_Imm;

		opcodeTable[0x04] = &CPU::NOP_ZP;
		opcodeTable[0x44] = &CPU::NOP_ZP;
		opcodeTable[0x64] = &CPU::NOP_ZP;
		opcodeTable[0x14] = &CPU::NOP_ZPX;
		opcodeTable[0x34] = &CPU::NOP_ZPX;
		opcodeTable[0x54] = &CPU::NOP_ZPX;
		opcodeTable[0x74] = &CPU::NOP_ZPX;
		opcodeTable[0xD4] = &CPU::NOP_ZPX;
		opcodeTable[0xF4] = &CPU::NOP_ZPX;

		opcodeTable[0x0C] = &CPU::NOP_Abs;
		opcodeTable[0x1C] = &CPU::NOP_AbsX;
		opcodeTable[0x3C] = &CPU::NOP_AbsX;
		opcodeTable[0x5C] = &CPU::NOP_AbsX;
		opcodeTable[0x7C] = &CPU::NOP_AbsX;
		opcodeTable[0xDC] = &CPU::NOP_AbsX;
		opcodeTable[0xFC] = &CPU::NOP_AbsX;


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
		PC++;

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

		//Utils::Logger::Warning("CPU ENTERED INTERRUPT HANDLER");

		cycles += 7;
	}

	void CPU::JSR()
	{
		//Push the return address to the stack, -1 to get the return address.
		PushStackWord(PC - 1);

		Utils::Logger::Info("Pushing PC onto stack - ", Utils::Logger::Uint16ToHex(PC - 1));

		//Preform normal JMP
		JMP_Abs();

		cycles += 3; //Accounting for JMP cycles.
	}

	void CPU::INX()
	{
		X += 1;

		SetZeroFlag(X == 0); // Set Zero flag if X is 0
		SetNegativeFlag(X & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

		cycles += 2;
	}

	void CPU::RTS()
	{
		//Get the return addy from stack
		uint16_t ret = PullStackWord();

		PC = ret + 1;

		cycles += 6;
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

#pragma region Decrement

	void CPU::DEC()
	{
		//Decrement the value at the zero page memory address.
		uint8_t zp = Fetch();

		uint8_t value = memoryBus.Read(zp) - 1;

		memoryBus.Write(zp, value);

		//Set flags
		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		cycles += 5;
	}

	void CPU::DEY()
	{
		Y--;

		SetZeroFlag(Y == 0);   // Set Zero flag if Y is 0
		SetNegativeFlag(Y & NEGATIVE_FLAG);  // Set Negative flag if bit 7 of Y is set

		cycles += 2;
	}

	void CPU::DEX()
	{
		X--;

		SetZeroFlag(X == 0);   // Set Zero flag if Y is 0
		SetNegativeFlag(X & NEGATIVE_FLAG);  // Set Negative flag if bit 7 of Y is set

		cycles += 2;
	}

#pragma endregion

#pragma region Branches

	void CPU::BEQ()
	{
		cycles++;

		int8_t operand = static_cast<int8_t>(Fetch());

		if (P & ZERO_FLAG)
		{
			uint16_t oldPC = PC;

			PC += operand;  // Add the signed offset to the PC
			cycles++;

			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles++;
			}
		}
	}

	void CPU::BPL()
	{
		cycles++;

		int8_t operand = static_cast<int8_t>(Fetch());

		if (!(P & NEGATIVE_FLAG))
		{
			uint16_t oldPC = PC;

			PC += operand;  // Add the signed offset to the PC
			cycles++;

			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles++;
			}
		}
	}

	void CPU::BNE()
	{
		cycles++;

		int8_t operand = static_cast<int8_t>(Fetch());

		if (!(P & ZERO_FLAG))
		{
			uint16_t oldPC = PC;

			PC += operand;  // Add the signed offset to the PC
			cycles++;

			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles++;
			}
		}
	}

	void CPU::BVS()
	{
		cycles++;

		int8_t offset = static_cast<int8_t>(Fetch());

		if (P & OVERFLOW_FLAG)
		{
			uint16_t oldPC = PC;

			PC += offset;  // Add the signed offset to the PC
			cycles++;

			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles++;
			}
		}
	}

	void CPU::BCC()
	{
		//Branch on carry clear.

		cycles++;

		int8_t offset = static_cast<int8_t>(Fetch());

		if (!(P & CARRY_FLAG))
		{
			uint16_t oldPC = PC;

			PC += offset;  // Add the signed offset to the PC
			cycles++;

			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles++;
			}
		}
	}

	void CPU::BCS()
	{
		//Branch on carry set.
		cycles++;

		int8_t offset = static_cast<int8_t>(Fetch());

		if (P & CARRY_FLAG)
		{
			uint16_t oldPC = PC;

			PC += offset;  // Add the signed offset to the PC
			cycles++;

			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles++;
			}
		}
	}

#pragma endregion

#pragma region NOPs

	void CPU::NOP()
	{
		cycles += 2;
		return;
	}

	void CPU::NOP_Implied() 
	{
		cycles += 2;
	}

	void CPU::NOP_Imm() 
	{
		Fetch();
		cycles += 2;
	}

	void CPU::NOP_ZP() 
	{
		Fetch();
		cycles += 3;
	}

	void CPU::NOP_ZPX() 
	{
		Fetch();
		cycles += 4;
	}

	void CPU::NOP_Abs() 
	{
		FetchWord();
		cycles += 4;
	}

	void CPU::NOP_AbsX() 
	{
		FetchWord();
		cycles += 4;
	}

#pragma endregion

#pragma region ASL

	void CPU::ASL_A()
	{
		SetCarryFlag(A & 0x80); //Set the carry to bit 7, we are shifting out.

		A <<= 1; //Bit shift once to the left.

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 2;
	}

#pragma endregion

#pragma region BIT

	void CPU::BIT_Abs()
	{
		uint8_t memValue = memoryBus.Read(FetchWord());

		//Memory AND A
		uint8_t result = A & memValue;

		//Set N & V using Bits 7 and 6
		SetZeroFlag(result == 0);
		SetNegativeFlag(memValue & NEGATIVE_FLAG);
		SetOverflowFlag(memValue & OVERFLOW_FLAG);

		cycles += 4;
	}

#pragma endregion

#pragma region ADC

	void CPU::ADC_Imm()
	{
		uint8_t value = Fetch();
		// Perform the addition with carry
		uint16_t result = A + value + (P & CARRY_FLAG ? 1 : 0);

		SetCarryFlag(result > 0xFF);
		SetZeroFlag(LOBYTE(result) == 0);
		SetNegativeFlag(result & NEGATIVE_FLAG);

		bool overflow = (~(A ^ value) & (A ^ result) & NEGATIVE_FLAG) != 0;

		SetOverflowFlag(overflow);

		A = LOBYTE(result);

		cycles += 2;
	}

	void CPU::ADC_Abs()
	{
		//Adds a value from a specified memory location to the accumulator (A), along with the carry bit.
		uint8_t value = memoryBus.Read(FetchWord());

		// Perform the addition with carry
		uint16_t result = A + value + (P & CARRY_FLAG ? 1 : 0);

		// Set or clear the Carry flag (if result is greater than 255)
		SetCarryFlag(result > 0xFF);

		// Set or clear the Zero flag (if result is 0)
		SetZeroFlag(LOBYTE(result) == 0);

		// Set or clear the Negative flag (if bit 7 of the result is set)
		SetNegativeFlag(result & NEGATIVE_FLAG);

		// Set or clear the Overflow flag (if there was a signed overflow)
		bool overflow = (~(A ^ value) & (A ^ result) & NEGATIVE_FLAG) != 0;

		SetOverflowFlag(overflow);

		// Store the result back in the accumulator (only the lower 8 bits)
		A = LOBYTE(result);

		cycles += 4;
	}

#pragma endregion

#pragma region LDX

	void CPU::LDX_Imm()
	{
		//Loads a value directly into X
		X = Fetch();

		SetZeroFlag(X == 0); // Set Zero flag if X is 0
		SetNegativeFlag(X & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

		cycles += 2;
	}

	void CPU::LDX_Abs()
	{
		uint16_t address = FetchWord();

		X = memoryBus.Read(address);

		SetZeroFlag(X == 0); // Set Zero flag if X is 0
		SetNegativeFlag(X & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

		cycles += 4;
	}

#pragma endregion

#pragma region LDA

	void CPU::LDA_Imm()
	{
		//Loads a value directly into Accumulator (A)
		A = Fetch();

		SetZeroFlag(A == 0); // Set Zero flag if A is 0
		SetNegativeFlag(A & NEGATIVE_FLAG); // Set Negative flag if bit 7 of A is set

		cycles += 2;
	}

	void CPU::LDA_Abs()
	{
		//Get value from memory.
		A = memoryBus.Read(FetchWord());

		SetZeroFlag(A == 0); // Set Zero flag if A is 0
		SetNegativeFlag(A & NEGATIVE_FLAG); // Set Negative flag if bit 7 of A is set

		cycles += 4;
	}

	void CPU::LDA_ZP()
	{
		A = memoryBus.Read(Fetch());

		SetZeroFlag(A == 0); // Set Zero flag if A is 0
		SetNegativeFlag(A & NEGATIVE_FLAG); // Set Negative flag if bit 7 of A is set

		cycles += 3;
	}

#pragma endregion

#pragma region LDY

	void CPU::LDY_Imm()
	{
		//Loads a value directly into X
		Y = Fetch();

		SetZeroFlag(Y == 0); // Set Zero flag if X is 0
		SetNegativeFlag(Y & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

		cycles += 2;
	}

	void CPU::LDY_ZP()
	{
		//Loads a value directly into X
		Y = memoryBus.Read(Fetch());

		SetZeroFlag(Y == 0); // Set Zero flag if X is 0
		SetNegativeFlag(Y & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

		cycles += 3;
	}

#pragma endregion

#pragma region STX

	void CPU::STX_Abs()
	{
		//Loads the value of the X register into memory at the address.
		memoryBus.Write(FetchWord(), X);

		cycles += 4;
	}

#pragma endregion

#pragma region STA

	void CPU::STA_ZP()
	{
		//Zero page refers to the first 256 bytes of memory.

		//Store the value of the A register into memory at the zero-page address.
		memoryBus.Write(Fetch(), A);

		cycles += 3;
	}

	void CPU::STA_Abs()
	{
		//Store the value of the A register into memory at the absolute address.
		memoryBus.Write(FetchWord(), A);

		cycles += 4;
	}

	void CPU::STA_IndirectY()
	{
		// Read the base address from the zero-page address
		uint16_t baseAddress = memoryBus.ReadWord(Fetch());

		// Calculate the effective address by adding the Y register to the base address
		uint16_t effectiveAddress = baseAddress + Y;

		// Store the contents of the accumulator at the effective address
		memoryBus.Write(effectiveAddress, A);

		cycles += 6;
	}

	void CPU::STA_AbsY()
	{
		// Calculate the effective address by adding the Y register to the base address
		uint16_t effectiveAddress = FetchWord() + Y;

		memoryBus.Write(effectiveAddress, A);

		cycles += 5;
	}

	void CPU::STA_AbsX()
	{
		// Calculate the effective address by adding the X register to the base address
		uint16_t effectiveAddress = FetchWord() + X;

		memoryBus.Write(effectiveAddress, A);

		cycles += 5;
	}

#pragma endregion

#pragma region STY

	void CPU::STY_ZP()
	{
		//Loads the value of the Y register into a zero page memory address.
		memoryBus.Write(Fetch(), Y);

		cycles += 3;
	}

#pragma endregion

#pragma region AND

	void CPU::AND_Imm()
	{
		A = A & Fetch(); // Perform bitwise AND with the immediate value

		SetZeroFlag(A == 0);  // Set the Zero flag if the result is 0
		SetNegativeFlag(A & NEGATIVE_FLAG);  // Set the Negative flag if bit 7 is set

		cycles += 2;
	}

#pragma endregion

#pragma region JMP

	void CPU::JMP_Abs()
	{
		// JMP to address.
		PC = FetchWord();

		cycles += 3;
	}

#pragma endregion

#pragma region Compare OpCodes

	void CPU::CMP_Imm()
	{
		uint8_t operand = Fetch(); // Fetch the immediate operand

		uint8_t result = A - operand;

		// Set the Carry flag if the accumulator is greater than or equal to the operand
		SetCarryFlag(A >= operand);
		SetZeroFlag(result == 0);
		SetNegativeFlag(result & NEGATIVE_FLAG);

		cycles += 2;
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

		P = status; // Restore the original status register without modifying the Break flag

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

#pragma region Transfer Indexs

	void CPU::TXS()
	{
		SP = X;

		cycles += 2;
	}

	void CPU::TSX()
	{
		X = SP;

		SetNegativeFlag(X & NEGATIVE_FLAG);
		SetZeroFlag(X == 0);

		cycles += 2;
	}

	void CPU::TXA()
	{
		A = X;

		SetNegativeFlag(A & NEGATIVE_FLAG);
		SetZeroFlag(A == 0);

		cycles += 2;
	}

	void CPU::TAX()
	{
		X = A;

		SetNegativeFlag(X & NEGATIVE_FLAG);
		SetZeroFlag(X == 0);

		cycles += 2;
	}

	void CPU::TAY()
	{
		Y = A;

		SetNegativeFlag(Y & NEGATIVE_FLAG);
		SetZeroFlag(Y == 0);

		cycles += 2;
	}

	void CPU::TYA()
	{
		A = Y;

		SetNegativeFlag(A & NEGATIVE_FLAG);
		SetZeroFlag(A == 0);

		cycles += 2;
	}

#pragma endregion

#pragma region Flag OpCodes

	void CPU::CLD()
	{
		SetDecimalFlag(false);

		cycles += 2;
	}

	void CPU::SEI()
	{
		SetInterruptFlag(true);

		cycles += 2;
	}

	void CPU::CLC()
	{
		SetCarryFlag(false);

		cycles += 2;
	}

#pragma endregion

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