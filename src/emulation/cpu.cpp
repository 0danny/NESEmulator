#include "cpu.h"

namespace Emulation
{
	CPU::CPU()
	{
		InitializeOpcodes();
		Reset();
	}

	void CPU::Reset()
	{
		A = X = Y = 0;

		SP = 0xFD;
		P = 0x34;

		PC = ReadWord(0xFFFC);  // Reset vector
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
			ThrowException("OpCode Not Implemented", Utils::Logger::Uint8ToHex(opcode));
		}
	}

	void CPU::InitializeOpcodes() 
	{
		// Clear the table
		std::fill(std::begin(opcodeTable), std::end(opcodeTable), nullptr);

		// Populate the table with opcode handlers
		opcodeTable[0xA2] = &CPU::LDX_Imm;
		opcodeTable[0xA9] = &CPU::LDA_Imm;

		opcodeTable[0xAD] = &CPU::LDA_Abs;

		opcodeTable[0x8E] = &CPU::STX_Abs;

		opcodeTable[0x85] = &CPU::STA_ZP;
		opcodeTable[0x8D] = &CPU::STA_Abs;

		opcodeTable[0x6D] = &CPU::ADC_Abs;

		opcodeTable[0x29] = &CPU::AND_Imm;

		opcodeTable[0x4C] = &CPU::JMP_Abs;

		//Flag OpCodes
		opcodeTable[0xD8] = &CPU::CLD;
		opcodeTable[0x78] = &CPU::SEI;

		//Branch Opcodes
		opcodeTable[0xF0] = &CPU::BEQ;
		opcodeTable[0xD0] = &CPU::BNE;
		opcodeTable[0x10] = &CPU::BPL;

		//Single OpCodes
		opcodeTable[0xE8] = &CPU::INX;
		opcodeTable[0x00] = &CPU::BRK;
		opcodeTable[0x20] = &CPU::JSR;
		opcodeTable[0x60] = &CPU::RTS;
		opcodeTable[0xEA] = &CPU::NOP;
		opcodeTable[0x9A] = &CPU::TXS;
	}

	void CPU::StartThread()
	{
		// Start CPU thread
		cpuThread = std::thread(&CPU::Loop, this);

		Utils::Logger::Info("CPU thread started - ", cpuThread.get_id());
	}

	void CPU::Cleanup()
	{
		cpuThread.join();
	}

	void CPU::Loop()
	{
		while (clocking)
		{
			Clock();

			std::this_thread::sleep_for(std::chrono::microseconds(16));
		}
	}

	void CPU::LoadRawProgram(const uint8_t* program, uint16_t programSize, uint16_t loadAddress)
	{
		for (uint16_t i = 0; i < programSize; ++i)
		{
			memory[loadAddress + i] = program[i];
		}

		// Set the reset vector to point to the program's start address
		WriteWord(0xFFFC, loadAddress);
	}

	void CPU::ThrowException(std::string reason, std::string error)
	{
		//Throw an exception when the CPU encouters an error.
		Utils::Logger::Error("--------------- Encountered an Exception! ---------------");
		Utils::Logger::Error(reason + " - ", error);
		PrintState();
		Utils::Logger::Error("---------------------------------------------------------");

		clocking = false;
	}

	void CPU::LoadPrgProgram(const std::vector<uint8_t>& prgRom)
	{
		// Assume PRG ROM is always loaded at 0x8000
		size_t prgSize = prgRom.size();

		if (prgSize == 16384) // 16 KB PRG ROM
		{
			Utils::Logger::Info("16 KB ROM, Mirroring...");

			// Load the PRG ROM into both 0x8000-0xBFFF and 0xC000-0xFFFF
			std::copy(prgRom.begin(), prgRom.end(), memory.begin() + 0x8000);
			std::copy(prgRom.begin(), prgRom.end(), memory.begin() + 0xC000);
		}
		else if (prgSize == 32768) // 32 KB PRG ROM
		{
			// Load the PRG ROM into 0x8000-0xFFFF
			std::copy(prgRom.begin(), prgRom.end(), memory.begin() + 0x8000);

			Utils::Logger::Info("32 KB ROM, Loading...");
		}
		else
		{
			Utils::Logger::Error("Unexpected PRG ROM size!");
		}

		//Ensure we load the correct reset vector.
		Reset();

		/*
		Utils::Logger::Info("First 30 instructions after reset:");
		for (int i = 0; i < 30; i++) {
			uint16_t addr = PC + i;
			Utils::Logger::Info(Utils::Logger::Uint16ToHex(addr) + ": " + Utils::Logger::Uint8ToHex(Read(addr)));
		}*/
	}

#pragma region Single OpCodes

	void CPU::NOP()
	{
		cycles += 2;
		return;
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
		PC = ReadWord(0xFFFE);

		Utils::Logger::Warning("CPU ENTERED INTERRUPT HANDLER");

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

	void CPU::RTS()
	{
		//Get the return addy from stack
		uint16_t ret = PullStackWord();

		PC = ret + 1;

		cycles += 6;
	}

#pragma endregion

#pragma region ADC

	void CPU::ADC_Abs()
	{
		//Adds a value from a specified memory location to the accumulator (A), along with the carry bit.
		uint8_t value = Read(FetchWord());

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
		A = Read(FetchWord());

		SetZeroFlag(A == 0); // Set Zero flag if A is 0
		SetNegativeFlag(A & NEGATIVE_FLAG); // Set Negative flag if bit 7 of A is set

		cycles += 4;
	}

#pragma endregion

#pragma region STX

	void CPU::STX_Abs()
	{
		//Loads the value of the X register into memory at the address.
		Write(FetchWord(), X);

		cycles += 4;
	}

#pragma endregion

#pragma region STA

	void CPU::STA_ZP()
	{
		//Zero page refers to the first 256 bytes of memory.

		//Store the value of the A register into memory at the zero-page address.
		Write(Fetch(), A);

		cycles += 3;
	}

	void CPU::STA_Abs()
	{
		//Store the value of the A register into memory at the absolute address.
		Write(FetchWord(), A);

		cycles += 4;
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

#pragma region TXS

	void CPU::TXS()
	{
		SP = X;

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

#pragma endregion

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
		return memory[PC++];
	}

	// Fetch the next word (16-bit) and increment the PC by 2 bytes
	uint16_t CPU::FetchWord()
	{
		uint16_t word = memory[PC] | (memory[PC + 1] << 8);
		PC += 2;
		return word;
	}

	// Read a byte from memory
	uint8_t CPU::Read(uint16_t addr)
	{
		//Check if the read is inside bounds (uint cant be negative, but we check anyways lmao)
		if (addr < 0 || addr > memory.size() - 1)
		{
			ThrowException("Attempted memory read outside bounds.", Utils::Logger::Uint16ToHex(addr));

			return 0;
		}
		else
		{
			return memory[addr];
		}
	}

	// Read a word (16-bit) from memory
	uint16_t CPU::ReadWord(uint16_t addr)
	{
		if (addr < 0 || addr > memory.size() - 1)
		{
			ThrowException("Attempted memory read word outside bounds.", Utils::Logger::Uint16ToHex(addr));

			return 0;
		}
		else
		{
			return memory[addr] | (memory[addr + 1] << 8);
		}
	}

	// Write a byte to memory
	void CPU::Write(uint16_t addr, uint8_t value)
	{
		// Check if the write is inside bounds
		if (addr < 0 || addr > memory.size() - 1)
		{
			ThrowException("Attempted memory write outside bounds.", Utils::Logger::Uint16ToHex(addr));
		}
		else
		{
			memory[addr] = value;
		}
	}

	// Write a word to memory
	void CPU::WriteWord(uint16_t addr, uint16_t value)
	{
		// Check if the write is inside bounds
		if (addr < 0 || addr > memory.size() - 2)  // Check if there's enough space for both bytes
		{
			ThrowException("Attempted memory write word outside bounds.", Utils::Logger::Uint16ToHex(addr));
		}
		else
		{
			// Write the low byte to the specified address
			Write(addr, LOBYTE(value));

			// Write the high byte to the next address
			Write(addr + 1, HIBYTE(value));
		}
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
		Utils::Logger::Info("Stack (top 4 bytes):");

		for (int i = 0; i < 4; ++i) {
			uint8_t stackValue = Read(StackStart + (SP + 1 + i));  // Corrected stack indexing

			Utils::Logger::Info("  [", Utils::Logger::Uint16ToHex(StackStart + (SP + 1 + i)), "] = ", Utils::Logger::Uint8ToHex(stackValue));
		}

		Utils::Logger::Info("-------------------------------------");
	}
}