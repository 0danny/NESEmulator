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

		if (opcodeTable[opcode] != nullptr)
		{
			Utils::Logger::Info("[", Utils::Logger::Uint16ToHex(PC), "] OpCode (", Utils::Logger::Uint8ToHex(opcode), ") - ", opcodeStruct.name);

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
		opcodeTable[0xA6] = &CPU::LDX_ZP;

		//LDA
		opcodeTable[0xA9] = &CPU::LDA_Imm;
		opcodeTable[0xAD] = &CPU::LDA_Abs;
		opcodeTable[0xA5] = &CPU::LDA_ZP;
		opcodeTable[0xB1] = &CPU::LDA_IndirectY;
		opcodeTable[0xA1] = &CPU::LDA_IndirectX;

		//LDY
		opcodeTable[0xAC] = &CPU::LDY_Abs;
		opcodeTable[0xA0] = &CPU::LDY_Imm;
		opcodeTable[0xA4] = &CPU::LDY_ZP;

		opcodeTable[0xBB] = &CPU::LAS;

		//STX
		opcodeTable[0x8E] = &CPU::STX_Abs;
		opcodeTable[0x86] = &CPU::STX_ZP;

		//SLO
		opcodeTable[0x17] = &CPU::SLO_ZPX;

		//STA
		opcodeTable[0x85] = &CPU::STA_ZP;
		opcodeTable[0x8D] = &CPU::STA_Abs;
		opcodeTable[0x99] = &CPU::STA_AbsY;
		opcodeTable[0x9D] = &CPU::STA_AbsX;
		opcodeTable[0x91] = &CPU::STA_IndirectY;
		opcodeTable[0x81] = &CPU::STA_IndirectX;

		opcodeTable[0x8C] = &CPU::STY_Abs;
		opcodeTable[0x84] = &CPU::STY_ZP;

		opcodeTable[0xE9] = &CPU::SBC_Imm;
		opcodeTable[0xED] = &CPU::SBC_Abs;
		opcodeTable[0xE1] = &CPU::SBC_IndirectX;
		opcodeTable[0xE5] = &CPU::SBC_ZP;

		//ADC
		opcodeTable[0x6D] = &CPU::ADC_Abs;
		opcodeTable[0x69] = &CPU::ADC_Imm;
		opcodeTable[0x65] = &CPU::ADC_ZP;
		opcodeTable[0x61] = &CPU::ADC_IndirectX;

		//INC
		opcodeTable[0xE6] = &CPU::INC_ZP;
		opcodeTable[0xEE] = &CPU::INC_Abs;

		//AND
		opcodeTable[0x29] = &CPU::AND_Imm;
		opcodeTable[0x25] = &CPU::AND_ZP;
		opcodeTable[0x21] = &CPU::AND_IndirectX;
		opcodeTable[0x2D] = &CPU::AND_Abs;

		//JMP
		opcodeTable[0x4C] = &CPU::JMP_Abs;
		opcodeTable[0x6C] = &CPU::JMP_Indirect;

		//ASL
		opcodeTable[0x0A] = &CPU::ASL_A;
		opcodeTable[0x0E] = &CPU::ASL_Abs;
		opcodeTable[0x06] = &CPU::ASL_ZP;

		//LSR
		opcodeTable[0x4A] = &CPU::LSR_A;
		opcodeTable[0x46] = &CPU::LSR_ZP;
		opcodeTable[0x4E] = &CPU::LSR_Abs;

		//Compare OpCodes
		opcodeTable[0xC9] = &CPU::CMP_Imm;
		opcodeTable[0xC5] = &CPU::CMP_ZP;
		opcodeTable[0xCD] = &CPU::CMP_Abs;
		opcodeTable[0xC1] = &CPU::CMP_IndirectX;

		opcodeTable[0xE0] = &CPU::CPX_Imm;
		opcodeTable[0xE4] = &CPU::CPX_ZP;
		opcodeTable[0xEC] = &CPU::CPX_Abs;

		opcodeTable[0xC0] = &CPU::CPY_Imm;
		opcodeTable[0xC4] = &CPU::CPY_ZP;
		opcodeTable[0xCC] = &CPU::CPY_Abs;

		//EOR
		opcodeTable[0x4D] = &CPU::EOR_Abs;
		opcodeTable[0x49] = &CPU::EOR_Imm;
		opcodeTable[0x41] = &CPU::EOR_IndirectX;
		opcodeTable[0x45] = &CPU::EOR_ZP;

		//Flag OpCodes
		opcodeTable[0xD8] = &CPU::CLD;
		opcodeTable[0x18] = &CPU::CLC;
		opcodeTable[0xB8] = &CPU::CLV;
		opcodeTable[0x78] = &CPU::SEI;
		opcodeTable[0x38] = &CPU::SEC;
		opcodeTable[0xF8] = &CPU::SED;

		//Branch Opcodes
		opcodeTable[0xF0] = &CPU::BEQ;
		opcodeTable[0xD0] = &CPU::BNE;
		opcodeTable[0x10] = &CPU::BPL;
		opcodeTable[0x70] = &CPU::BVS;
		opcodeTable[0x50] = &CPU::BVC;
		opcodeTable[0x90] = &CPU::BCC;
		opcodeTable[0xB0] = &CPU::BCS;
		opcodeTable[0x30] = &CPU::BMI;

		//Decrement Opcodes
		opcodeTable[0xC6] = &CPU::DEC_ZP;
		opcodeTable[0xCE] = &CPU::DEC_Abs;


		opcodeTable[0x88] = &CPU::DEY;
		opcodeTable[0xCA] = &CPU::DEX;

		//Bit Test
		opcodeTable[0x2C] = &CPU::BIT_Abs;
		opcodeTable[0x24] = &CPU::BIT_ZP;

		//ORA
		opcodeTable[0x0D] = &CPU::ORA_Abs;
		opcodeTable[0x09] = &CPU::ORA_Imm;
		opcodeTable[0x05] = &CPU::ORA_ZP;
		opcodeTable[0x01] = &CPU::ORA_IndirectX;
		opcodeTable[0x11] = &CPU::ORA_IndirectY;

		//Single OpCodes
		opcodeTable[0xE8] = &CPU::INX;
		opcodeTable[0xC8] = &CPU::INY;

		opcodeTable[0x00] = &CPU::BRK;
		opcodeTable[0x20] = &CPU::JSR;
		opcodeTable[0x60] = &CPU::RTS;
		opcodeTable[0x40] = &CPU::RTI;

		//Rotate OpCodes
		opcodeTable[0x23] = &CPU::RLA_IndirectX;
		opcodeTable[0x33] = &CPU::RLA_IndirectY;

		opcodeTable[0x6A] = &CPU::ROR_A;
		opcodeTable[0x66] = &CPU::ROR_ZP;
		opcodeTable[0x6E] = &CPU::ROR_Abs;

		opcodeTable[0x2A] = &CPU::ROL_A;
		opcodeTable[0x26] = &CPU::ROL_ZP;
		opcodeTable[0x2E] = &CPU::ROL_Abs;

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

	uint8_t CPU::FetchIndirect(uint16_t& effectiveAddr, uint8_t reg)
	{
		uint8_t zeroPageAddr = Fetch();  // Fetch the zero-page address
		uint8_t indexedAddr = zeroPageAddr + reg;  // Add the X register to the zero-page address (wrap-around happens naturally)

		effectiveAddr = memoryBus.ReadWord(indexedAddr);  // Fetch the 16-bit address from the zero page, considering the wrap-around

		return memoryBus.Read(effectiveAddr);  // Fetch the value at the effective address
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
		//Push the return address to the stack, -1 to get the return address.
		PushStackWord(PC + 1);

		Utils::Logger::Info("Pushing PC onto stack - ", Utils::Logger::Uint16ToHex(PC + 1));

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

	void CPU::INY()
	{
		Y += 1;

		SetZeroFlag(Y == 0); // Set Zero flag if X is 0
		SetNegativeFlag(Y & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

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

#pragma region Decrement

	void CPU::DEC_ZP()
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

	void CPU::DEC_Abs()
	{
		//Decrement the value at the zero page memory address.
		uint16_t zp = FetchWord();

		uint8_t value = memoryBus.Read(zp) - 1;

		memoryBus.Write(zp, value);

		//Set flags
		SetZeroFlag(value == 0);
		SetNegativeFlag(value & NEGATIVE_FLAG);

		cycles += 6;
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
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (P & ZERO_FLAG)
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BPL()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (!(P & NEGATIVE_FLAG))
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BNE()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (!(P & ZERO_FLAG))
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BMI()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (P & NEGATIVE_FLAG)
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BVS()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (P & OVERFLOW_FLAG)
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BVC()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (!(P & OVERFLOW_FLAG))
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BCC()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (!(P & CARRY_FLAG))
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

	void CPU::BCS()
	{
		cycles += 2;  // Base cycles for opcode fetch and branch offset fetch
		int8_t offset = static_cast<int8_t>(Fetch());
		if (P & CARRY_FLAG)
		{
			uint16_t oldPC = PC;
			PC += offset;  // Add the signed offset to the PC
			// Check if the branch crosses a page boundary
			if ((oldPC & PAGE_NUMBER) != (PC & PAGE_NUMBER))
			{
				cycles += 2;
			}
			else
				cycles++;
		}
	}

#pragma endregion

#pragma region NOPs

	void CPU::NOP()
	{
		cycles += 2;
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

#pragma region INC

	void CPU::INC_ZP()
	{
		uint8_t addr = Fetch();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		value += 1;  // Increment the value by 1

		memoryBus.Write(addr, value);  // Write the incremented value back to the zero-page address

		// Set the Zero and Negative flags based on the result
		SetZeroFlag(value == 0);
		SetNegativeFlag(value & 0x80);

		cycles += 5;  // INC zero-page takes 5 cycles
	}

	void CPU::INC_Abs()
	{
		uint16_t addr = FetchWord();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		value += 1;  // Increment the value by 1

		memoryBus.Write(addr, value);  // Write the incremented value back to the zero-page address

		// Set the Zero and Negative flags based on the result
		SetZeroFlag(value == 0);
		SetNegativeFlag(value & 0x80);

		cycles += 6;  // INC zero-page takes 5 cycles
	}


#pragma endregion

#pragma region ORA

	void CPU::ORA_Imm()
	{
		uint8_t value = Fetch();  // Fetch the zero-page address

		A |= value;  // Perform bitwise OR with the accumulator

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 2;  // ORA zero-page takes 3 cycles
	}

	void CPU::ORA_Abs()
	{
		uint8_t value = memoryBus.Read(FetchWord());  // Fetch the value

		A |= value;  // Perform bitwise OR with the accumulator

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 4;  // ORA zero-page takes 3 cycles
	}

	void CPU::ORA_ZP()
	{
		uint8_t addr = Fetch();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		A |= value;  // Perform bitwise OR with the accumulator

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 3;  // ORA zero-page takes 3 cycles
	}

	void CPU::ORA_IndirectX()
	{
		uint16_t effectiveAddr;
		uint8_t value = FetchIndirect(effectiveAddr, X);

		// Perform bitwise OR with the accumulator
		A |= value;

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 6;  // ORA (indirect, X) takes 6 cycles
	}

	void CPU::ORA_IndirectY()
	{
		uint8_t zeroPageAddr = Fetch();  // Fetch the zero-page address (the pointer)

		// Read the base address from the zero page with manual wrapping
		uint16_t lowByte = memoryBus.Read(zeroPageAddr);
		uint16_t highByte = memoryBus.Read((zeroPageAddr + 1) & 0xFF);

		uint16_t baseAddr = (highByte << 8) | lowByte;  // Combine high and low bytes
		uint16_t effectiveAddr = baseAddr + Y;  // Add Y to the base address to get the effective address

		uint8_t value = memoryBus.Read(effectiveAddr);  // Fetch the value at the effective address

		// Perform bitwise OR with the accumulator
		A |= value;

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);  // Set the Negative flag if bit 7 of the result is set

		cycles += 5;  // Default cycle count

		// Handle page boundary crossing
		if ((baseAddr & 0xFF00) != (effectiveAddr & 0xFF00))
		{
			cycles += 1;  // Add 1 cycle if a page boundary is crossed
		}
	}

#pragma endregion

#pragma region RLA

	void CPU::RLA_IndirectX()
	{
		uint16_t effectiveAddr;
		uint8_t value = FetchIndirect(effectiveAddr, X);

		uint8_t carryIn = (P & CARRY_FLAG) ? 1 : 0;
		SetCarryFlag(value & 0x80);  // Set the carry flag to the high bit of the value
		value = (value << 1) | carryIn;

		memoryBus.Write(effectiveAddr, value);

		A &= value;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 8;
	}

	void CPU::RLA_IndirectY()
	{
		uint16_t effectiveAddr;
		uint8_t value = FetchIndirect(effectiveAddr, Y);

		// Perform a ROL (Rotate Left) operation on the value
		uint8_t carryIn = (P & CARRY_FLAG) ? 1 : 0;
		SetCarryFlag(value & 0x80);  // Set the carry flag to the high bit of the value
		value = (value << 1) | carryIn;

		// Write the rotated value back into memory
		memoryBus.Write(effectiveAddr, value);

		// Perform an AND operation between the result and the accumulator
		A &= value;

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 8;  // RLA (indirect),Y takes 8 cycles
	}

	void CPU::ROR_A()
	{
		uint8_t carryIn = (P & CARRY_FLAG) ? 0x80 : 0x00;

		SetCarryFlag(A & 0x01);

		A = (A >> 1) | carryIn;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 2;
	}

	void CPU::ROR_ZP()
	{
		uint8_t addr = Fetch();

		uint8_t value = memoryBus.Read(addr);

		uint8_t carryIn = (P & CARRY_FLAG) ? 0x80 : 0x00;

		SetCarryFlag(value & 0x01);

		value = (value >> 1) | carryIn;

		SetZeroFlag(value == 0);

		SetNegativeFlag(value & 0x80);

		memoryBus.Write(addr, value);

		cycles += 5;
	}

	void CPU::ROR_Abs()
	{
		uint16_t addr = FetchWord();

		uint8_t value = memoryBus.Read(addr);

		uint8_t carryIn = (P & CARRY_FLAG) ? 0x80 : 0x00;

		SetCarryFlag(value & 0x01);

		value = (value >> 1) | carryIn;

		SetZeroFlag(value == 0);

		SetNegativeFlag(value & 0x80);

		memoryBus.Write(addr, value);

		cycles += 6;
	}

	void CPU::ROL_ZP()
	{
		uint8_t addr = Fetch();

		uint8_t value = memoryBus.Read(addr);

		uint8_t carryIn = (P & CARRY_FLAG) ? 0x01 : 0x00;

		SetCarryFlag(value & 0x80);

		value = (value << 1) | carryIn;

		SetZeroFlag(value == 0);

		SetNegativeFlag(value & 0x80);

		memoryBus.Write(addr, value);

		cycles += 5;
	}

	void CPU::ROL_Abs()
	{
		uint16_t addr = FetchWord();

		uint8_t value = memoryBus.Read(addr);

		uint8_t carryIn = (P & CARRY_FLAG) ? 0x01 : 0x00;

		SetCarryFlag(value & 0x80);

		value = (value << 1) | carryIn;

		SetZeroFlag(value == 0);

		SetNegativeFlag(value & 0x80);

		memoryBus.Write(addr, value);

		cycles += 6;
	}

	void CPU::ROL_A()
	{
		uint8_t carryIn = (P & CARRY_FLAG) ? 0x01 : 0x00;

		SetCarryFlag(A & NEGATIVE_FLAG);

		A = (A << 1) | carryIn;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & 0x80);

		cycles += 2;
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

	void CPU::ASL_Abs()
	{
		uint16_t addr = FetchWord();  // Fetch the 16-bit absolute address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the absolute address

		// Perform an ASL (Arithmetic Shift Left) operation on the value
		SetCarryFlag(value & 0x80);  // Set the carry flag if the high bit (bit 7) is set
		value <<= 1;  // Shift the value left by 1

		// Write the shifted value back to the absolute address
		memoryBus.Write(addr, value);

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(value == 0);
		SetNegativeFlag(value & 0x80);

		cycles += 6;  // ASL absolute takes 6 cycles
	}

	void CPU::ASL_ZP()
	{
		uint16_t addr = Fetch();

		uint8_t value = memoryBus.Read(addr);

		SetCarryFlag(value & 0x80);

		value <<= 1;

		SetZeroFlag(value == 0);
		SetNegativeFlag(value & 0x80);

		memoryBus.Write(addr, value);

		cycles += 5;
	}


#pragma endregion

#pragma region LSR

	void CPU::LSR_A()
	{
		SetCarryFlag(A & 0x01);

		A >>= 1; // Bit shift once to the right.

		SetZeroFlag(A == 0);
		SetNegativeFlag(0);

		cycles += 2;
	}

	void CPU::LSR_ZP()
	{
		uint8_t addr = Fetch();
		uint8_t value = memoryBus.Read(addr);

		SetCarryFlag(value & 0x01);

		value >>= 1;

		SetZeroFlag(value == 0);
		SetNegativeFlag(false);

		memoryBus.Write(addr, value);

		cycles += 5;
	}

	void CPU::LSR_Abs()
	{
		uint16_t addr = FetchWord();
		uint8_t value = memoryBus.Read(addr);

		SetCarryFlag(value & 0x01);

		value >>= 1;

		SetZeroFlag(value == 0);
		SetNegativeFlag(false);

		memoryBus.Write(addr, value);

		cycles += 6;
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

	void CPU::BIT_ZP()
	{
		uint8_t memValue = memoryBus.Read(Fetch());

		//Memory AND A
		uint8_t result = A & memValue;

		//Set N & V using Bits 7 and 6
		SetZeroFlag(result == 0);
		SetNegativeFlag(memValue & NEGATIVE_FLAG);
		SetOverflowFlag(memValue & OVERFLOW_FLAG);

		cycles += 3;
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

	void CPU::ADC_ZP()
	{
		uint16_t addr = Fetch();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		// Perform the addition with carry
		uint16_t temp = A + value + (P & CARRY_FLAG ? 1 : 0);

		// Set or clear the Carry flag
		SetCarryFlag(temp > 0xFF);

		// Set or clear the Zero flag
		SetZeroFlag((temp & 0xFF) == 0);

		// Set or clear the Overflow flag (if there was a signed overflow)
		SetOverflowFlag((~(A ^ value) & (A ^ temp) & 0x80) != 0);

		// Set or clear the Negative flag
		SetNegativeFlag(temp & 0x80);

		// Store the result back in the accumulator (only the lower 8 bits)
		A = static_cast<uint8_t>(temp);

		cycles += 3;  // ADC zero-page takes 3 cycles
	}

	void CPU::ADC_IndirectX()
	{
		uint8_t zeroPageAddr = Fetch();
		uint8_t indexedAddr = zeroPageAddr + X;

		uint16_t lowByte = memoryBus.Read(indexedAddr);
		uint16_t highByte = memoryBus.Read((indexedAddr + 1) & 0xFF);

		uint16_t effectiveAddr = (highByte << 8) | lowByte;

		uint8_t value = memoryBus.Read(effectiveAddr);

		// Perform the addition with carry
		uint16_t temp = A + value + (P & CARRY_FLAG ? 1 : 0);

		// Set or clear the Carry flag
		SetCarryFlag(temp > 0xFF);

		// Set or clear the Zero flag
		SetZeroFlag((temp & 0xFF) == 0);

		// Set or clear the Overflow flag (if there was a signed overflow)
		SetOverflowFlag((~(A ^ value) & (A ^ temp) & 0x80) != 0);

		// Set or clear the Negative flag
		SetNegativeFlag(temp & NEGATIVE_FLAG);

		// Store the result back in the accumulator (only the lower 8 bits)
		A = static_cast<uint8_t>(temp);

		cycles += 6;
	}

	void CPU::ADC_IndirectY()
	{
		
	}



#pragma endregion

#pragma region EOR

	void CPU::EOR_Imm()
	{
		uint8_t value = Fetch();  // Fetch the immediate

		A ^= value;  // Perform exclusive OR with the accumulator

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 2;
	}

	void CPU::EOR_Abs()
	{
		uint16_t abs = FetchWord();  // Fetch the address

		A ^= memoryBus.Read(abs);  // Perform exclusive OR with the accumulator

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 4;
	}

	void CPU::EOR_ZP()
	{
		uint8_t zp = Fetch();  // Fetch the zero-page address

		A ^= memoryBus.Read(zp);  // Perform exclusive OR with the accumulator

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 3;
	}

	void CPU::EOR_IndirectX()
	{
		uint8_t zeroPageAddr = Fetch();
		uint8_t indexedAddr = zeroPageAddr + X;

		uint16_t lowByte = memoryBus.Read(indexedAddr);
		uint16_t highByte = memoryBus.Read((indexedAddr + 1) & 0xFF);

		uint16_t effectiveAddr = (highByte << 8) | lowByte;

		uint8_t value = memoryBus.Read(effectiveAddr);

		A ^= value;

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 6;
	}

#pragma endregion

#pragma region SLO

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

	void CPU::LDX_ZP()
	{
		uint16_t zp = Fetch();

		X = memoryBus.Read(zp);

		SetZeroFlag(X == 0); // Set Zero flag if X is 0
		SetNegativeFlag(X & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set

		cycles += 3;
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
		uint16_t address = FetchWord();

		Utils::Logger::Debug("LDA ABS to address ", Utils::Logger::Uint16ToHex(address));

		A = memoryBus.Read(address);

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

	void CPU::LDA_IndirectX()
	{
		uint8_t indexedAddr = Fetch() + X;

		uint16_t lowByte = memoryBus.Read(indexedAddr);
		uint16_t highByte = memoryBus.Read((indexedAddr + 1) & 0xFF);

		uint16_t effectiveAddr = (highByte << 8) | lowByte;

		A = memoryBus.Read(effectiveAddr);

		Utils::Logger::Debug("LDA_IndirectX has loaded ", Utils::Logger::Uint8ToHex(A), " into ", Utils::Logger::Uint16ToHex(effectiveAddr));

		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);  // Set the Negative flag based on the most significant bit (bit 7)

		cycles += 6;  // LDA (indirect, X) takes 6 cycles
	}

	void CPU::LDA_IndirectY()
	{
		uint8_t zeroPageAddr = Fetch();  // Fetch the zero-page address (the pointer)

		// Read the base address from the zero page with manual wrapping
		uint16_t lowByte = memoryBus.Read(zeroPageAddr);
		uint16_t highByte = memoryBus.Read((zeroPageAddr + 1) & 0xFF);

		uint16_t baseAddr = (highByte << 8) | lowByte;  // Combine high and low bytes
		uint16_t effectiveAddr = baseAddr + Y;  // Add Y to the base address to get the effective address

		A = memoryBus.Read(effectiveAddr);  // Load the value at the effective address into A

		// Set the flags
		SetZeroFlag(A == 0);  // Set if the accumulator is zero
		SetNegativeFlag(A & 0x80);  // Set if the high bit (bit 7) of the accumulator is set

		cycles += 5;  // Default cycle count

		// Handle page boundary crossing
		if ((baseAddr & 0xFF00) != (effectiveAddr & 0xFF00))
		{
			cycles += 1;  // Add 1 cycle if a page boundary is crossed
		}
	}

#pragma endregion

#pragma region LDY

	void CPU::LDY_Abs()
	{
		uint16_t address = FetchWord();

		Y = memoryBus.Read(address);

		SetZeroFlag(Y == 0);
		SetNegativeFlag(Y & NEGATIVE_FLAG);

		cycles += 4;
	}

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

	void CPU::STX_ZP()
	{
		//Loads the value of the X register into the zero page memory address.
		memoryBus.Write(Fetch(), X);

		cycles += 3;
	}

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
		uint8_t indexedAddr = Fetch() + Y;

		uint16_t effectiveAddr = memoryBus.Read(indexedAddr) | (memoryBus.Read((indexedAddr + 1) & 0xFF) << 8);

		memoryBus.Write(effectiveAddr, A);

		cycles += 6;
	}

	void CPU::STA_IndirectX()
	{
		uint8_t indexedAddr = Fetch() + X;

		uint16_t effectiveAddr = memoryBus.Read(indexedAddr) | (memoryBus.Read((indexedAddr + 1) & 0xFF) << 8);

		memoryBus.Write(effectiveAddr, A);

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

	void CPU::STY_Abs()
	{
		//Loads the value of the Y register into a memory address.
		memoryBus.Write(FetchWord(), Y);

		cycles += 4;
	}

#pragma endregion

#pragma region SBC

	void CPU::SBC_Imm()
	{
		uint8_t value = Fetch();  // Fetch the immediate value

		// Invert the value and add it with carry
		uint16_t result = A - value - (P & CARRY_FLAG ? 0 : 1);

		// Set or clear the carry flag
		SetCarryFlag(result <= 0xFF);

		// Set or clear the zero flag
		SetZeroFlag((result & 0xFF) == 0);

		// Set or clear the overflow flag
		bool overflow = ((A ^ result) & (A ^ value) & 0x80) != 0;
		SetOverflowFlag(overflow);

		// Set or clear the negative flag
		SetNegativeFlag(result & 0x80);

		// Store the result in the accumulator (only the lower 8 bits)
		A = result & 0xFF;

		cycles += 2;  // SBC immediate takes 2 cycles
	}

	void CPU::SBC_Abs()
	{
		uint8_t value = memoryBus.Read(FetchWord());  // Fetch the immediate value

		// Invert the value and add it with carry
		uint16_t result = A - value - (P & CARRY_FLAG ? 0 : 1);

		// Set or clear the carry flag
		SetCarryFlag(result <= 0xFF);

		// Set or clear the zero flag
		SetZeroFlag((result & 0xFF) == 0);

		// Set or clear the overflow flag
		bool overflow = ((A ^ result) & (A ^ value) & 0x80) != 0;
		SetOverflowFlag(overflow);

		// Set or clear the negative flag
		SetNegativeFlag(result & 0x80);

		// Store the result in the accumulator (only the lower 8 bits)
		A = result & 0xFF;

		cycles += 4;  // SBC immediate takes 2 cycles
	}

	void CPU::SBC_ZP()
	{
		uint8_t value = memoryBus.Read(Fetch());  // Fetch the immediate value

		// Invert the value and add it with carry
		uint16_t result = A - value - (P & CARRY_FLAG ? 0 : 1);

		// Set or clear the carry flag
		SetCarryFlag(result <= 0xFF);

		// Set or clear the zero flag
		SetZeroFlag((result & 0xFF) == 0);

		// Set or clear the overflow flag
		bool overflow = ((A ^ result) & (A ^ value) & 0x80) != 0;
		SetOverflowFlag(overflow);

		// Set or clear the negative flag
		SetNegativeFlag(result & 0x80);

		// Store the result in the accumulator (only the lower 8 bits)
		A = result & 0xFF;

		cycles += 3;  // SBC immediate takes 2 cycles
	}

	void CPU::SBC_IndirectX()
	{
		uint8_t zeroPageAddr = Fetch();
		uint8_t indexedAddr = zeroPageAddr + X;

		uint16_t lowByte = memoryBus.Read(indexedAddr);
		uint16_t highByte = memoryBus.Read((indexedAddr + 1) & 0xFF);

		uint16_t effectiveAddr = (highByte << 8) | lowByte;

		uint8_t value = memoryBus.Read(effectiveAddr);

		// Invert the value (for 2's complement subtraction)
		value = ~value;

		// Perform the addition with the inverted value and the carry flag
		uint16_t result = A + value + (P & CARRY_FLAG ? 1 : 0);

		SetCarryFlag(result > 0xFF);  // Set the carry flag based on the result (borrow is the inverse of carry in SBC)
		SetZeroFlag((result & 0xFF) == 0);  // Set if the result is zero
		SetOverflowFlag(((A ^ result) & (value ^ result) & 0x80) != 0);  // Set overflow flag if the sign of the result is incorrect
		SetNegativeFlag(result & 0x80);  // Set if the result is negative

		A = result & 0xFF;  // Store the result in the accumulator

		cycles += 6;  // SBC (Indirect,X) takes 6 cycles
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

	void CPU::AND_ZP()
	{
		uint8_t addr = Fetch();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		// Perform bitwise AND with the accumulator
		A &= value;

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 3;  // AND zero-page takes 3 cycles
	}

	void CPU::AND_Abs()
	{
		uint16_t addr = FetchWord();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		// Perform bitwise AND with the accumulator
		A &= value;

		// Set the Zero and Negative flags based on the result in the accumulator
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 4;
	}

	void CPU::AND_IndirectX()
	{
		uint8_t zeroPageAddr = Fetch();
		uint8_t indexedAddr = zeroPageAddr + X;

		uint16_t effectiveAddr = memoryBus.Read(indexedAddr) | (memoryBus.Read((indexedAddr + 1) & 0xFF) << 8);
		uint8_t value = memoryBus.Read(effectiveAddr);

		A &= value;

		// Set flags
		SetZeroFlag(A == 0);
		SetNegativeFlag(A & NEGATIVE_FLAG);

		cycles += 6;
	}

#pragma endregion

#pragma region JMP

	void CPU::JMP_Abs()
	{
		// JMP to address.
		PC = FetchWord();

		cycles += 3;
	}

	void CPU::JMP_Indirect()
	{
		uint16_t indirectAddress = FetchWord();  // Fetch the 16-bit address operand

		// Handle page boundary bug (NES-specific behavior)
		uint16_t addressLow = indirectAddress;
		uint16_t addressHigh = (indirectAddress & 0xFF00) | ((indirectAddress + 1) & 0x00FF);

		// Fetch the target address by reading two bytes from the indirect address
		uint16_t targetAddress = memoryBus.Read(addressLow) | (memoryBus.Read(addressHigh) << 8);

		PC = targetAddress;  // Set the program counter to the target address

		cycles += 5;  // Indirect JMP takes 5 cycles
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

	void CPU::CMP_Abs()
	{
		uint16_t addr = FetchWord();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		uint16_t result = A - value;  // Perform the comparison

		// Set the Carry flag if the accumulator is greater than or equal to the memory value
		SetCarryFlag(A >= value);

		// Set the Zero flag if the accumulator is equal to the memory value
		SetZeroFlag((result & 0xFF) == 0);

		// Set the Negative flag based on the result (if the result's most significant bit is set)
		SetNegativeFlag(result & 0x80);

		cycles += 4;  // CMP zero-page takes 3 cycles
	}

	void CPU::CMP_ZP()
	{
		uint8_t addr = Fetch();  // Fetch the zero-page address
		uint8_t value = memoryBus.Read(addr);  // Read the value from the zero-page address

		uint16_t result = A - value;  // Perform the comparison

		// Set the Carry flag if the accumulator is greater than or equal to the memory value
		SetCarryFlag(A >= value);

		// Set the Zero flag if the accumulator is equal to the memory value
		SetZeroFlag((result & 0xFF) == 0);

		// Set the Negative flag based on the result (if the result's most significant bit is set)
		SetNegativeFlag(result & 0x80);

		cycles += 3;  // CMP zero-page takes 3 cycles
	}

	void CPU::CMP_IndirectX()
	{
		uint8_t zeroPageAddr = Fetch();
		uint8_t indexedAddr = zeroPageAddr + X;

		uint16_t effectiveAddr = memoryBus.Read(indexedAddr) | (memoryBus.Read((indexedAddr + 1) & 0xFF) << 8);
		uint8_t value = memoryBus.Read(effectiveAddr);

		uint16_t result = A - value;  // Perform the comparison

		SetCarryFlag(A >= value);
		SetZeroFlag((result & 0xFF) == 0);
		SetNegativeFlag(result & NEGATIVE_FLAG);

		cycles += 6;
	}

	void CPU::CPX_Imm()
	{
		uint8_t operand = Fetch();  // Fetch the immediate value
		CompareOp(operand, X);
		cycles += 2;  // CPX immediate takes 2 cycles
	}

	void CPU::CPX_ZP()
	{
		uint16_t address = Fetch();  // Fetch the zero-page address
		uint8_t operand = memoryBus.Read(address);
		CompareOp(operand, X);
		cycles += 3;  // CPX zero-page takes 3 cycles
	}

	void CPU::CPX_Abs()
	{
		uint16_t address = FetchWord();  // Fetch the absolute address
		uint8_t operand = memoryBus.Read(address);
		CompareOp(operand, X);
		cycles += 4;  // CPX absolute takes 4 cycles
	}

	void CPU::CPY_Imm()
	{
		uint8_t operand = Fetch();  // Fetch the immediate value
		CompareOp(operand, Y);
		cycles += 2;  // CPY immediate takes 2 cycles
	}

	void CPU::CPY_ZP()
	{
		uint16_t address = Fetch();  // Fetch the zero-page address
		uint8_t operand = memoryBus.Read(address);
		CompareOp(operand, Y);
		cycles += 3;  // CPY zero-page takes 3 cycles
	}

	void CPU::CPY_Abs()
	{
		uint16_t address = FetchWord();  // Fetch the absolute address
		uint8_t operand = memoryBus.Read(address);
		CompareOp(operand, Y);
		cycles += 4;  // CPY absolute takes 4 cycles
	}

	void CPU::CompareOp(uint8_t operand, uint8_t reg)
	{
		uint16_t result = reg - operand;

		SetCarryFlag(reg >= operand);       // Set if X >= operand
		SetZeroFlag((result & 0xFF) == 0); // Set if result is zero
		SetNegativeFlag(result & 0x80);   // Set if bit 7 of result is set (negative)
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

	void CPU::SED()
	{
		SetDecimalFlag(true);

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

	void CPU::SEC()
	{
		SetCarryFlag(true);

		cycles += 2;
	}

	void CPU::CLV()
	{
		SetOverflowFlag(false);

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