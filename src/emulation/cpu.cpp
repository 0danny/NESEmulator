#include "cpu.h"

namespace Emulation
{
    CPU::CPU()
    {
        Reset();
    }

    void CPU::Reset()
    {
        A = X = Y = 0;
        SP = 0xFD;
        P = 0x34;
        PC = ReadWord(0xFFFC);  // Reset vector
        cycles = 0;
    }

    void CPU::Execute()
    {
        uint8_t opcode = Fetch();

        Utils::Logger::Info("OpCode - ", Utils::Logger::Uint8ToHex(opcode));

        switch (opcode) 
        {
        //LDX
        case 0xA2:
            LDX_Imm(Fetch());
            break;

        //LDA
        case 0xA9:
            LDA_Imm(Fetch());
            break;
        case 0xAD:
            LDA_Abs(FetchWord());
            break;

        //STX
        case 0x8E:
            STX_Abs(FetchWord());
            break;
        case 0x85:
            STA_ZP(Fetch());
            break;

        //ADC
        case 0x6D:
            ADC_Abs(FetchWord());
            break;

        //BEQ
        case 0xF0:
            BEQ(Fetch());
            break;

        //BNE
        case 0xD0:
            BNE(Fetch());
            break;
        
        //INX
        case 0xE8:
            INX();
            break;

        //BRK
        case 0x00:
            BRK();
            break;

        //AND
        case 0x29:
            AND_Imm(Fetch());
            break;

        //JSR
        case 0x20:
            JSR(FetchWord());
            break;

        //RTS
        case 0x60:
            RTS();
            break;

        //NOP
        case 0xEA:
            NOP();
            break;

        //JMP
        case 0x4C:
            JMP_Abs(FetchWord());
            Utils::Logger::Info("Value of X - ", int(X));
            break;

        //DEFAULT
        default:
            Utils::Logger::Error("OpCode Not Implemented - ", Utils::Logger::Uint8ToHex(opcode));
            break;
        }
    }

    void CPU::Run()
    {
        Reset();

        Utils::Logger::Info("Launching 6502 CPU...");

        while (true)
        {
            Execute();

            PrintState();

            //Ask for key
            int key = _getch();
        }
    }

    void CPU::LoadProgram(const uint8_t* program, uint16_t programSize, uint16_t loadAddress)
    {
        for (uint16_t i = 0; i < programSize; ++i)
        {
            memory[loadAddress + i] = program[i];
        }

        // Set the reset vector to point to the program's start address
        WriteWord(0xFFFC, loadAddress);
    }

    #pragma region Single OpCodes

    void CPU::NOP()
    {
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
    }

    void CPU::JSR(uint16_t address)
    {
        //Push the return address to the stack, -1 to get the return address.
        PushStackWord(PC - 1);

        Utils::Logger::Info("Pushing PC onto stack - ", Utils::Logger::Uint16ToHex(PC - 1));

        //Preform normal JMP
        JMP_Abs(address);
    }

    void CPU::INX()
    {
        X += 1;

        SetZeroFlag(X == 0); // Set Zero flag if X is 0
        SetNegativeFlag(X & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set
    }

    void CPU::BEQ(uint8_t offset)
    {
        if (P & ZERO_FLAG)
        {
            PC += static_cast<int8_t>(offset);
        }
    }

    void CPU::BNE(uint8_t offset)
    {
        if ((P & ZERO_FLAG) == false)
        {
            PC += static_cast<int8_t>(offset);
        }
    }

    void CPU::RTS()
    {
        //Get the return addy from stack
        uint16_t ret = PullStackWord();

        PC = ret + 1;
    }

    #pragma endregion

    #pragma region ADC

    void CPU::ADC_Abs(uint16_t address)
    {
        //Adds a value from a specified memory location to the accumulator (A), along with the carry bit.
        uint8_t value = Read(address);

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
    }

    #pragma endregion

    #pragma region LDX

    void CPU::LDX_Imm(uint8_t value)
    {
        //Loads a value directly into X
        X = value;

        SetZeroFlag(X == 0); // Set Zero flag if X is 0
        SetNegativeFlag(X & NEGATIVE_FLAG); // Set Negative flag if bit 7 of X is set
    }

    #pragma endregion

    #pragma region LDA

    void CPU::LDA_Imm(uint8_t value)
    {
        //Loads a value directly into Accumulator (A)
        A = value;

        SetZeroFlag(A == 0); // Set Zero flag if A is 0
        SetNegativeFlag(A & NEGATIVE_FLAG); // Set Negative flag if bit 7 of A is set
    }

    void CPU::LDA_Abs(uint16_t address)
    {
        //Get value from memory.
        A = Read(address);

        SetZeroFlag(A == 0); // Set Zero flag if A is 0
        SetNegativeFlag(A & NEGATIVE_FLAG); // Set Negative flag if bit 7 of A is set
    }

    #pragma endregion

    #pragma region STX

    void CPU::STX_Abs(uint16_t address)
    {
        //Loads the value of the X register into memory at the address.
        Write(address, X);
    }

    #pragma endregion
  
    #pragma region STA

    void CPU::STA_ZP(uint8_t address)
    {
        //Zero page refers to the first 256 bytes of memory.

        //Loads the value of the A register into memory at the address.
        Write(address, A);
    }

    #pragma endregion

    #pragma region AND

    void CPU::AND_Imm(uint8_t value)
    {
        A = A & value; // Perform bitwise AND with the immediate value

        SetZeroFlag(A == 0);  // Set the Zero flag if the result is 0
        SetNegativeFlag(A & NEGATIVE_FLAG);  // Set the Negative flag if bit 7 is set
    }

    #pragma endregion

    #pragma region JMP

    void CPU::JMP_Abs(uint16_t address)
    {
        // JMP to address.
        PC = address;
    }

    #pragma endregion
  
    void CPU::PushStack(uint8_t value) {
        Write(StackStart + SP, value);  // Store the value at the current stack pointer address
        SP--;  // Decrement the stack pointer
    }

    void CPU::PushStackWord(uint16_t value) {
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
        return memory[addr];
    }

    // Read a word (16-bit) from memory
    uint16_t CPU::ReadWord(uint16_t addr)
    {
        return memory[addr] | (memory[addr + 1] << 8);
    }

    // Write a byte to memory
    void CPU::Write(uint16_t addr, uint8_t value) 
    {
        memory[addr] = value;
    }

    // Write a word to memory
    void CPU::WriteWord(uint16_t addr, uint16_t value) 
    {
        // Write the low byte to the specified address
        Write(addr, LOBYTE(value));

        // Write the high byte to the next address
        Write(addr + 1, HIBYTE(value));
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
        std::cout << "------------- CPU State -------------" << std::endl;
        std::cout << "A: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(A) << std::endl;
        std::cout << "X: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(X) << std::endl;
        std::cout << "Y: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(Y) << std::endl;
        std::cout << "SP: 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(SP) << std::endl;
        std::cout << "PC: 0x" << std::hex << std::setw(4) << std::setfill('0') << PC << std::endl;

        std::cout << "Status Flags (P): 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(P) << std::endl;
        std::cout << "  N V - B D I Z C" << std::endl;
        std::cout << "  " << (P & NEGATIVE_FLAG ? '1' : '0')
            << " " << (P & OVERFLOW_FLAG ? '1' : '0')
            << " " << ((P & UNUSED_FLAG) ? '1' : '0')  // Unused bit
            << " " << (P & BREAK_FLAG ? '1' : '0')
            << " " << (P & DECIMAL_FLAG ? '1' : '0')
            << " " << (P & INTERRUPT_FLAG ? '1' : '0')
            << " " << (P & ZERO_FLAG ? '1' : '0')
            << " " << (P & CARRY_FLAG ? '1' : '0') << std::endl;

        std::cout << "Cycles: " << std::dec << cycles << std::endl;

        // Display the top 4 bytes of the stack
        std::cout << "Stack (top 4 bytes):" << std::endl;

        for (int i = 0; i < 4; ++i) {
            uint8_t stackValue = Read(StackStart + (SP + 1 + i));  // Corrected stack indexing

            std::cout << "  [0x" << std::hex << std::setw(4) << std::setfill('0') << (StackStart + (SP + 1 + i))
                << "] = 0x" << std::setw(2) << static_cast<int>(stackValue) << std::endl;
        }


        std::cout << "-------------------------------------" << std::endl;
    }
}