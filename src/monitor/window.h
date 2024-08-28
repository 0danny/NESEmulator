#pragma once

#include "utils/logger.h"
#include "emulation/cpu.h"

#include <unordered_map>
#include <string>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QListWidget>

namespace Monitor
{
	struct InstructionInfo {
		std::string name;  // Instruction name
		uint8_t operandSize; // Operand size in bytes (excluding the opcode itself)
	};

	class Window
	{
	public:
		Window();

		void Create(int argc, char* argv[]);
		void Run();
		void AddControls(const std::vector<uint8_t>& prgRom);

		std::unordered_map<uint8_t, InstructionInfo> opcodeLookup = {
			{0x00, {"BRK", 0}},
			{0x01, {"ORA", 1}},
			{0x05, {"ORA", 1}},
			{0x06, {"ASL", 1}},
			{0x08, {"PHP", 0}},
			{0x09, {"ORA", 1}},
			{0x0A, {"ASL", 0}},
			{0x0D, {"ORA", 2}},
			{0x0E, {"ASL", 2}},
			{0x10, {"BPL", 1}},
			{0x11, {"ORA", 1}},
			{0x15, {"ORA", 1}},
			{0x16, {"ASL", 1}},
			{0x18, {"CLC", 0}},
			{0x19, {"ORA", 2}},
			{0x1D, {"ORA", 2}},
			{0x1E, {"ASL", 2}},
			{0x20, {"JSR", 2}},
			{0x21, {"AND", 1}},
			{0x24, {"BIT", 1}},
			{0x25, {"AND", 1}},
			{0x26, {"ROL", 1}},
			{0x28, {"PLP", 0}},
			{0x29, {"AND", 1}},
			{0x2A, {"ROL", 0}},
			{0x2C, {"BIT", 2}},
			{0x2D, {"AND", 2}},
			{0x2E, {"ROL", 2}},
			{0x30, {"BMI", 1}},
			{0x31, {"AND", 1}},
			{0x35, {"AND", 1}},
			{0x36, {"ROL", 1}},
			{0x38, {"SEC", 0}},
			{0x39, {"AND", 2}},
			{0x3D, {"AND", 2}},
			{0x3E, {"ROL", 2}},
			{0x40, {"RTI", 0}},
			{0x41, {"EOR", 1}},
			{0x45, {"EOR", 1}},
			{0x46, {"LSR", 1}},
			{0x48, {"PHA", 0}},
			{0x49, {"EOR", 1}},
			{0x4A, {"LSR", 0}},
			{0x4C, {"JMP", 2}},
			{0x4D, {"EOR", 2}},
			{0x4E, {"LSR", 2}},
			{0x50, {"BVC", 1}},
			{0x51, {"EOR", 1}},
			{0x55, {"EOR", 1}},
			{0x56, {"LSR", 1}},
			{0x58, {"CLI", 0}},
			{0x59, {"EOR", 2}},
			{0x5D, {"EOR", 2}},
			{0x5E, {"LSR", 2}},
			{0x60, {"RTS", 0}},
			{0x61, {"ADC", 1}},
			{0x65, {"ADC", 1}},
			{0x66, {"ROR", 1}},
			{0x68, {"PLA", 0}},
			{0x69, {"ADC", 1}},
			{0x6A, {"ROR", 0}},
			{0x6C, {"JMP", 2}},
			{0x6D, {"ADC", 2}},
			{0x6E, {"ROR", 2}},
			{0x70, {"BVS", 1}},
			{0x71, {"ADC", 1}},
			{0x75, {"ADC", 1}},
			{0x76, {"ROR", 1}},
			{0x78, {"SEI", 0}},
			{0x79, {"ADC", 2}},
			{0x7D, {"ADC", 2}},
			{0x7E, {"ROR", 2}},
			{0x81, {"STA", 1}},
			{0x84, {"STY", 1}},
			{0x85, {"STA", 1}},
			{0x86, {"STX", 1}},
			{0x88, {"DEY", 0}},
			{0x8A, {"TXA", 0}},
			{0x8C, {"STY", 2}},
			{0x8D, {"STA", 2}},
			{0x8E, {"STX", 2}},
			{0x90, {"BCC", 1}},
			{0x91, {"STA", 1}},
			{0x94, {"STY", 1}},
			{0x95, {"STA", 1}},
			{0x96, {"STX", 1}},
			{0x98, {"TYA", 0}},
			{0x99, {"STA", 2}},
			{0x9A, {"TXS", 0}},
			{0x9D, {"STA", 2}},
			{0xA0, {"LDY", 1}},
			{0xA1, {"LDA", 1}},
			{0xA2, {"LDX", 1}},
			{0xA4, {"LDY", 1}},
			{0xA5, {"LDA", 1}},
			{0xA6, {"LDX", 1}},
			{0xA8, {"TAY", 0}},
			{0xA9, {"LDA", 1}},
			{0xAA, {"TAX", 0}},
			{0xAC, {"LDY", 2}},
			{0xAD, {"LDA", 2}},
			{0xAE, {"LDX", 2}},
			{0xB0, {"BCS", 1}},
			{0xB1, {"LDA", 1}},
			{0xB4, {"LDY", 1}},
			{0xB5, {"LDA", 1}},
			{0xB6, {"LDX", 1}},
			{0xB8, {"CLV", 0}},
			{0xB9, {"LDA", 2}},
			{0xBA, {"TSX", 0}},
			{0xBC, {"LDY", 2}},
			{0xBD, {"LDA", 2}},
			{0xBE, {"LDX", 2}},
			{0xC0, {"CPY", 1}},
			{0xC1, {"CMP", 1}},
			{0xC4, {"CPY", 1}},
			{0xC5, {"CMP", 1}},
			{0xC6, {"DEC", 1}},
			{0xC8, {"INY", 0}},
			{0xC9, {"CMP", 1}},
			{0xCA, {"DEX", 0}},
			{0xCC, {"CPY", 2}},
			{0xCD, {"CMP", 2}},
			{0xCE, {"DEC", 2}},
			{0xD0, {"BNE", 1}},
			{0xD1, {"CMP", 1}},
			{0xD5, {"CMP", 1}},
			{0xD6, {"DEC", 1}},
			{0xD8, {"CLD", 0}},
			{0xD9, {"CMP", 2}},
			{0xDD, {"CMP", 2}},
			{0xDE, {"DEC", 2}},
			{0xE0, {"CPX", 1}},
			{0xE1, {"SBC", 1}},
			{0xE4, {"CPX", 1}},
			{0xE5, {"SBC", 1}},
			{0xE6, {"INC", 1}},
			{0xE8, {"INX", 0}},
			{0xE9, {"SBC", 1}},
			{0xEA, {"NOP", 0}},
			{0xEC, {"CPX", 2}},
			{0xED, {"SBC", 2}},
			{0xEE, {"INC", 2}},
			{0xF0, {"BEQ", 1}},
			{0xF1, {"SBC", 1}},
			{0xF5, {"SBC", 1}},
			{0xF6, {"INC", 1}},
			{0xF8, {"SED", 0}},
			{0xF9, {"SBC", 2}},
			{0xFD, {"SBC", 2}},
			{0xFE, {"INC", 2}},
		};

		static Window& Instance()
		{
			static Window INSTANCE;
			return INSTANCE;
		}

	private:
		const int WND_WIDTH = 600;
		const int WND_HEIGHT = 500;

		QApplication* app;
		QMainWindow* mainWindow;
	};
}