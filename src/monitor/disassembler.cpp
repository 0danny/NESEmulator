#include "disassembler.h"

namespace Monitor
{
	Disassembler::Disassembler() :
		cpu(Emulation::CPU::Instance()),
		memoryBus(Emulation::MemoryBus::Instance()),
		disassemblerPanel(nullptr)
	{
		Utils::Logger::Info("Disassembler created...");

		cpu.RegisterPCCallback([this]()
		{
			return this->PCCallback();
		});
	}

	void Disassembler::PCCallback()
	{
		if (disassemblerPanel != nullptr)
		{
			
		}
	}

	QTableWidget* Disassembler::Create()
	{
		disassemblerPanel = new QTableWidget();
		disassemblerPanel->setColumnCount(3);
		disassemblerPanel->setHorizontalHeaderLabels({ "Address", "Opcodes", "Instruction" });

		disassemblerPanel->horizontalHeader()->setStretchLastSection(true);
		disassemblerPanel->verticalHeader()->setVisible(true);
		disassemblerPanel->setSelectionBehavior(QAbstractItemView::SelectRows);
		disassemblerPanel->setSelectionMode(QAbstractItemView::SingleSelection);

		disassemblerPanel->setUpdatesEnabled(false);

		// Add instructions
		size_t i = 0x8000;
		size_t endAddress = 0xFFFA;
		int row = 0;

		//Disassemble everythingggg
		while (i < memoryBus.ram.size() && i <= endAddress)
		{
			uint8_t opcode = memoryBus.ram[i];
			auto it = opcodeLookup.find(opcode);

			if (it != opcodeLookup.end())
			{
				disassemblerPanel->insertRow(row);

				// Address column
				QTableWidgetItem* addressItem = new QTableWidgetItem(QString::fromStdString(Utils::Logger::Uint16ToHexUppercase(i)));
				addressItem->setFlags(addressItem->flags() & ~Qt::ItemIsEditable);
				disassemblerPanel->setItem(row, 0, addressItem);

				// Opcodes column
				std::stringstream opcodesSS;
				opcodesSS << Utils::Logger::Uint8ToHexUppercase(opcode);

				for (int j = 1; j <= it->second.operandSize; ++j)
				{
					opcodesSS << " " << Utils::Logger::Uint8ToHexUppercase(memoryBus.ram[i + j]);
				}

				QTableWidgetItem* opcodesItem = new QTableWidgetItem(QString::fromStdString(opcodesSS.str()));
				opcodesItem->setFlags(opcodesItem->flags() & ~Qt::ItemIsEditable);
				disassemblerPanel->setItem(row, 1, opcodesItem);

				std::stringstream instructionSS;
				instructionSS << it->second.name;

				if (it->second.operandSize == 1)
				{
					instructionSS << " " << Utils::Logger::Uint8ToHexUppercase(memoryBus.ram[i + 1]);
				}
				else if (it->second.operandSize == 2)
				{
					instructionSS << " " << Utils::Logger::Uint16ToHexUppercase(memoryBus.ram[i + 1] | (memoryBus.ram[i + 2] << 8));
				}

				QTableWidgetItem* instructionItem = new QTableWidgetItem(QString::fromStdString(instructionSS.str()));
				instructionItem->setFlags(instructionItem->flags() & ~Qt::ItemIsEditable);
				disassemblerPanel->setItem(row, 2, instructionItem);

				i += (1 + it->second.operandSize);
				row++;
			}
			else
			{
				Utils::Logger::Info("[Monitor]: Breaking at unknown opcode - ", Utils::Logger::Uint8ToHex(opcode));
				break;
			}
		}

		disassemblerPanel->setUpdatesEnabled(true);

		return disassemblerPanel;
	}
}
