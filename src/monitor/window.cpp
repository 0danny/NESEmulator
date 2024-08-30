#include "window.h"
#include "qapplication.h"

namespace Monitor
{
	Window::Window() : app(nullptr), mainWindow(nullptr) { }

	void Window::Create(int argc, char* argv[])
	{
		Utils::Logger::Info("Creating monitor window...");

		app = new QApplication(argc, argv);

		mainWindow = new QMainWindow();
		mainWindow->setWindowTitle("NES Emulator - CPU View");
		mainWindow->resize(WND_WIDTH, WND_HEIGHT);
	}

    void Window::AddControls(const std::vector<uint8_t>& prgRom)
    {
        QListWidget* listBox = new QListWidget(mainWindow);

        listBox->setGeometry(0, 0, WND_WIDTH, WND_HEIGHT);

        QFont font = listBox->font();
        font.setPointSize(14);
        listBox->setFont(font);

        mainWindow->setCentralWidget(listBox);

        Utils::Logger::Info("[Monitor]: PRG-ROM size - ", prgRom.size());

        // Get the reset vector (last two bytes of PRG-ROM)
        uint16_t resetVectorAddr = prgRom.size() - 4;
        uint16_t resetVector = prgRom[resetVectorAddr] | (prgRom[resetVectorAddr + 1] << 8);

        size_t i = resetVector - 0x8000;
        while (i < prgRom.size())
        {
            uint16_t address = i + 0x8000;  // Add 0x8000 to get the actual NES address
            uint8_t opcode = prgRom[i];
            auto it = opcodeLookup.find(opcode);

            if (it != opcodeLookup.end())
            {
                std::stringstream ss;
                ss << Utils::Logger::Uint16ToHex(address) << " | ";
                ss << it->second.name;

                if (it->second.operandSize == 1)
                {
                    ss << " " << Utils::Logger::Uint8ToHex(prgRom[i + it->second.operandSize]);
                }

                if (it->second.operandSize == 2)
                {
                    ss << " " << Utils::Logger::Uint16ToHex(prgRom[i + (it->second.operandSize - 1)] | prgRom[i + it->second.operandSize] << 8);
                }
               
                listBox->addItem(QString::fromStdString(ss.str()));

                // Move to the next opcode
                i += (1 + it->second.operandSize);
            }
            else
            {
                Utils::Logger::Info("[Monitor]: Breaking at unknown opcode - ", Utils::Logger::Uint8ToHex(opcode));
                break;
            }
        }
    }

	void Window::Run()
	{
		//mainWindow->show();

		if (app)
		{
			app->exec();
		
			// Delete the ptrs
			delete mainWindow;
			delete app;

			// Reset the ptrs
			mainWindow = nullptr;
			app = nullptr;
		}
	}
}