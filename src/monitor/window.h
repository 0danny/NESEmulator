#pragma once

#include "utils/logger.h"
#include "disassembler.h"
#include "emulation/memorybus.h"

#include <string>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QSplitter>

namespace Monitor
{
	class Window
	{
	public:
		Window();

		void Create(int argc, char* argv[]);

		static Window& Instance()
		{
			static Window INSTANCE;
			return INSTANCE;
		}

	private:
		const int WND_WIDTH = 1400;
		const int WND_HEIGHT = 800;

		void Run();
		void AddControls();

		QApplication* app;
		QMainWindow* mainWindow;
	};
}