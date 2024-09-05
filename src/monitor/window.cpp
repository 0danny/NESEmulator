#include "window.h"

namespace Monitor
{
    Window::Window() :
        app(nullptr),
        mainWindow(nullptr)
    { }

	void Window::Create(int argc, char* argv[])
	{
		Utils::Logger::Info("Creating monitor window...");

		app = new QApplication(argc, argv);

		mainWindow = new QMainWindow();
		mainWindow->setWindowTitle("NES Emulator - CPU View");
		mainWindow->resize(WND_WIDTH, WND_HEIGHT);

        AddControls();
        Run();
	}

    void Window::AddControls()
    {
        QWidget* centralWidget = new QWidget(mainWindow);
        QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);

        QSplitter* topSplitter = new QSplitter(Qt::Horizontal);

        // Disassembler panel
        auto& disam = Disassembler::Instance();        

        topSplitter->addWidget(disam.Create());

        // Register panel
        QListWidget* registerPanel = new QListWidget();
        registerPanel->addItem("Register Status...");
        topSplitter->addWidget(registerPanel);

        topSplitter->setStretchFactor(0, 3);
        topSplitter->setStretchFactor(1, 1);

        QSplitter* bottomSplitter = new QSplitter(Qt::Horizontal);

        // Memory dump panel
        QListWidget* memoryDumpPanel = new QListWidget();
        memoryDumpPanel->addItem("Memory Dump...");
        bottomSplitter->addWidget(memoryDumpPanel);

        // Stack panel
        QListWidget* stackPanel = new QListWidget();
        stackPanel->addItem("Stack View...");
        bottomSplitter->addWidget(stackPanel);

        bottomSplitter->setStretchFactor(0, 3);
        bottomSplitter->setStretchFactor(1, 2); // Less space to stack view

        mainLayout->addWidget(topSplitter);
        mainLayout->addWidget(bottomSplitter);

        mainLayout->setStretch(0, 3);
        mainLayout->setStretch(1, 2);

        centralWidget->setLayout(mainLayout);
        mainWindow->setCentralWidget(centralWidget);
    }

	void Window::Run()
	{
		mainWindow->show();

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