#include "emulator.h"
#include <windows.h>

int main(int argc, char* argv[])
{
	SetConsoleTitle(L"NESEmulator");

	auto& emulator = Core::Emulator::Instance();

	return emulator.Start(argc, argv);
}