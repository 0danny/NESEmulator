#include "emulator.h"
#include <windows.h>

int main()
{
	SetConsoleTitle("NESEmulator");

	auto& emulator = Core::Emulator::Instance();

	return emulator.Start();
}