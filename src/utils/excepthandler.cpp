#include "excepthandler.h"

namespace Utils
{
	ExceptHandler::ExceptHandler() { }

	void ExceptHandler::ThrowException(std::string reason, std::string error)
	{
		//Throw an exception when the CPU encouters an error.
		Utils::Logger::Error("--------------- Encountered an Exception! ---------------");
		Utils::Logger::Error(reason + " - ", error);
		hasException = true;
	}
}

