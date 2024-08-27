#pragma once

#include "logger.h"
#include <string>

namespace Utils
{
	class ExceptHandler
	{
	private:
		ExceptHandler();

		bool hasException = false;

	public:
		// Singleton access
		static ExceptHandler& Instance()
		{
			static ExceptHandler INSTANCE;
			return INSTANCE;
		}

		// Prevent copying and assignment
		ExceptHandler(const ExceptHandler&) = delete;
		ExceptHandler& operator=(const ExceptHandler&) = delete;

		bool HasException() const
		{
			return hasException;
		}

		void ThrowException(std::string reason, std::string error);
	};
}

