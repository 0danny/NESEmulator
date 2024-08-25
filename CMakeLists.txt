# CMakeList.txt : CMake project for NESEmulator, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("NESEmulator")

# Add source to this project's executable.
add_executable (NESEmulator "src/main.cpp"   "src/emulator.cpp" "src/emulator.h" "src/utils/logger.h" "src/emulation/cpu.h" "src/emulation/cpu.cpp"    "src/utils/logger.cpp" "src/emulation/romreader.cpp" "src/emulation/romreader.h")

target_include_directories(NESEmulator PRIVATE ${CMAKE_SOURCE_DIR}/src)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET NESEmulator PROPERTY CXX_STANDARD 20)
endif()