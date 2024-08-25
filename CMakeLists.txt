cmake_minimum_required (VERSION 3.8)

# Manually set the paths to SDL2
set(SDL2_INCLUDE_DIR "C:/SDKs/SDL2/include")
set(SDL2_LIBRARY_DIR "C:/SDKs/SDL2/lib/x64")

include_directories(${SDL2_INCLUDE_DIR})
link_directories(${SDL2_LIBRARY_DIR})

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("NESEmulator")

add_executable (NESEmulator "src/main.cpp"   "src/emulator.cpp" "src/emulator.h" "src/utils/logger.h" "src/emulation/cpu.h" "src/emulation/cpu.cpp"    "src/utils/logger.cpp" "src/emulation/romreader.cpp" "src/emulation/romreader.h" "src/emulation/graphics/renderer.cpp" "src/emulation/graphics/renderer.h")

target_include_directories(NESEmulator PRIVATE ${CMAKE_SOURCE_DIR}/src)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET NESEmulator PROPERTY CXX_STANDARD 20)
endif()

# Link SDL2 libraries
target_link_libraries(NESEmulator SDL2 SDL2main)