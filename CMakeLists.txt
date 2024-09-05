cmake_minimum_required (VERSION 3.8)

project ("NESEmulator")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Manually set the paths to SDL2
set(SDL2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/externals/SDL2/include")
set(SDL2_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/externals/SDL2/lib/x64")

set(QT_DIR "C:/Qt/6.7.2/msvc2019_64")
set(CMAKE_PREFIX_PATH ${QT_DIR} ${CMAKE_PREFIX_PATH})

include_directories(${SDL2_INCLUDE_DIR})
link_directories(${SDL2_LIBRARY_DIR})

find_package(Qt6 COMPONENTS Widgets REQUIRED)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()


add_executable (${PROJECT_NAME} "src/main.cpp"   "src/emulator.cpp" "src/emulator.h" "src/utils/logger.h" "src/emulation/cpu.h" "src/emulation/cpu.cpp"    "src/utils/logger.cpp" "src/emulation/romreader.cpp" "src/emulation/romreader.h" "src/emulation/graphics/renderer.cpp" "src/emulation/graphics/renderer.h" "src/monitor/window.cpp" "src/monitor/window.h" "src/emulation/graphics/ppu.cpp" "src/emulation/graphics/ppu.h" "src/emulation/memorybus.cpp" "src/emulation/memorybus.h" "src/utils/excepthandler.cpp" "src/utils/excepthandler.h" "src/emulation/testing/cputest.cpp" "src/emulation/testing/cputest.h" "src/emulation/controller.cpp" "src/emulation/controller.h" "src/monitor/disassembler.cpp" "src/monitor/disassembler.h")

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
endif()

# Link SDL2 libraries
target_link_libraries(${PROJECT_NAME} SDL2 SDL2main)
target_link_libraries(${PROJECT_NAME} Qt6::Widgets)