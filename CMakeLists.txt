cmake_minimum_required(VERSION 3.16)

project(MachineCoordinator LANGUAGES CXX)

include(CTest)

add_library(machine_coordinator
    src/MachineCoordinator.cpp
)
add_library(MachineCoordinator::machine_coordinator ALIAS machine_coordinator)

target_include_directories(machine_coordinator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(machine_coordinator PUBLIC cxx_std_17)

option(BUILD_COORDINATED_EXAMPLE "Build the hardware-connected dispense application" OFF)
if(BUILD_COORDINATED_EXAMPLE)
    set(ARDUINO_CONTROLLER_SOURCE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../ArduinoController"
        CACHE PATH "Path to the ArduinoController project")
    set(MARLIN_CONTROLLER_SOURCE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/MarlinController"
        CACHE PATH "Path to the MarlinController project")
    set(EASY_MACHINE_LANGUAGE_SOURCE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../EasyMachineLanguage"
        CACHE PATH "Path to the EasyMachineLanguage project")
    if(NOT EXISTS "${ARDUINO_CONTROLLER_SOURCE_DIR}/include/GpioController.hpp")
        message(FATAL_ERROR "ArduinoController was not found at '${ARDUINO_CONTROLLER_SOURCE_DIR}'")
    endif()
    if(NOT EXISTS "${MARLIN_CONTROLLER_SOURCE_DIR}/include/MotionController.hpp")
        message(FATAL_ERROR "MarlinController was not found at '${MARLIN_CONTROLLER_SOURCE_DIR}'")
    endif()
    if(NOT EXISTS "${EASY_MACHINE_LANGUAGE_SOURCE_DIR}/include/EasyMachineLanguage.hpp")
        message(FATAL_ERROR
            "EasyMachineLanguage was not found at '${EASY_MACHINE_LANGUAGE_SOURCE_DIR}'")
    endif()
    add_subdirectory("${ARDUINO_CONTROLLER_SOURCE_DIR}"
                     "${CMAKE_CURRENT_BINARY_DIR}/arduino-controller")
    add_subdirectory("${MARLIN_CONTROLLER_SOURCE_DIR}"
                     "${CMAKE_CURRENT_BINARY_DIR}/marlin-controller")
    add_subdirectory("${EASY_MACHINE_LANGUAGE_SOURCE_DIR}"
                     "${CMAKE_CURRENT_BINARY_DIR}/easy-machine-language")

    add_executable(machine_coordinator_run examples/RunEmlProgram.cpp)
    target_link_libraries(machine_coordinator_run PRIVATE
        MachineCoordinator::machine_coordinator
        ArduinoController::arduino_controller
        marlincontroller
        EasyMachineLanguage::easy_machine_language
    )

    set(ARDUINO_CLI_EXECUTABLE "arduino-cli" CACHE FILEPATH
        "Path to Arduino CLI")
    set(ARDUINO_FQBN "arduino:renesas_uno:minima" CACHE STRING
        "Arduino board FQBN for the ArduinoController firmware")
    set(ARDUINO_UPLOAD_PORT "" CACHE STRING
        "Serial port used by the explicit upload_arduino_firmware target")

    add_custom_target(arduino_firmware
        COMMAND "${ARDUINO_CLI_EXECUTABLE}" compile
                --fqbn "${ARDUINO_FQBN}"
                "${ARDUINO_CONTROLLER_SOURCE_DIR}/firmware/ArduinoController"
        COMMENT "Compiling ArduinoController firmware for ${ARDUINO_FQBN}"
        VERBATIM
    )

    if(ARDUINO_UPLOAD_PORT)
        add_custom_target(upload_arduino_firmware
            COMMAND "${ARDUINO_CLI_EXECUTABLE}" compile --upload
                    --fqbn "${ARDUINO_FQBN}"
                    --port "${ARDUINO_UPLOAD_PORT}"
                    "${ARDUINO_CONTROLLER_SOURCE_DIR}/firmware/ArduinoController"
            COMMENT "Uploading ArduinoController firmware to ${ARDUINO_UPLOAD_PORT}"
            VERBATIM
        )
    else()
        add_custom_target(upload_arduino_firmware
            COMMAND "${CMAKE_COMMAND}" -E echo
                    "Set -DARDUINO_UPLOAD_PORT=/dev/ttyACM0 to enable this explicit upload target"
            COMMAND "${CMAKE_COMMAND}" -E false
            VERBATIM
        )
    endif()
endif()

if(BUILD_TESTING)
    find_package(Threads REQUIRED)
    add_executable(machine_coordinator_tests tests/MachineCoordinatorTests.cpp)
    target_link_libraries(machine_coordinator_tests PRIVATE
        MachineCoordinator::machine_coordinator
        Threads::Threads
    )
    add_test(NAME machine_coordinator_tests COMMAND machine_coordinator_tests)
endif()

include(GNUInstallDirs)
install(TARGETS machine_coordinator
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
