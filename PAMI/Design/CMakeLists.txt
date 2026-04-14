# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Pico VS Code extension (NE PAS MODIFIER)
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# Board
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pico SDK
include(pico_sdk_import.cmake)

# ===== PROJECT =====
project(double_encoder_project C CXX ASM)

pico_sdk_init()

# ===== EXECUTABLE =====
add_executable(double_encoder_project
    main.c
    motors.c
    encoders.c
    ultrasound.c
)

# ===== PIO =====
pico_generate_pio_header(double_encoder_project
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

# ===== LIBS =====
target_link_libraries(double_encoder_project
    pico_stdlib
    hardware_pio
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_irq
)

# ===== STDIO =====
pico_enable_stdio_usb(double_encoder_project 1)
pico_enable_stdio_uart(double_encoder_project 1)

# ===== OUTPUTS =====
pico_add_extra_outputs(double_encoder_project)
