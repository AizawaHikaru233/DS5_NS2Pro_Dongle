# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if (WIN32)
    set(USERHOME $ENV{USERPROFILE})
else ()
    set(USERHOME $ENV{HOME})
endif ()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif ()
# ====================================================================================

option(PICO_W_BUILD "Build for Rasberry Pi Pico W, audio is missing" OFF)
option(WAVESHARE_RP2350B_PLUS_W_BUILD "Build for Waveshare RP2350B-Plus-W (USB-C, 16MB flash, RM2 wireless)" OFF)

if (PICO_W_BUILD AND WAVESHARE_RP2350B_PLUS_W_BUILD)
        message(FATAL_ERROR "PICO_W_BUILD and WAVESHARE_RP2350B_PLUS_W_BUILD are mutually exclusive")
endif ()

if (PICO_W_BUILD)
        set(PICO_BOARD pico_w CACHE STRING "Board type")
        set(PICO_RP2350 OFF CACHE BOOL "Not using RP2350")
elseif (WAVESHARE_RP2350B_PLUS_W_BUILD)
        set(PICO_BOARD waveshare_rp2350b_plus_w CACHE STRING "Board type")
        # Waveshare board header is not in upstream pico-sdk; ship it here and
        # tell the SDK to search this directory when resolving PICO_BOARD.
        list(APPEND PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/boards/headers")
else ()
        set(PICO_BOARD pico2_w CACHE STRING "Board type")
endif ()

# Diagnostic: confirm PICO_BOARD resolution actually applied. Build log will
# show which board path was taken. Remove these once verified on hardware.
message(STATUS "DS5_NS2Pro_Dongle build target -- PICO_BOARD=${PICO_BOARD}")
message(STATUS "DS5_NS2Pro_Dongle build target -- PICO_W_BUILD=${PICO_W_BUILD}")
message(STATUS "DS5_NS2Pro_Dongle build target -- WAVESHARE_RP2350B_PLUS_W_BUILD=${WAVESHARE_RP2350B_PLUS_W_BUILD}")
message(STATUS "DS5_NS2Pro_Dongle build target -- PICO_BOARD_HEADER_DIRS=${PICO_BOARD_HEADER_DIRS}")
#set(PICO_CYW43_SUPPORTED 1)
#set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND 1)

option(ENABLE_SERIAL "Enable USB CDC serial printf output" OFF)
if (ENABLE_SERIAL)
        set(ENABLE_SERIAL_VALUE 1)
else ()
        set(ENABLE_SERIAL_VALUE 0)
endif ()
option(ENABLE_NS2PRO_SERIAL_BRIDGE "Enable Pico CDC NS2Pro serial bridge input" ON)
if (ENABLE_NS2PRO_SERIAL_BRIDGE)
        set(ENABLE_NS2PRO_SERIAL_BRIDGE_VALUE 1)
else ()
        set(ENABLE_NS2PRO_SERIAL_BRIDGE_VALUE 0)
endif ()
option(ENABLE_VERBOSE "Enable Log Verbose" OFF)
if(ENABLE_VERBOSE)
    set(ENABLE_VERBOSE_VALUE 1)
else()
    set(ENABLE_VERBOSE_VALUE 0)
endif ()
option(ENABLE_BATT_LED "Blink Pico LED when DualSense battery is low (<=10%)" ON)
if (ENABLE_BATT_LED)
        set(ENABLE_BATT_LED_VALUE 1)
else ()
        set(ENABLE_BATT_LED_VALUE 0)
endif ()
option(DISABLE_SPEAKER_PROC "Disable Speaker in controller" OFF)
if (DISABLE_SPEAKER_PROC)
        set(DISABLE_SPEAKER_PROC_VALUE 1)
else ()
        set(DISABLE_SPEAKER_PROC_VALUE 0)
endif ()
option(PICO_W_BUILD "Build for Rasberry Pi Pico W, audio is missing" OFF)
if (PICO_W_BUILD)
        set(CYW43_PIO_CLOCK_DIV_VALUE 3)
        set(SYS_CLOCK_KHZ_VALUE 200000)
        set(DISABLE_SPEAKER_PROC_VALUE 1)
else ()
        if(DISABLE_SPEAKER_PROC_VALUE)
            set(CYW43_PIO_CLOCK_DIV_VALUE 2)
            set(SYS_CLOCK_KHZ_VALUE 150000)
        else ()
            set(CYW43_PIO_CLOCK_DIV_VALUE 2)
            set(SYS_CLOCK_KHZ_VALUE 150000)
        endif ()
endif ()



# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

set(FIRMWARE_TARGET ds5_ns2pro_dongle)
set(FIRMWARE_VERSION "1.0.0" CACHE STRING "Program version string")
set(FIRMWARE_OUTPUT_NAME "${FIRMWARE_TARGET}_v${FIRMWARE_VERSION}")

project(${FIRMWARE_TARGET} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(${PROJECT_NAME}
        src/usb_descriptors.cpp
        src/main.cpp
        src/usb.cpp
        src/audio.cpp
        src/bt.cpp
        src/config.cpp
        src/cmd.cpp
        src/bridge_manager_config.cpp
        src/ds5_input_tuning.cpp
        src/pico_led.cpp
        src/state_mgr.cpp
        src/ns2pro_config.cpp
        src/ns2pro_ds5_identity.cpp
        src/ns2pro_haptics_bridge.cpp
        src/ns2pro_ble.cpp
        src/ns2pro_rumble.cpp
        src/ns2pro_translator.cpp
        src/ns2pro_serial_bridge.cpp
        src/ram_mem.c
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${FIRMWARE_OUTPUT_NAME}")

# Keep the extracted NS2Pro haptics bridge on the same RAM-resident hot path as
# audio_loop(), without pulling those implementation details back into the
# original DS5 files.
set_source_files_properties(
        src/ns2pro_haptics_bridge.cpp
        src/ns2pro_ble.cpp
        src/ns2pro_rumble.cpp
        src/ns2pro_serial_bridge.cpp
        PROPERTIES COMPILE_OPTIONS "-fno-function-sections;-fno-data-sections"
)

if (ENABLE_BATT_LED)
        target_sources(${PROJECT_NAME} PRIVATE src/battery_led.cpp)
endif ()

# Cockos WDLResampler
add_library(wdl_resampler STATIC
        lib/WDL/WDL/resample.cpp
)
target_include_directories(wdl_resampler PUBLIC
        lib/WDL/WDL
)

# Opus
#set(OPUS_FIXED_POINT ON)
add_subdirectory(lib/opus)

add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/opus_ram_relocate.stamp
    COMMAND ${CMAKE_OBJCOPY} --rename-section .text=.time_critical.opus_text --rename-section .rodata=.time_critical.opus_rodata --rename-section .rodata.str1.4=.time_critical.opus_strings $<TARGET_FILE:opus>
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_BINARY_DIR}/opus_ram_relocate.stamp
    DEPENDS $<TARGET_FILE:opus>
    COMMENT "Relocating libopus to .time_critical.* sections"
    VERBATIM
)
add_custom_target(opus_ram_relocate ALL DEPENDS ${CMAKE_BINARY_DIR}/opus_ram_relocate.stamp)
add_dependencies(${PROJECT_NAME} opus_ram_relocate)

target_compile_options(wdl_resampler PRIVATE -fsigned-char)
target_compile_options(wdl_resampler PRIVATE -ffunction-sections)
add_custom_command(TARGET wdl_resampler POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
        --rename-section .text._ZN13WDL_Resampler11ResampleOutEPfiii=.time_critical.WDL_ResampleOut
        --rename-section .text._ZN13WDL_Resampler15ResamplePrepareEiiPPf=.time_critical.WDL_ResamplePrepare
        $<TARGET_FILE:wdl_resampler>
    COMMENT "Relocating WDL_Resampler hot-path methods to .time_critical.*"
)

function(relocate_to_ram _suffix _renames _comment)
    add_custom_command(TARGET ${PROJECT_NAME} PRE_LINK
        COMMAND ${CMAKE_COMMAND}
            "-DOBJROOT=${CMAKE_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}.dir"
            "-DOBJCOPY=${CMAKE_OBJCOPY}"
            "-DSUFFIX=${_suffix}"
            "-DRENAMES=${_renames}"
            -P "${CMAKE_SOURCE_DIR}/cmake/relocate_to_ram.cmake"
        COMMENT "${_comment}"
        VERBATIM)
endfunction()

relocate_to_ram(lib/btstack/src/hci.c.o
    ".text.packet_handler=.time_critical.bt_packet_handler|.text.hci_run=.time_critical.hci_run|.text.hci_send_acl_packet_fragments=.time_critical.hci_send_acl_packet_fragments|.text.hci_send_acl_packet_buffer=.time_critical.hci_send_acl_packet_buffer"
    "Relocating hci hot paths to RAM")
relocate_to_ram(lib/btstack/src/l2cap.c.o
    ".text.l2cap_acl_handler=.time_critical.l2cap_acl_handler|.text.l2cap_run=.time_critical.l2cap_run|.text.l2cap_send=.time_critical.l2cap_send|.text.l2cap_send_prepared=.time_critical.l2cap_send_prepared|.text.l2cap_notify_channel_can_send=.time_critical.l2cap_notify_channel_can_send"
    "Relocating l2cap hot paths to RAM")
relocate_to_ram(lib/tinyusb/src/common/tusb_fifo.c.o
    ".text.tu_fifo_read=.time_critical.tu_fifo_read|.text.tu_fifo_read_n=.time_critical.tu_fifo_read_n|.text.tu_fifo_write=.time_critical.tu_fifo_write|.text.tu_fifo_write_n=.time_critical.tu_fifo_write_n|.text.tu_fifo_count=.time_critical.tu_fifo_count"
    "Relocating TinyUSB FIFO hot paths to RAM")
relocate_to_ram(src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c.o
    ".text.cyw43_spi_transfer=.time_critical.cyw43_spi_transfer|.text.cyw43_read_bytes=.time_critical.cyw43_read_bytes|.text.cyw43_write_bytes=.time_critical.cyw43_write_bytes|.text.cyw43_read_reg_u32=.time_critical.cyw43_read_reg_u32|.text.cyw43_read_reg_u16=.time_critical.cyw43_read_reg_u16|.text.cyw43_read_reg_u8=.time_critical.cyw43_read_reg_u8|.text.cyw43_write_reg_u32=.time_critical.cyw43_write_reg_u32|.text.cyw43_write_reg_u16=.time_critical.cyw43_write_reg_u16|.text.cyw43_write_reg_u8=.time_critical.cyw43_write_reg_u8|.text.read_reg_u32_swap=.time_critical.read_reg_u32_swap|.text.write_reg_u32_swap=.time_critical.write_reg_u32_swap|.text.ns_delay.constprop.0=.time_critical.cyw43_ns_delay"
    "Relocating cyw43 PIO-SPI hot paths to RAM")
relocate_to_ram(lib/cyw43-driver/src/cyw43_ll.c.o
    ".text.cyw43_ll_sdpcm_poll_device=.time_critical.cyw43_ll_sdpcm_poll_device|.text.cyw43_ll_process_packets=.time_critical.cyw43_ll_process_packets|.text.cyw43_sdpcm_send_common=.time_critical.cyw43_sdpcm_send_common|.text.cyw43_do_ioctl.part.0=.time_critical.cyw43_do_ioctl|.text.cyw43_ll_bt_has_work=.time_critical.cyw43_ll_bt_has_work|.text.cyw43_ll_parse_async_event=.time_critical.cyw43_ll_parse_async_event|.text.cyw43_ll_write_backplane_mem=.time_critical.cyw43_ll_write_backplane_mem|.text.cyw43_ll_read_backplane_mem=.time_critical.cyw43_ll_read_backplane_mem|.text.cyw43_write_backplane=.time_critical.cyw43_write_backplane|.text.cyw43_read_backplane.constprop.0=.time_critical.cyw43_read_backplane"
    "Relocating cyw43 SDPCM hot paths to RAM")
relocate_to_ram(lib/cyw43-driver/src/cyw43_ctrl.c.o
    ".text.cyw43_poll_func=.time_critical.cyw43_poll_func|.text.cyw43_bluetooth_hci_read=.time_critical.cyw43_bluetooth_hci_read|.text.cyw43_bluetooth_hci_write=.time_critical.cyw43_bluetooth_hci_write"
    "Relocating cyw43 control hot paths to RAM")
relocate_to_ram(lib/tinyusb/src/device/usbd.c.o
    ".text.tud_task_ext=.time_critical.tud_task_ext|.text.usbd_edpt_xfer=.time_critical.usbd_edpt_xfer|.text.usbd_edpt_xfer_fifo=.time_critical.usbd_edpt_xfer_fifo"
    "Relocating TinyUSB device hot paths to RAM")
relocate_to_ram(lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.o
    ".text.dcd_edpt_xfer=.time_critical.dcd_edpt_xfer|.text.dcd_edpt_iso_activate=.time_critical.dcd_edpt_iso_activate"
    "Relocating RP2040 DCD hot paths to RAM")
relocate_to_ram(lib/tinyusb/src/class/audio/audio_device.c.o
    ".text.tud_audio_n_read=.time_critical.tud_audio_n_read|.text.tud_audio_n_write=.time_critical.tud_audio_n_write|.text.audiod_calc_tx_packet_sz.isra.0=.time_critical.audiod_calc_tx_packet_sz|.text.audiod_xfer_isr=.time_critical.audiod_xfer_isr"
    "Relocating TinyUSB audio hot paths to RAM")
relocate_to_ram(lib/tinyusb/src/class/hid/hid_device.c.o
    ".text.tud_hid_n_report=.time_critical.tud_hid_n_report|.text.tud_hid_n_ready=.time_critical.tud_hid_n_ready|.text.hidd_xfer_cb=.time_critical.hidd_xfer_cb"
    "Relocating TinyUSB HID hot paths to RAM")
relocate_to_ram(src/common/pico_util/queue.c.o
    ".text.queue_try_add=.time_critical.queue_try_add|.text.queue_try_remove=.time_critical.queue_try_remove"
    "Relocating pico queue hot paths to RAM")
relocate_to_ram(src/rp2_common/pico_cyw43_arch/cyw43_arch.c.o
    ".text.cyw43_arch_poll=.time_critical.cyw43_arch_poll"
    "Relocating cyw43_arch_poll to RAM")
relocate_to_ram(lib/btstack/src/btstack_run_loop.c.o
    ".text.btstack_run_loop_base_poll_data_sources=.time_critical.btstack_run_loop_base_poll_data_sources|.text.btstack_run_loop_poll_data_sources_from_irq=.time_critical.btstack_run_loop_poll_data_sources_from_irq"
    "Relocating btstack run-loop hot paths to RAM")
relocate_to_ram(src/ns2pro_haptics_bridge.cpp.o
    ".text=.time_critical.ns2pro_haptics_bridge_text|.rodata=.time_critical.ns2pro_haptics_bridge_rodata|.rodata.cst4=.time_critical.ns2pro_haptics_bridge_cst4|.rodata.cst8=.time_critical.ns2pro_haptics_bridge_cst8|.rodata.str1.1=.time_critical.ns2pro_haptics_bridge_strings1|.rodata.str1.4=.time_critical.ns2pro_haptics_bridge_strings4"
    "Relocating NS2Pro haptics bridge to RAM")
relocate_to_ram(src/ns2pro_ble.cpp.o
    ".text=.time_critical.ns2pro_ble_text|.rodata=.time_critical.ns2pro_ble_rodata|.rodata.cst4=.time_critical.ns2pro_ble_cst4|.rodata.cst8=.time_critical.ns2pro_ble_cst8|.rodata.str1.1=.time_critical.ns2pro_ble_strings1|.rodata.str1.4=.time_critical.ns2pro_ble_strings4"
    "Relocating NS2Pro BLE path to RAM")
relocate_to_ram(src/ns2pro_rumble.cpp.o
    ".text=.time_critical.ns2pro_rumble_text|.rodata=.time_critical.ns2pro_rumble_rodata|.rodata.cst4=.time_critical.ns2pro_rumble_cst4|.rodata.cst8=.time_critical.ns2pro_rumble_cst8|.rodata.str1.1=.time_critical.ns2pro_rumble_strings1|.rodata.str1.4=.time_critical.ns2pro_rumble_strings4"
    "Relocating NS2Pro rumble pipeline to RAM")
relocate_to_ram(src/ns2pro_serial_bridge.cpp.o
    ".text=.time_critical.ns2pro_serial_bridge_text|.rodata=.time_critical.ns2pro_serial_bridge_rodata|.rodata.cst4=.time_critical.ns2pro_serial_bridge_cst4|.rodata.cst8=.time_critical.ns2pro_serial_bridge_cst8|.rodata.str1.1=.time_critical.ns2pro_serial_bridge_strings1|.rodata.str1.4=.time_critical.ns2pro_serial_bridge_strings4"
    "Relocating NS2Pro serial rumble bridge to RAM")

set_source_files_properties(src/ram_mem.c PROPERTIES COMPILE_OPTIONS
        "-fno-builtin;-fno-tree-loop-distribute-patterns")

target_compile_options(${PROJECT_NAME} PRIVATE -fsigned-char)
target_compile_definitions(wdl_resampler PRIVATE
        WDL_RESAMPLE_TYPE=float
)
if(${SYS_CLOCK_KHZ_VALUE} GREATER 266000)
    set(PICO_FLASH_SPI_CLKDIV 4)
else()
    set(PICO_FLASH_SPI_CLKDIV 2)
endif()
target_compile_definitions(bs2_default PRIVATE PICO_FLASH_SPI_CLKDIV=${PICO_FLASH_SPI_CLKDIV})
target_compile_definitions(${PROJECT_NAME} PRIVATE PICO_EMBED_XIP_SETUP=1)
target_compile_definitions(${PROJECT_NAME} PRIVATE
        CYW43_LWIP=0
        ENABLE_SERIAL=${ENABLE_SERIAL_VALUE}
        ENABLE_NS2PRO_SERIAL_BRIDGE=${ENABLE_NS2PRO_SERIAL_BRIDGE_VALUE}
        PICO_FLASH_ASSUME_CORE1_SAFE=1
        WDL_RESAMPLE_TYPE=float
        CYW43_PIO_CLOCK_DIV_INT=${CYW43_PIO_CLOCK_DIV_VALUE}
        PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0
        PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0
        PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0
        ENABLE_VERBOSE=${ENABLE_VERBOSE_VALUE}
        ENABLE_BATT_LED=${ENABLE_BATT_LED_VALUE}
        SYS_CLOCK_KHZ=${SYS_CLOCK_KHZ_VALUE}
        DISABLE_SPEAKER_PROC=${DISABLE_SPEAKER_PROC_VALUE}
)

option(ENABLE_WAKE_HID "Wake the Windows host on PS-button press" OFF)
if(ENABLE_WAKE_HID)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
            ENABLE_WAKE_HID
    )
    target_sources(${PROJECT_NAME} PRIVATE
            src/wake.cpp
            src/ps_shortcut.cpp
     )
endif()

option(WAKE_DEBUG "Print wake-FSM transitions to UART (developer use)" OFF)
if(WAKE_DEBUG)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
            WAKE_DEBUG
    )
endif()

pico_set_program_name(${PROJECT_NAME} "${FIRMWARE_OUTPUT_NAME}")
pico_set_program_version(${PROJECT_NAME} "${FIRMWARE_VERSION}")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} ${ENABLE_SERIAL_VALUE})

# Add the standard include files to the build
#target_include_directories(${PROJECT_NAME} PRIVATE
#        ${CMAKE_CURRENT_LIST_DIR}
#)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_multicore
        hardware_watchdog
        hardware_timer
        hardware_flash
        pico_btstack_ble
        pico_btstack_classic
#        pico_cyw43_arch_threadsafe_background
        pico_cyw43_arch_poll
        pico_btstack_cyw43
        tinyusb_device
        tinyusb_board
        wdl_resampler
        opus
)

pico_add_extra_outputs(${PROJECT_NAME})

