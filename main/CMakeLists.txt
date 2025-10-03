# Gather all source files
file(GLOB SRC_FILES
    "*.cpp"
    "*.h"
    "**/*.cpp"
    "**/*.h"
    "parser.c"
    "modules/r4/src/*.c"       # Include r4 module C sources
    "modules/r4/include/*.h"   # Include r4 headers
)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS
        "."
        "compilation"
        "modules"
        "modules/r4/include"   # Add r4 include path
        "utils"
        "${CMAKE_BINARY_DIR}/generated"
    REQUIRES
        driver
        esp_wifi
        esp_timer
        efuse
        lwip
        esp_event
        esp_netif
        esp_adc
        esp32-ble-command
        esp32-zeug
        esp32-serial-flasher
        app_update
        esp_driver_uart
        esp_https_ota
        spi_flash
)

# Compile-time definitions
add_compile_definitions(OWL_TOKEN_RUN_LENGTH=256)

# C++ compile options
component_compile_options(-std=gnu++17 -Wsuggest-override)

# Parser specific compile flags
set_source_files_properties(parser.c PROPERTIES COMPILE_FLAGS -Wno-missing-field-initializers)
