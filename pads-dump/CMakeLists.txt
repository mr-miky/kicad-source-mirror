# SPDX-License-Identifier: GPL-3.0-or-later
# Add this directory at the END of KiCad's top-level CMakeLists.txt.
if(NOT TARGET kicommon OR NOT TARGET kimath)
    message(FATAL_ERROR "Usare add_subdirectory(pads-dump) nella build dei sorgenti KiCad")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PADS_DIR "${CMAKE_SOURCE_DIR}/pcbnew/pcb_io/pads")
set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
add_custom_command(
    OUTPUT "${GENERATED}/dump_fields.inc" "${GENERATED}/dump_getters.inc"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/generate.py"
            "${CMAKE_SOURCE_DIR}" "${GENERATED}"
    DEPENDS generate.py "${PADS_DIR}/pads_parser.h" "${PADS_DIR}/pads_binary_parser.h"
    VERBATIM)
set(PARSER_SOURCES "${PADS_DIR}/pads_binary_parser.cpp")
if(EXISTS "${PADS_DIR}/pads_sdb.cpp")
    list(APPEND PARSER_SOURCES "${PADS_DIR}/pads_sdb.cpp")
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/common/io/pads/pads_binary_utils.cpp")
    list(APPEND PARSER_SOURCES "${CMAKE_SOURCE_DIR}/common/io/pads/pads_binary_utils.cpp")
endif()
add_executable(pads-dump pads_dump.cpp pads_binary_graphics.cpp pads_binary_parttypes.cpp ${PARSER_SOURCES}
    "${GENERATED}/dump_fields.inc" "${GENERATED}/dump_getters.inc")
target_compile_features(pads-dump PRIVATE cxx_std_20)
target_include_directories(pads-dump PRIVATE
    "${GENERATED}" "${PADS_DIR}" "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/common" "${CMAKE_SOURCE_DIR}/libs/kimath/include"
    "${CMAKE_SOURCE_DIR}/libs/core/include" "${CMAKE_BINARY_DIR}")
target_link_libraries(pads-dump PRIVATE kicommon kimath fmt::fmt Boost::headers ${wxWidgets_LIBRARIES})

# The Python exporter reuses the JSON generated from the same parser instance.
target_compile_definitions(pads-dump PRIVATE
    PADS_DUMP_EXPORTER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/export_decals.py"
    PADS_DUMP_PYTHON_PATH="${Python3_EXECUTABLE}")
