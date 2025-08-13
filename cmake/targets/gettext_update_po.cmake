# Distributed under the OSI-approved BSD 3-Clause License.
# See accompanying file LICENSE-BSD for details.

cmake_minimum_required(VERSION 3.25)
get_filename_component(SCRIPT_NAME "${CMAKE_CURRENT_LIST_FILE}" NAME_WE)
set(CMAKE_MESSAGE_INDENT "[${VERSION}][${LANGUAGE}] ")
set(CMAKE_MESSAGE_INDENT_BACKUP "${CMAKE_MESSAGE_INDENT}")
message(STATUS "-------------------- ${SCRIPT_NAME} --------------------")


set(CMAKE_MODULE_PATH   "${PROJ_CMAKE_MODULES_DIR}")
set(CMAKE_PROGRAM_PATH  "${PROJ_CONDA_DIR}"
                        "${PROJ_CONDA_DIR}/Library")
find_package(Git        MODULE REQUIRED)
find_package(Gettext    MODULE REQUIRED COMPONENTS Msgcat Msgmerge)
include(LogUtils)
include(JsonUtils)
include(GettextUtils)


file(READ "${REFERENCES_JSON_PATH}" REFERENCES_JSON_CNT)
if (NOT LANGUAGE STREQUAL "all")
    set(LANGUAGE_LIST "${LANGUAGE}")
endif()
foreach(_LANGUAGE ${LANGUAGE_LIST})
    if (_LANGUAGE STREQUAL LANGUAGE_SOURCE)
        continue()
    endif()


    message(STATUS "Determining whether it is required to update .po files...")
    get_reference_of_pot_and_po_from_json(
        IN_JSON_CNT                     "${REFERENCES_JSON_CNT}"
        IN_VERSION_TYPE                 "${VERSION_TYPE}"
        IN_LANGUAGE                     "${_LANGUAGE}"
        OUT_POT_OBJECT                  CURRENT_POT_OBJECT
        OUT_POT_REFERENCE               CURRENT_POT_REFERENCE
        OUT_PO_OBJECT                   CURRENT_PO_OBJECT
        OUT_PO_REFERENCE                CURRENT_PO_REFERENCE)
    if (MODE_OF_UPDATE STREQUAL "COMPARE")
        if (NOT CURRENT_POT_REFERENCE STREQUAL CURRENT_PO_REFERENCE)
            set(UPDATE_PO_REQUIRED      ON)
        else()
            set(UPDATE_PO_REQUIRED      OFF)
        endif()
    elseif (MODE_OF_UPDATE STREQUAL "ALWAYS")
        set(UPDATE_PO_REQUIRED          ON)
    elseif (MODE_OF_UPDATE STREQUAL "NEVER")
        if (NOT CURRENT_PO_REFERENCE)
            set(UPDATE_PO_REQUIRED      ON)
        else()
            set(UPDATE_PO_REQUIRED      OFF)
        endif()
    else()
        message(FATAL_ERROR "Invalid MODE_OF_UPDATE value. (${MODE_OF_UPDATE})")
    endif()
    remove_cmake_message_indent()
    message("")
    message("CURRENT_POT_OBJECT     = ${CURRENT_POT_OBJECT}")
    message("CURRENT_PO_OBJECT      = ${CURRENT_PO_OBJECT}")
    message("CURRENT_POT_REFERENCE  = ${CURRENT_POT_REFERENCE}")
    message("CURRENT_PO_REFERENCE   = ${CURRENT_PO_REFERENCE}")
    message("MODE_OF_UPDATE         = ${MODE_OF_UPDATE}")
    message("UPDATE_PO_REQUIRED     = ${UPDATE_PO_REQUIRED}")
    message("")
    restore_cmake_message_indent()


    if (NOT UPDATE_PO_REQUIRED)
        message(STATUS "No need to update .po files for '${_LANGUAGE}' language.")
        continue()
    else()
        message(STATUS "Prepare to update .po files for '${_LANGUAGE}' language.")
    endif()


    message(STATUS "Updating .po files for '${_LANGUAGE}' language...")
    set(LOCALE_POT_DIR  "${PROJ_L10N_VERSION_LOCALE_DIR}/pot")
    set(LOCALE_PO_DIR   "${PROJ_L10N_VERSION_LOCALE_DIR}/${_LANGUAGE}")
    remove_cmake_message_indent()
    message("")
    message("From: ${LOCALE_POT_DIR}/")
    message("To:   ${LOCALE_PO_DIR}/")
    message("")
    update_po_from_pot_in_locale(
        IN_LOCALE_POT_DIR   "${LOCALE_POT_DIR}"
        IN_LOCALE_PO_DIR    "${LOCALE_PO_DIR}"
        IN_LANGUAGE         "${_LANGUAGE}"
        IN_WRAP_WIDTH       "${GETTEXT_WRAP_WIDTH}")
    message("")
    restore_cmake_message_indent()


    set_json_value_by_dot_notation(
        IN_JSON_OBJECT      "${REFERENCES_JSON_CNT}"
        IN_DOT_NOTATION     ".po.${_LANGUAGE}"
        IN_JSON_VALUE       "${CURRENT_POT_OBJECT}"
        OUT_JSON_OBJECT     REFERENCES_JSON_CNT)
endforeach()
unset(_LANGUAGE)


file(WRITE "${REFERENCES_JSON_PATH}" "${REFERENCES_JSON_CNT}")
