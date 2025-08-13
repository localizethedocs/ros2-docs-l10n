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
find_package(Gettext    MODULE REQUIRED COMPONENTS Msgmerge)
find_package(Crowdin    MODULE REQUIRED)
include(LogUtils)
include(JsonUtils)
include(GettextUtils)


message(STATUS "The followings are the required environment variables...")
remove_cmake_message_indent()
message("")
message("ENV{CROWDIN_PROJECT_ID}      = $ENV{CROWDIN_PROJECT_ID}")
message("ENV{CROWDIN_PERSONAL_TOKEN}  = $ENV{CROWDIN_PERSONAL_TOKEN}")
message("ENV{CROWDIN_BASE_URL}        = $ENV{CROWDIN_BASE_URL}")
message("")
restore_cmake_message_indent()
if ("$ENV{CROWDIN_PROJECT_ID}" STREQUAL "")
    message(FATAL_ERROR "Missing ENV{CROWDIN_PROJECT_ID}.")
endif()
if ("$ENV{CROWDIN_PERSONAL_TOKEN}" STREQUAL "")
    message(FATAL_ERROR "Missing ENV{CROWDIN_PERSONAL_TOKEN}.")
endif()
if ("$ENV{CROWDIN_BASE_URL}" STREQUAL "")
    message(FATAL_ERROR "Missing ENV{CROWDIN_BASE_URL}.")
endif()


file(READ "${LANGUAGES_JSON_PATH}" LANGUAGES_JSON_CNT)
if (NOT LANGUAGE STREQUAL "all")
    set(LANGUAGE_LIST "${LANGUAGE}")
endif()
foreach(_LANGUAGE ${LANGUAGE_LIST})
    if (_LANGUAGE STREQUAL LANGUAGE_SOURCE)
        continue()
    endif()


    get_json_value_by_dot_notation(
        IN_JSON_OBJECT    "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION   ".${_LANGUAGE}.crowdin"
        OUT_JSON_VALUE    _LANGUAGE_CROWDIN)


    message(STATUS "Preparing to download '${_LANGUAGE_CROWDIN}' translations for '${VERSION}' version from Crowdin...")
    set(CROWDIN_PO_DIR  "${PROJ_L10N_VERSION_CROWDIN_DIR}/${_LANGUAGE}")
    set(SINGLE_PO_FILE  "${PROJ_L10N_VERSION_SINGLE_DIR}/${_LANGUAGE}.po")
    set(LOCALE_PO_DIR   "${PROJ_L10N_VERSION_LOCALE_DIR}/${_LANGUAGE}")
    set(LOCALE_POT_DIR  "${PROJ_L10N_VERSION_LOCALE_DIR}/pot")
    remove_cmake_message_indent()
    message("")
    message("_LANGUAGE          = ${_LANGUAGE}")
    message("_LANGUAGE_CROWDIN  = ${_LANGUAGE_CROWDIN}")
    message("CROWDIN_YML_PATH   = ${CROWDIN_YML_PATH}")
    message("CROWDIN_PO_DIR     = ${CROWDIN_PO_DIR}")
    message("SINGLE_PO_FILE     = ${SINGLE_PO_FILE}")
    message("LOCALE_PO_DIR      = ${LOCALE_PO_DIR}")
    message("LOCALE_POT_DIR     = ${LOCALE_POT_DIR}")
    message("")
    restore_cmake_message_indent()


    message(STATUS "Downloading '${_LANGUAGE_CROWDIN}' translations for '${VERSION}' version from Crowdin...")
    remove_cmake_message_indent()
    message("")
    execute_process(
        COMMAND ${Crowdin_EXECUTABLE} download translations
                --language=${_LANGUAGE_CROWDIN}
                --branch=${VERSION}
                --config=${CROWDIN_YML_PATH}
                # --export-only-approved
                --no-progress
                --verbose
        WORKING_DIRECTORY ${PROJ_L10N_VERSION_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        COMMAND_ERROR_IS_FATAL ANY)
    message("")
    restore_cmake_message_indent()


    message(STATUS "Concatenating '${_LANGUAGE}' translations of '${VERSION}' verison from Crowdin into a single compendium file...")
    remove_cmake_message_indent()
    message("")
    concat_po_from_locale_to_single(
        IN_LOCALE_PO_DIR        "${CROWDIN_PO_DIR}"
        IN_SINGLE_PO_FILE       "${SINGLE_PO_FILE}"
        IN_WRAP_WIDTH           "${GETTEXT_WRAP_WIDTH}")
    message("")
    restore_cmake_message_indent()


    message(STATUS "Merging '${_LANGUAGE}' translations of '${VERSION}' verison from Crowdin with the compendium file...")
    remove_cmake_message_indent()
    message("")
    merge_po_with_compendium_from_src_to_dst(
        IN_LANGUAGE             "${_LANGUAGE}"
        IN_WRAP_WIDTH           "${GETTEXT_WRAP_WIDTH}"
        IN_SRC_SINGLE_PO_FILE   "${SINGLE_PO_FILE}"
        IN_SRC_LOCALE_PO_DIR    "${CROWDIN_PO_DIR}"
        IN_DST_LOCALE_PO_DIR    "${LOCALE_PO_DIR}"
        IN_DST_LOCALE_POT_DIR   "${LOCALE_POT_DIR}")
    message("")
    restore_cmake_message_indent()
endforeach()
unset(_LANGUAGE)
