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
find_package(Gettext    MODULE REQUIRED COMPONENTS Msgcat Msgmerge)
include(LogUtils)
include(GettextUtils)


if (VERSION_COMPENDIUM STREQUAL VERSION OR
    VERSION_COMPENDIUM STREQUAL "")
    message(STATUS "No need to merge translations from compendium.")
    return()
endif()


if (NOT LANGUAGE STREQUAL "all")
    set(LANGUAGE_LIST "${LANGUAGE}")
endif()
foreach(_LANGUAGE ${LANGUAGE_LIST})
    if (_LANGUAGE STREQUAL LANGUAGE_SOURCE)
        continue()
    endif()


    set(SRC_VERSION         "${VERSION_COMPENDIUM}")
    set(SRC_LOCALE_PO_DIR   "${PROJ_L10N_DIR}/${SRC_VERSION}/locale/${_LANGUAGE}")
    set(SRC_SINGLE_PO_FILE  "${PROJ_L10N_DIR}/${SRC_VERSION}/.single/${_LANGUAGE}.po")
    set(DST_VERSION         "${VERSION}")
    set(DST_LOCALE_PO_DIR   "${PROJ_L10N_DIR}/${DST_VERSION}/locale/${_LANGUAGE}")
    set(DST_LOCALE_POT_DIR  "${PROJ_L10N_DIR}/${DST_VERSION}/locale/pot")
    remove_cmake_message_indent()
    message("")
    message("SRC_VERSION        = ${SRC_VERSION}")
    message("SRC_LOCALE_PO_DIR  = ${SRC_LOCALE_PO_DIR}")
    message("SRC_SINGLE_PO_FILE = ${SRC_SINGLE_PO_FILE}")
    message("DST_VERSION        = ${DST_VERSION}")
    message("DST_LOCALE_PO_DIR  = ${DST_LOCALE_PO_DIR}")
    message("DST_LOCALE_POT_DIR = ${DST_LOCALE_POT_DIR}")
    message("")
    restore_cmake_message_indent()


    message(STATUS "Concatenating '${_LANGUAGE}' translations of '${SRC_VERSION}' version into a single compendium file...")
    remove_cmake_message_indent()
    message("")
    concat_po_from_locale_to_single(
        IN_LOCALE_PO_DIR        "${SRC_LOCALE_PO_DIR}"
        IN_SINGLE_PO_FILE       "${SRC_SINGLE_PO_FILE}"
        IN_WRAP_WIDTH           "${GETTEXT_WRAP_WIDTH}")
    message("")
    restore_cmake_message_indent()


    message(STATUS "Merging '${_LANGUAGE}' translations of '${DST_VERSION}' version from '${SRC_VERSION}' version with the compendium file...")
    remove_cmake_message_indent()
    message("")
    merge_po_with_compendium_from_src_to_dst(
        IN_LANGUAGE             "${_LANGUAGE}"
        IN_WRAP_WIDTH           "${GETTEXT_WRAP_WIDTH}"
        IN_SRC_SINGLE_PO_FILE   "${SRC_SINGLE_PO_FILE}"
        IN_SRC_LOCALE_PO_DIR    "${SRC_LOCALE_PO_DIR}"
        IN_DST_LOCALE_PO_DIR    "${DST_LOCALE_PO_DIR}"
        IN_DST_LOCALE_POT_DIR   "${DST_LOCALE_POT_DIR}")
    message("")
    restore_cmake_message_indent()
endforeach()
unset(_LANGUAGE)
