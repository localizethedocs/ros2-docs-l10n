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
include(LogUtils)
include(JsonUtils)


if (CMAKE_HOST_WIN32)
    execute_process(
        COMMAND net session
        RESULT_VARIABLE RES_VAR
        OUTPUT_VARIABLE OUT_VAR OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_VARIABLE  ERR_VAR ERROR_STRIP_TRAILING_WHITESPACE)
    if (RES_VAR EQUAL 0)
        # It's adiministrator privileges.
    else()
        find_program(SUDO_COMMAND gsudo)
        if (NOT SUDO_COMMAND)
            message(FATAL_ERROR "Failed to find gsudo.")
        endif()
    endif()
else()
    set(SUDO_COMMAND "")
endif()


file(READ "${LANGUAGES_JSON_PATH}" LANGUAGES_JSON_CNT)
if (NOT LANGUAGE STREQUAL "all")
    set(LANGUAGE_LIST "${LANGUAGE}")
endif()


message(STATUS "Creating ReadTheDocs symlinks for language directories...")
remove_cmake_message_indent()
message("")
foreach(_LANGUAGE ${LANGUAGE_LIST})
    if (_LANGUAGE STREQUAL LANGUAGE_SOURCE)
        continue()
    endif()


    get_json_value_by_dot_notation(
        IN_JSON_OBJECT    "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION   ".${_LANGUAGE}.readthedocs"
        OUT_JSON_VALUE    _LANGUAGE_READTHEDOCS)


    if (_LANGUAGE STREQUAL _LANGUAGE_READTHEDOCS)
        message("No need to create ReadTheDocs symlink for language directory '${_LANGUAGE}'.")
    else()
        if (NOT EXISTS "${PROJ_L10N_VERSION_LOCALE_DIR}/${_LANGUAGE}")
            message(FATAL_ERROR "'${PROJ_L10N_VERSION_LOCALE_DIR}/${_LANGUAGE}' doesn't exist.")
        endif()
        execute_process(
            COMMAND ${SUDO_COMMAND}
                    ${CMAKE_COMMAND} -E create_symlink
                    ${_LANGUAGE}                # original directory
                    ${_LANGUAGE_READTHEDOCS}    # symbolic link
            WORKING_DIRECTORY ${PROJ_L10N_VERSION_LOCALE_DIR}
            RESULT_VARIABLE RES_VAR
            OUTPUT_VARIABLE OUT_VAR OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_VARIABLE  ERR_VAR ERROR_STRIP_TRAILING_WHITESPACE)
        if (RES_VAR EQUAL 0)
            message("Created ReadTheDocs symlink '${_LANGUAGE_READTHEDOCS}' for language directory '${_LANGUAGE}'.")
        else()
            string(APPEND FAILURE_REASON
            "The command failed with fatal errors.\n"
            "    result:\n${RES_VAR}\n"
            "    stdout:\n${OUT_VAR}\n"
            "    stderr:\n${ERR_VAR}")
            message(FATAL_ERROR "${FAILURE_REASON}")
        endif()
    endif()
endforeach()
unset(_LANGUAGE)
message("")
restore_cmake_message_indent()
