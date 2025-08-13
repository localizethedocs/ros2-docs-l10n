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
find_package(Gettext    MODULE REQUIRED COMPONENTS Msgattrib)
include(LogUtils)
include(JsonUtils)
include(GettextUtils)


set(STATISTICS_TXT_CNT "")
foreach(_LANGUAGE ${LANGUAGE_LIST})
    if (_LANGUAGE STREQUAL LANGUAGE_SOURCE)
        continue()
    endif()


    message(STATUS "Calculating the statistics for '${_LANGUAGE}' language...")
    remove_cmake_message_indent()
    message("")
    caculate_statistics_of_gettext(
        IN_LOCALE_PO_DIR              "${PROJ_L10N_VERSION_LOCALE_DIR}/${_LANGUAGE}"
        IN_PADDING_LENGTH             "3"
        OUT_NUM_OF_PO_COMPLETED       NUM_OF_PO_COMPLETED
        OUT_NUM_OF_PO_PROGRESSING     NUM_OF_PO_PROGRESSING
        OUT_NUM_OF_PO_UNSTARTED       NUM_OF_PO_UNSTARTED
        OUT_NUM_OF_PO_TOTAL           NUM_OF_PO_TOTAL
        OUT_PCT_OF_PO_COMPLETED       PCT_OF_PO_COMPLETED
        OUT_NUM_OF_MSGID_TRANSLATED   NUM_OF_MSGID_TRANSLATED
        OUT_NUM_OF_MSGID_FUZZY        NUM_OF_MSGID_FUZZY
        OUT_NUM_OF_MSGID_TOTAL        NUM_OF_MSGID_TOTAL
        OUT_PCT_OF_MSGID_TRANSLATED   PCT_OF_MSGID_TRANSLATED)
    message("")
    set(LOG_MESSAGES "[${_LANGUAGE}]\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of completed   po    files   : ${NUM_OF_PO_COMPLETED}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of progressing po    files   : ${NUM_OF_PO_PROGRESSING}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of unstarted   po    files   : ${NUM_OF_PO_UNSTARTED}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of total       po    files   : ${NUM_OF_PO_TOTAL}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Percentage of completed   po    files   : ${PCT_OF_PO_COMPLETED}%\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of translated  msgid entries : ${NUM_OF_MSGID_TRANSLATED}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of fuzzy       msgid entries : ${NUM_OF_MSGID_FUZZY}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Number     of total       msgid entries : ${NUM_OF_MSGID_TOTAL}\n")
    set(LOG_MESSAGES "${LOG_MESSAGES}Percentage of translated  msgid entries : ${PCT_OF_MSGID_TRANSLATED}%")
    message("${LOG_MESSAGES}")
    message("")
    restore_cmake_message_indent()
    set(STATISTICS_TXT_CNT "${STATISTICS_TXT_CNT}${LOG_MESSAGES}\n")
endforeach()
unset(_LANGUAGE)


file(WRITE "${STATISTICS_TXT_PATH}" "${STATISTICS_TXT_CNT}")
