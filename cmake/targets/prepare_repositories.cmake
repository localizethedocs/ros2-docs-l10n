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
include(LogUtils)
include(GitUtils)
include(JsonUtils)


#[============================================================[
# Prepare and synchronize the repository:
# - Clone from remote if not present.
# - Switch to 'current' branch.
# - Clean untracked files and submodules.
# - Fetch the latest commit or tag based on VERSION_TYPE.
#]============================================================]


message(STATUS "Cloning the repository from remote to local...")
set(REMOTE_URL  "${REMOTE_URL_OF_DOCS}")
set(LOCAL_PATH  "${PROJ_OUT_REPO_DIR}")
remove_cmake_message_indent()
message("")
message("REMOTE_URL: ${REMOTE_URL}")
message("LOCAL_PATH: ${LOCAL_PATH}/")
message("")
clone_repository_from_remote_to_local(
    IN_REMOTE_URL   "${REMOTE_URL}"
    IN_LOCAL_PATH   "${LOCAL_PATH}")
message("")
restore_cmake_message_indent()


message(STATUS "Switching to the local branch 'current' of the repository...")
remove_cmake_message_indent()
message("")
execute_process(
    COMMAND ${Git_EXECUTABLE} checkout -B current
    WORKING_DIRECTORY ${PROJ_OUT_REPO_DIR}
    ECHO_OUTPUT_VARIABLE
    ECHO_ERROR_VARIABLE
    COMMAND_ERROR_IS_FATAL ANY)
message("")
restore_cmake_message_indent()


message(STATUS "Removing untracked files/directories of the repository...")
remove_cmake_message_indent()
message("")
execute_process(
    COMMAND ${Git_EXECUTABLE} clean -xfdf
    WORKING_DIRECTORY ${PROJ_OUT_REPO_DIR}
    ECHO_OUTPUT_VARIABLE
    ECHO_ERROR_VARIABLE
    COMMAND_ERROR_IS_FATAL ANY)
if (EXISTS "${PROJ_OUT_REPO_DIR}/.gitmodules")
    message("")
    execute_process(
        COMMAND ${Git_EXECUTABLE} submodule foreach --recursive
                ${Git_EXECUTABLE} clean -xfdf
        WORKING_DIRECTORY ${PROJ_OUT_REPO_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        COMMAND_ERROR_IS_FATAL ANY)
endif()
message("")
restore_cmake_message_indent()


if (VERSION_TYPE STREQUAL "branch")
    message(STATUS "Getting the latest commit of the branch '${BRANCH_NAME}' from the remote...")
    get_git_latest_commit_on_branch_name(
        IN_LOCAL_PATH       "${PROJ_OUT_REPO_DIR}"
        IN_SOURCE_TYPE      "remote"
        IN_BRANCH_NAME      "${BRANCH_NAME}"
        OUT_COMMIT_HASH     LATEST_COMMIT_HASH)
    remove_cmake_message_indent()
    message("")
    message("LATEST_COMMIT_HASH = ${LATEST_COMMIT_HASH}")
    message("")
    restore_cmake_message_indent()
    message(STATUS "Fetching the latest commit '${LATEST_COMMIT_HASH}' to the local branch '${BRANCH_NAME}'...")
    remove_cmake_message_indent()
    message("")
    execute_process(
        COMMAND ${Git_EXECUTABLE} fetch origin
                ${LATEST_COMMIT_HASH}:refs/heads/${BRANCH_NAME}
                --depth=1
                --verbose
                --force
        WORKING_DIRECTORY ${PROJ_OUT_REPO_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        COMMAND_ERROR_IS_FATAL ANY)
    message("")
    restore_cmake_message_indent()
elseif (VERSION_TYPE STREQUAL "tag")
    message(STATUS "Getting the latest tag of '${TAG_PATTERN}' from the remote...")
    get_git_latest_tag_on_tag_pattern(
        IN_LOCAL_PATH       "${PROJ_OUT_REPO_DIR}"
        IN_SOURCE_TYPE      "remote"
        IN_TAG_PATTERN      "${TAG_PATTERN}"
        IN_TAG_SUFFIX       "${TAG_SUFFIX}"
        OUT_TAG             LATEST_TAG)
    remove_cmake_message_indent()
    message("")
    message("LATEST_TAG = ${LATEST_TAG}")
    message("")
    restore_cmake_message_indent()
    message(STATUS "Fetching the latest tag '${LATEST_TAG}' to the local tag '${LATEST_TAG}'...")
    remove_cmake_message_indent()
    message("")
    execute_process(
        COMMAND ${Git_EXECUTABLE} fetch origin
                refs/tags/${LATEST_TAG}:refs/tags/${LATEST_TAG}
                --depth=1
                --verbose
        WORKING_DIRECTORY ${PROJ_OUT_REPO_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        COMMAND_ERROR_IS_FATAL ANY)
    message("")
    restore_cmake_message_indent()
else()
    message(FATAL_ERROR "Invalid VERSION_TYPE value. (${VERSION_TYPE})")
endif()
