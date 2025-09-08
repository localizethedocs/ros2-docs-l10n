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
find_package(Python     MODULE REQUIRED COMPONENTS Interpreter)
find_package(Sphinx     MODULE REQUIRED COMPONENTS Build)
include(LogUtils)
include(JsonUtils)
include(GettextUtils)


message(STATUS "Removing directory '${PROJ_OUT_REPO_DOCS_LOCALE_DIR}/'...")
if (EXISTS "${PROJ_OUT_REPO_DOCS_LOCALE_DIR}")
    file(REMOVE_RECURSE "${PROJ_OUT_REPO_DOCS_LOCALE_DIR}")
    remove_cmake_message_indent()
    message("")
    message("Directory '${PROJ_OUT_REPO_DOCS_LOCALE_DIR}/' exists.")
    message("Removed '${PROJ_OUT_REPO_DOCS_LOCALE_DIR}/'.")
    message("")
    restore_cmake_message_indent()
else()
    remove_cmake_message_indent()
    message("")
    message("Directory '${PROJ_OUT_REPO_DOCS_LOCALE_DIR}/' does NOT exist.")
    message("No need to remove '${PROJ_OUT_REPO_DOCS_LOCALE_DIR}/'.")
    message("")
    restore_cmake_message_indent()
endif()


message(STATUS "Copying .po files to the local repository...")
if (NOT LANGUAGE STREQUAL "all")
    set(PO_SRC_DIR  "${PROJ_L10N_VERSION_LOCALE_DIR}/${LANGUAGE}")
    set(PO_DST_DIR  "${PROJ_OUT_REPO_DOCS_LOCALE_DIR}/${LANGUAGE}")
else()
    set(PO_SRC_DIR  "${PROJ_L10N_VERSION_LOCALE_DIR}")
    set(PO_DST_DIR  "${PROJ_OUT_REPO_DOCS_LOCALE_DIR}")
endif()
remove_cmake_message_indent()
message("")
message("From: ${PO_SRC_DIR}/")
message("To:   ${PO_DST_DIR}/")
message("")
copy_po_from_src_to_dst(
    IN_SRC_DIR  "${PO_SRC_DIR}"
    IN_DST_DIR  "${PO_DST_DIR}")
message("")
restore_cmake_message_indent()


file(READ "${LANGUAGES_JSON_PATH}" LANGUAGES_JSON_CNT)
if (NOT LANGUAGE STREQUAL "all")
    set(LANGUAGE_LIST "${LANGUAGE}")
endif()


foreach(_LANGUAGE ${LANGUAGE_LIST})
    get_json_value_by_dot_notation(
        IN_JSON_OBJECT      "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION     ".${_LANGUAGE}.langtag"
        OUT_JSON_VALUE      _LANGTAG)


    message(STATUS "Running 'sphinx-build' command with '${SPHINX_BUILDER}' builder to build documentation for '${_LANGUAGE}' language...")
    if (CMAKE_HOST_LINUX)
        set(ENV_PATH                "${PROJ_CONDA_DIR}/bin:$ENV{PATH}")
        set(ENV_LD_LIBRARY_PATH     "${PROJ_CONDA_DIR}/lib:$ENV{LD_LIBRARY_PATH}")
        set(ENV_PYTHONPATH          "${PROJ_OUT_REPO_DOCS_EXTNS_DIR}")
        set(ENV_VARS_OF_SYSTEM      PATH=${ENV_PATH}
                                    LD_LIBRARY_PATH=${ENV_LD_LIBRARY_PATH}
                                    PYTHONPATH=${ENV_PYTHONPATH})
    elseif (CMAKE_HOST_WIN32)
        set(ENV_PATH                "${PROJ_CONDA_DIR}/bin"
                                    "${PROJ_CONDA_DIR}/Scripts"
                                    "${PROJ_CONDA_DIR}/Library/bin"
                                    "${PROJ_CONDA_DIR}"
                                    "$ENV{PATH}")
        set(ENV_PYTHONPATH          "${PROJ_OUT_REPO_DOCS_EXTNS_DIR}")
        string(REPLACE ";" "\\\\;" ENV_PATH "${ENV_PATH}")
        string(REPLACE ";" "\\\\;" ENV_PYTHONPATH "${ENV_PYTHONPATH}")
        set(ENV_VARS_OF_SYSTEM      PATH=${ENV_PATH}
                                    PYTHONPATH=${ENV_PYTHONPATH})
    else()
        message(FATAL_ERROR "Invalid OS platform. (${CMAKE_HOST_SYSTEM_NAME})")
    endif()
    set(HTML_LOGO_PATH              "source/Releases/${VERSION}-small.png")
    set(HTML_BASEURL                "${BASEURL_HREF}")
    set(CURRENT_VERSION             "${VERSION}")
    set(CURRENT_LANGUAGE            "${_LANGTAG}")
    set(VERSIONS_JSON_PATH          "versions.json")
    set(WARNING_FILE_PATH           "${PROJ_BINARY_DIR}/log-${SPHINX_BUILDER}-${VERSION}-${_LANGTAG}.txt")
    remove_cmake_message_indent()
    message("")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env
                ${ENV_VARS_OF_SYSTEM}
                ${Sphinx_BUILD_EXECUTABLE}
                -b ${SPHINX_BUILDER}
                -D language=${_LANGUAGE}
                -D locale_dirs=${LOCALE_TO_SOURCE_DIR}            # Relative to <sourcedir>.
                -D templates_path=${TMPLS_TO_CONFIG_DIR}          # Relative to <configdir>.
                -D gettext_compact=${SPHINX_GETTEXT_COMPACT}
                -D gettext_additional_targets=${SPHINX_GETTEXT_TARGETS}
                -D html_logo=${HTML_LOGO_PATH}
                -D html_baseurl=${HTML_BASEURL}                   # Passed to 'custom.py'.
                -D latest_version=${LATEST_VERSION}               # Passed to 'custom.py'.
                -D current_version=${CURRENT_VERSION}             # Passed to 'custom.py'.
                -D current_language=${CURRENT_LANGUAGE}           # Passed to 'custom.py'.
                -D versions_json_path=${VERSIONS_JSON_PATH}       # Passed to 'custom.py'.
                -w ${WARNING_FILE_PATH}
                -j ${SPHINX_JOB_NUMBER}
                ${SPHINX_VERBOSE_ARGS}
                -c ${PROJ_OUT_REPO_DOCS_CONFIG_DIR}               # <configdir>, where conf.py locates.
                ${PROJ_OUT_REPO_DOCS_SOURCE_DIR}                  # <sourcedir>, where index.rst locates.
                ${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}    # <outputdir>, where .html generates.
        WORKING_DIRECTORY ${PROJ_OUT_REPO_DOCS_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        RESULT_VARIABLE RES_VAR
        OUTPUT_VARIABLE OUT_VAR OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_VARIABLE  ERR_VAR ERROR_STRIP_TRAILING_WHITESPACE)
    if (RES_VAR EQUAL 0)
        if (ERR_VAR)
            string(APPEND WARNING_REASON
            "The command succeeded but had some warnings.\n\n"
            "    result:\n\n${RES_VAR}\n\n"
            "    stderr:\n\n${ERR_VAR}")
            message("${WARNING_REASON}")
        endif()
    else()
        string(APPEND FAILURE_REASON
        "The command failed with fatal errors.\n"
        "    result:\n${RES_VAR}\n"
        "    stderr:\n${ERR_VAR}")
        message(FATAL_ERROR "${FAILURE_REASON}")
    endif()
    message("")
    restore_cmake_message_indent()


    if (REMOVE_REDUNDANT)
        message(STATUS "Removing redundant files/directories...")
        file(REMOVE_RECURSE "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/.doctrees/")
        file(REMOVE         "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/.buildinfo")
        file(REMOVE         "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/objects.inv")
        remove_cmake_message_indent()
        message("")
        message("Removed '${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/.doctrees/'.")
        message("Removed '${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/.buildinfo'.")
        message("Removed '${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/objects.inv'.")
        message("")
        restore_cmake_message_indent()
    endif()
endforeach()
unset(_LANGUAGE)


#[============================================================[
# Configure redirecting index.html files.
#]============================================================]


set(REDIRECT_LANGTAG    "en-us")
set(REDIRECT_VERSION    "rolling")


message(STATUS "Configuring 'index.html' file to the root of the builder directory...")
set(REDIRECT_URL        "${REDIRECT_LANGTAG}/${REDIRECT_VERSION}/index.html")
file(MAKE_DIRECTORY     "${PROJ_OUT_BUILDER_DIR}")
configure_file(
    "${PROJ_CMAKE_CUSTOM_DIR}/index.html.in"
    "${PROJ_OUT_BUILDER_DIR}/index.html"
    @ONLY)
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_CUSTOM_DIR}/index.html.in")
message("To:   ${PROJ_OUT_BUILDER_DIR}/index.html")
message("")
restore_cmake_message_indent()


message(STATUS "Configuring 'index.html' file to the langtag subdir of the builder directory...")
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_CUSTOM_DIR}/index.html.in")
foreach(_LANGUAGE ${LANGUAGE_LIST})
    get_json_value_by_dot_notation(
        IN_JSON_OBJECT      "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION     ".${_LANGUAGE}.langtag"
        OUT_JSON_VALUE      _LANGTAG)
    set(REDIRECT_URL        "${REDIRECT_VERSION}/index.html")
    file(MAKE_DIRECTORY     "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}")
    configure_file(
        "${PROJ_CMAKE_CUSTOM_DIR}/index.html.in"
        "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/index.html"
        @ONLY)
    message("To:   ${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/index.html")
endforeach()
unset(_LANGUAGE)
message("")
restore_cmake_message_indent()


#[============================================================[
# Configure the provenance dropdown.
#]============================================================]


set(UPSTREAM_DOCS   "https://docs.ros.org")
set(UPSTREAM_REPO   "https://github.com/ros2/ros2_documentation")
set(INSERT_POINT    "div[role=\"main\"]")


message(STATUS "Configuring 'ltd-provenance.js' file to the version subdir of the builder directory...")
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_PLUGINS_DIR}/provenance/ltd-sphinx.js.in")
foreach(_LANGUAGE ${LANGUAGE_LIST})
    get_json_value_by_dot_notation(
        IN_JSON_OBJECT      "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION     ".${_LANGUAGE}.langtag"
        OUT_JSON_VALUE      _LANGTAG)
    set(CURRENT_VERSION     "${VERSION}")
    set(CURRENT_LANGUAGE    "${_LANGTAG}")
    file(MAKE_DIRECTORY "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}")
    configure_file(
        "${PROJ_CMAKE_PLUGINS_DIR}/provenance/ltd-sphinx.js.in"
        "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/ltd-provenance.js"
        @ONLY)
    message("To:   ${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/ltd-provenance.js")
endforeach()
message("")
restore_cmake_message_indent()


#[============================================================[
# Configure the flyout navigation menu.
#]============================================================]


message(STATUS "Configuring 'ltd-config.js' file to the root of the builder directory...")
file(MAKE_DIRECTORY "${PROJ_OUT_BUILDER_DIR}")
configure_file(
    "${PROJ_CMAKE_CUSTOM_DIR}/ltd-config.js"
    "${PROJ_OUT_BUILDER_DIR}/ltd-config.js"
    @ONLY)
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_CUSTOM_DIR}/ltd-config.js")
message("To:   ${PROJ_OUT_BUILDER_DIR}/ltd-config.js")
message("")
restore_cmake_message_indent()


message(STATUS "Configuring 'ltd-flyout.js' file to the root of the builder directory...")
file(MAKE_DIRECTORY "${PROJ_OUT_BUILDER_DIR}")
configure_file(
    "${PROJ_CMAKE_PLUGINS_DIR}/flyout/ltd-flyout.js"
    "${PROJ_OUT_BUILDER_DIR}/ltd-flyout.js"
    @ONLY)
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_PLUGINS_DIR}/flyout/ltd-flyout.js")
message("To:   ${PROJ_OUT_BUILDER_DIR}/ltd-flyout.js")
message("")
restore_cmake_message_indent()


message(STATUS "Configuring 'ltd-icon.svg' file to the root of the builder directory...")
file(MAKE_DIRECTORY "${PROJ_OUT_BUILDER_DIR}")
configure_file(
    "${PROJ_CMAKE_PLUGINS_DIR}/flyout/ltd-icon.svg"
    "${PROJ_OUT_BUILDER_DIR}/ltd-icon.svg"
    @ONLY)
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_PLUGINS_DIR}/flyout/ltd-icon.svg")
message("To:   ${PROJ_OUT_BUILDER_DIR}/ltd-icon.svg")
message("")
restore_cmake_message_indent()


message(STATUS "Configuring 'ltd-current.js' file to the version subdir of the builder directory...")
remove_cmake_message_indent()
message("")
message("From: ${PROJ_CMAKE_PLUGINS_DIR}/flyout/ltd-current.js.in")
foreach(_LANGUAGE ${LANGUAGE_LIST})
    get_json_value_by_dot_notation(
        IN_JSON_OBJECT      "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION     ".${_LANGUAGE}.langtag"
        OUT_JSON_VALUE      _LANGTAG)
    set(CURRENT_LANGUAGE    "${_LANGTAG}")
    set(CURRENT_VERSION     "${VERSION}")
    file(MAKE_DIRECTORY "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}")
    configure_file(
        "${PROJ_CMAKE_PLUGINS_DIR}/flyout/ltd-current.js.in"
        "${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/ltd-current.js"
        @ONLY)
    message("To:   ${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/ltd-current.js")
endforeach()
unset(_LANGUAGE)
message("")
restore_cmake_message_indent()


#[============================================================[
# Display home pages of the built documentation.
#]============================================================]


message(STATUS "The '${SPHINX_BUILDER}' documentation is built succesfully!")
remove_cmake_message_indent()
message("")
foreach(_LANGUAGE ${LANGUAGE_LIST})
    get_json_value_by_dot_notation(
        IN_JSON_OBJECT      "${LANGUAGES_JSON_CNT}"
        IN_DOT_NOTATION     ".${_LANGUAGE}.langtag"
        OUT_JSON_VALUE      _LANGTAG)
    message("${_LANGUAGE} : ${PROJ_OUT_BUILDER_DIR}/${_LANGTAG}/${VERSION}/index.html")
endforeach()
message("")
restore_cmake_message_indent()
