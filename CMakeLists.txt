# Distributed under the OSI-approved BSD 3-Clause License.
# See accompanying file LICENSE-BSD for details.

cmake_minimum_required(VERSION 3.25)
project(ros2-docs-l10n NONE)

#[============================================================[
# Environment Variables.
#]============================================================]

set(ENV_LANG              "en_US.UTF-8")
set(ENV_LANGUAGE          "en_US")
set(ENV_PYTHONNOUSERSITE  "1")

#[============================================================[
# CMP<NNNN> Defaults.
#]============================================================]

if (POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
    cmake_policy(GET CMP0144 CMAKE_POLICY_DEFAULT_CMP0144)
endif()

#[============================================================[
# Constants.
#]============================================================]

set(LATEST_VERSION "kilted")

#[============================================================[
# Cache Variables.
#]============================================================]

set(LANGUAGE "all"
    CACHE STRING "The language code of the documentation.")
    # Possible values are "all" and languages listed in languages.json.

set(LANGUAGE_SOURCE "en_US"
    CACHE STRING "The source language code of the documentation")
    # Possible values are "en_US".

set(VERSION "rolling"
    CACHE STRING "The current version of the documentation.")
    # Possible values are versions listed in versions.json.

set(VERSION_COMPENDIUM "rolling"
    CACHE STRING "The version of compendium from which the current version will merge translations.")
    # Possible values are NULL and versions listed in versions.json.

set(AUTO_DEPEND ON
    CACHE BOOL "Whether to enable automatic dependency management for specific build targets.")
    # Possible values are "ON/OFF", "TRUE/FALSE", "true/false"...etc.

set(REMOVE_REDUNDANT ON
    CACHE BOOL "Whether to remove redundant files after building documentation.")
    # Possible values are "ON/OFF", "TRUE/FALSE", "true/false"...etc.

set(MODE_OF_UPDATE "NEVER"
    CACHE STRING "The mode of updating .pot/.po files.")
    # Possible values are "NEVER", "COMPARE", and "ALWAYS".
    # (1) NEVER:    Update .pot/.po files only if the current reference is missing.
    # (2) COMPARE:  Update .pot/.po files only if the current reference is different than the latest one.
    # (3) ALWAYS:   Update .pot/.po files even if the current reference is the same as    the latest one.

set(MODE_OF_INSTALL "COMPARE"
    CACHE STRING "The mode of installing the requirements.")
    # Possible values are "COMPARE" and "ALWAYS".
    # (1) COMPARE:  Install the requirements only if the current reference is different than the previous one.
    # (2) ALWAYS:   Install the requirements even if the current reference is the same as    the previous one.

set(SPHINX_BUILDER "html"
    CACHE STRING "The builder for the Sphinx documentation system.")
    # Possible values are "html".
    # See https://www.sphinx-doc.org/en/master/usage/builders/index.html for details.

set(SPHINX_JOB_NUMBER "4"
    CACHE STRING "The number of the build processes in parallel for Sphinx building.")
    # Possible values are "auto", "1", "2",...etc.
    # See https://www.sphinx-doc.org/en/master/man/sphinx-build.html#cmdoption-sphinx-build-j for details.

set(SPHINX_VERBOSE_LEVEL "1"
    CACHE STRING "The verbosity level of the debug logging output for Sphinx building.")
    # Possible values are "0", "1", "2", and "3".
    # See https://www.sphinx-doc.org/en/master/man/sphinx-build.html#cmdoption-sphinx-build-v for details.

set(SPHINX_GETTEXT_COMPACT "0"
    CACHE STRING "The compacting domain style for Sphinx with gettext builder.")
    # Possible values are "1" (compact) or "0" (non-compact).
    # Passed to sphinx-build as -Dgettext_compact=${SPHINX_GETTEXT_COMPACT}.
    # See https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-gettext_compact for details.

set(SPHINX_GETTEXT_TARGETS "index,literal-block,raw"
    CACHE STRING "The additional targets for Sphinx with gettext builder.")
    # Possible values are a comma-separated list of the following targets:
    # "index", "literal-block", "doctest-block", "raw", and "image".
    # Passed to sphinx-build as -Dgettext_additional_targets=${SPHINX_GETTEXT_TARGETS}.
    # See https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-gettext_additional_targets for details.

set(GETTEXT_WRAP_WIDTH 79
    CACHE STRING "The Gettext wrap width for .pot/.po files.")
    # Possible values are positive integers.
    # Passed to msgcat and msgmerge as --width=${GETTEXT_WRAP_WIDTH}.
    # See https://www.gnu.org/software/gettext/manual/html_node/msgcat-Invocation.html
    # and https://www.gnu.org/software/gettext/manual/html_node/msgmerge-Invocation.html for details.

set(BRANCH_OF_WORKTREE "l10n"
    CACHE STRING "The branch of the 'l10n' git worktree.")

set(REMOTE_URL_OF_L10N "https://github.com/localizethedocs/ros2-docs-l10n.git"
    CACHE STRING "The remote URL of the 'l10n' repository.")

set(REMOTE_URL_OF_DOCS "https://github.com/ros2/ros2_documentation.git"
    CACHE STRING "The remote URL of the 'docs' repository.")

set(BASEURL_HREF "file://${PROJECT_SOURCE_DIR}/out/$CACHE{SPHINX_BUILDER}"
    CACHE STRING "The base URL of the deployed documentation.")

set(VERSION_OF_PYTHON "[default]"
    CACHE STRING "The version of Python to install.")

#[============================================================[
# Directory-Related Variables.
#]============================================================]

set(PROJ_SOURCE_DIR                 "${PROJECT_SOURCE_DIR}")
set(PROJ_BINARY_DIR                 "${PROJECT_BINARY_DIR}")
set(PROJ_CMAKE_CUSTOM_DIR           "${PROJ_SOURCE_DIR}/cmake/custom")
set(PROJ_CMAKE_MODULES_DIR          "${PROJ_SOURCE_DIR}/cmake/modules")
set(PROJ_CMAKE_PLUGINS_DIR          "${PROJ_SOURCE_DIR}/cmake/plugins")
set(PROJ_CMAKE_TARGETS_DIR          "${PROJ_SOURCE_DIR}/cmake/targets")
set(PROJ_CONDA_DIR                  "${PROJ_SOURCE_DIR}/.conda")
set(PROJ_OUT_DIR                    "${PROJ_SOURCE_DIR}/out")
set(PROJ_OUT_REPO_DIR               "${PROJ_OUT_DIR}/repo")
set(PROJ_OUT_REPO_DOCS_DIR          "${PROJ_OUT_REPO_DIR}")
set(PROJ_OUT_REPO_DOCS_CONFIG_DIR   "${PROJ_OUT_REPO_DOCS_DIR}")
set(PROJ_OUT_REPO_DOCS_SOURCE_DIR   "${PROJ_OUT_REPO_DOCS_DIR}/source")
set(PROJ_OUT_REPO_DOCS_LOCALE_DIR   "${PROJ_OUT_REPO_DOCS_DIR}/locale")
set(PROJ_OUT_REPO_DOCS_TMPLS_DIR    "${PROJ_OUT_REPO_DOCS_DIR}/source/_templates")
set(PROJ_OUT_REPO_DOCS_EXTNS_DIR    "${PROJ_OUT_REPO_DOCS_DIR}/plugins")
set(PROJ_OUT_BUILDER_DIR            "${PROJ_OUT_DIR}/${SPHINX_BUILDER}")
set(PROJ_L10N_DIR                   "${PROJ_SOURCE_DIR}/l10n")
set(PROJ_L10N_VERSION_DIR           "${PROJ_L10N_DIR}/${VERSION}")
set(PROJ_L10N_VERSION_LOCALE_DIR    "${PROJ_L10N_VERSION_DIR}/locale")
set(PROJ_L10N_VERSION_SINGLE_DIR    "${PROJ_L10N_VERSION_DIR}/.single")
set(PROJ_L10N_VERSION_CROWDIN_DIR   "${PROJ_L10N_VERSION_DIR}/.crowdin")

#[============================================================[
# Configuration Files.
#]============================================================]

set(LANGUAGES_JSON_PATH             "${PROJ_SOURCE_DIR}/languages.json")
set(PREV_PACKAGES_TXT_PATH          "${PROJ_CONDA_DIR}/prev/packages.txt")
set(PREV_REFERENCE_TXT_PATH         "${PROJ_CONDA_DIR}/prev/reference.txt")
set(REFERENCES_JSON_PATH            "${PROJ_L10N_VERSION_DIR}/references.json")
set(STATISTICS_TXT_PATH             "${PROJ_L10N_VERSION_DIR}/statistics.txt")
set(CROWDIN_YML_PATH                "${PROJ_L10N_VERSION_DIR}/crowdin.yml")

#[============================================================[
# Include CMake modules.
#]============================================================]

set(CMAKE_MODULE_PATH   "${PROJ_CMAKE_MODULES_DIR}")
include(LogUtils)
include(GitUtils)
include(JsonUtils)

#[============================================================[
# Prepare TMPLS_TO_SOURCE_DIR  for 'sphinx_build_pot'  target.
# Prepare TMPLS_TO_CONFIG_DIR  for 'sphinx_build_docs' target.
# Prepare LOCALE_TO_SOURCE_DIR for 'sphinx_build_docs' taget.
#]============================================================]

file(RELATIVE_PATH TMPLS_TO_SOURCE_DIR
    "${PROJ_OUT_REPO_DOCS_SOURCE_DIR}"
    "${PROJ_OUT_REPO_DOCS_TMPLS_DIR}")

file(RELATIVE_PATH TMPLS_TO_CONFIG_DIR
    "${PROJ_OUT_REPO_DOCS_CONFIG_DIR}"
    "${PROJ_OUT_REPO_DOCS_TMPLS_DIR}")

file(RELATIVE_PATH LOCALE_TO_SOURCE_DIR
    "${PROJ_OUT_REPO_DOCS_SOURCE_DIR}"
    "${PROJ_OUT_REPO_DOCS_LOCALE_DIR}")

#[============================================================[
# Configure Sphinx verbose arguments based on verbosity level.
#]============================================================]

if (SPHINX_VERBOSE_LEVEL GREATER 0)
    set(SPHINX_VERBOSE_ARGS "-")
    math(EXPR REPEAT_COUNT "${SPHINX_VERBOSE_LEVEL} - 1")
    foreach(INDEX RANGE ${REPEAT_COUNT})
        set(SPHINX_VERBOSE_ARGS "${SPHINX_VERBOSE_ARGS}v")
    endforeach()
    unset(INDEX)
    unset(REPEAT_COUNT)
else()
    set(SPHINX_VERBOSE_ARGS "")
endif()

#[============================================================[
# Read LANGUAGE_LIST from languages.json.
#]============================================================]

file(READ "${LANGUAGES_JSON_PATH}"  LANGUAGES_JSON_CNT)
get_members_of_json_object(
    IN_JSON_OBJECT      "${LANGUAGES_JSON_CNT}"
    OUT_MEMBER_NAMES    LANGUAGE_LIST)

#[============================================================[
# Validate the VERSION cache variable.
# - For "rolling|<distro>|...":
#   - Set VERSION_TYPE    to "branch"
#   - Set BRANCH_NAME     to "${VERSION}"
#]============================================================]

if     (VERSION MATCHES "^(rolling)$"   OR
        VERSION MATCHES "^(kilted)$"    OR
        VERSION MATCHES "^(jazzy)$"     OR
        VERSION MATCHES "^(iron)$"      OR
        VERSION MATCHES "^(humble)$"    OR
        VERSION MATCHES "^(galactic)$"  OR
        VERSION MATCHES "^(foxy)$"      OR
        VERSION MATCHES "^(eloquent)$"  OR
        VERSION MATCHES "^(dashing)$"   OR
        VERSION MATCHES "^(crystal)$")
    set(VERSION_TYPE    "branch")
    set(BRANCH_NAME     "${VERSION}")
else()
    message(FATAL_ERROR "Invalid VERSION value. (${VERSION})")
endif()

#[============================================================[
# Validate the LANGUAGE cache variable.
#]============================================================]

set(LANGUAGE_IS_VALID FALSE)
if (LANGUAGE STREQUAL "all")
    set(LANGUAGE_IS_VALID TRUE)
else()
    foreach(_LANGUAGE ${LANGUAGE_LIST})
        if (_LANGUAGE STREQUAL LANGUAGE)
            set(LANGUAGE_IS_VALID TRUE)
        endif()
    endforeach()
    unset(_LANGUAGE)
endif()
if (NOT LANGUAGE_IS_VALID)
    message(FATAL_ERROR "Invalid LANGUAGE value. (${LANGUAGE})")
endif()

#[============================================================[
# Validate the MODE_OF_UPDATE         cache variable.
# Validate the MODE_OF_INSTALL        cache variable.
# Validate the SPHINX_BUILDER         cache variable.
# Validate the SPHINX_JOB_NUMBER      cache variable.
# Validate the SPHINX_VERBOSE_LEVEL   cache variable.
# Validate the SPHINX_GETTEXT_COMPACT cache variable.
# Validate the SPHINX_GETTEXT_TARGETS cache variable.
# Validate the GETTEXT_WRAP_WIDTH     cache variable.
#]============================================================]

if (NOT MODE_OF_UPDATE MATCHES "^(COMPARE|ALWAYS|NEVER)$")
    message(FATAL_ERROR "Invalid MODE_OF_UPDATE value. (${MODE_OF_UPDATE})")
endif()

if (NOT MODE_OF_INSTALL MATCHES "^(COMPARE|ALWAYS)$")
    message(FATAL_ERROR "Invalid MODE_OF_INSTALL value. (${MODE_OF_INSTALL})")
endif()

if (NOT SPHINX_BUILDER MATCHES "^(html)$")
    message(FATAL_ERROR "Invalid SPHINX_BUILDER value. (${SPHINX_BUILDER})")
endif()

if (NOT SPHINX_JOB_NUMBER MATCHES "^(auto|1|2|3|4|5|6|7|8)$")
    message(FATAL_ERROR "Invalid SPHINX_JOB_NUMBER value. (${SPHINX_JOB_NUMBER})")
endif()

if (NOT SPHINX_VERBOSE_LEVEL MATCHES "^(0|1|2|3)$")
    message(FATAL_ERROR "Invalid SPHINX_VERBOSE_LEVEL value. (${SPHINX_VERBOSE_LEVEL})")
endif()

if (NOT SPHINX_GETTEXT_COMPACT MATCHES "^(0|1)$")
    message(FATAL_ERROR "Invalid SPHINX_GETTEXT_COMPACT value. (${SPHINX_GETTEXT_COMPACT})")
endif()

set(POSSIBLE_TARGETS "index|literal-block|doctest-block|raw|image")
if (NOT SPHINX_GETTEXT_TARGETS MATCHES "^(${POSSIBLE_TARGETS})(,(${POSSIBLE_TARGETS}))*$")
    message(FATAL_ERROR "Invalid SPHINX_GETTEXT_TARGETS value. (${SPHINX_GETTEXT_TARGETS})")
endif()

if (NOT GETTEXT_WRAP_WIDTH MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "Invalid GETTEXT_WRAP_WIDTH value. (${GETTEXT_WRAP_WIDTH})")
endif()

#[============================================================[
# Initialize VERSION_OF_XXX if CACHE{VERSION_OF_XXX} is "[default]".
# Set VERSION_OF_XXX to $CACHE{VERSION_OF_XX} if it's not "[default]".
#]============================================================]

if     (VERSION MATCHES "^(rolling)$"   OR
        VERSION MATCHES "^(kilted)$"    OR
        VERSION MATCHES "^(jazzy)$"     OR
        VERSION MATCHES "^(iron)$"      OR
        VERSION MATCHES "^(humble)$"    OR
        VERSION MATCHES "^(galactic)$"  OR
        VERSION MATCHES "^(eloquent)$"  OR
        VERSION MATCHES "^(dashing)$"   OR
        VERSION MATCHES "^(crystal)$")
    set(VERSION_OF_PYTHON   "3.11")
elseif (VERSION MATCHES "^(foxy)$")
    set(VERSION_OF_PYTHON   "3.8")
endif()
if (NOT "$CACHE{VERSION_OF_PYTHON}" STREQUAL "[default]")
    set(VERSION_OF_PYTHON "$CACHE{VERSION_OF_PYTHON}")
endif()

#[============================================================[
# Check prerequisites.
#]============================================================]

find_package(Git        MODULE QUIET)
find_package(Conda      MODULE QUIET)
find_package(Gettext    MODULE QUIET)
find_package(Crowdin    MODULE QUIET)

#[============================================================[
# Create Git Worktree for 'l10n' branch in ${PROJ_L10N_DIR}.
#]============================================================]

create_git_worktree_for_l10n_branch(
    IN_BRANCH           "${BRANCH_OF_WORKTREE}"
    IN_REMOTE_URL       "${REMOTE_URL_OF_L10N}"
    IN_LOCAL_PATH       "${PROJ_SOURCE_DIR}"
    IN_WORKTREE_PATH    "${PROJ_L10N_DIR}")

#[============================================================[
# Initialize ${REFERENCES_JSON_PATH}.
#]============================================================]

init_references_json_file(
    IN_FILEPATH         "${REFERENCES_JSON_PATH}"
    IN_VERSION_TYPE     "${VERSION_TYPE}"
    IN_VERSION          "${VERSION}"
    IN_INIT_MODE        "language"
    IN_INIT_LIST        "${LANGUAGE_LIST}")

#[============================================================[
# Configure ${CROWDIN_YML_PATH}.
#]============================================================]

file(COPY_FILE
    "${PROJ_CMAKE_CUSTOM_DIR}/crowdin.yml"
    "${CROWDIN_YML_PATH}")

#[============================================================[
# Print Information.
#]============================================================]

message(STATUS "---------- Environment Variables ----------")
message(STATUS "ENV_LANG                        = ${ENV_LANG}")
message(STATUS "ENV_LANGUAGE                    = ${ENV_LANGUAGE}")
message(STATUS "ENV_PYTHONNOUSERSITE            = ${ENV_PYTHONNOUSERSITE}")
message(STATUS "---------- CMP<NNNN> Defaults ----------")
message(STATUS "CMAKE_POLICY_DEFAULT_CMP0144    = ${CMAKE_POLICY_DEFAULT_CMP0144}")
message(STATUS "---------- Constants ----------")
message(STATUS "LATEST_VERSION                  = ${LATEST_VERSION}")
message(STATUS "---------- Cache Variables ----------")
message(STATUS "LANGUAGE                        = ${LANGUAGE}")
message(STATUS "LANGUAGE_SOURCE                 = ${LANGUAGE_SOURCE}")
message(STATUS "VERSION                         = ${VERSION}")
message(STATUS "VERSION_COMPENDIUM              = ${VERSION_COMPENDIUM}")
message(STATUS "AUTO_DEPEND                     = ${AUTO_DEPEND}")
message(STATUS "REMOVE_REDUNDANT                = ${REMOVE_REDUNDANT}")
message(STATUS "MODE_OF_UPDATE                  = ${MODE_OF_UPDATE}")
message(STATUS "MODE_OF_INSTALL                 = ${MODE_OF_INSTALL}")
message(STATUS "SPHINX_BUILDER                  = ${SPHINX_BUILDER}")
message(STATUS "SPHINX_JOB_NUMBER               = ${SPHINX_JOB_NUMBER}")
message(STATUS "SPHINX_VERBOSE_LEVEL            = ${SPHINX_VERBOSE_LEVEL}")
message(STATUS "SPHINX_GETTEXT_COMPACT          = ${SPHINX_GETTEXT_COMPACT}")
message(STATUS "SPHINX_GETTEXT_TARGETS          = ${SPHINX_GETTEXT_TARGETS}")
message(STATUS "GETTEXT_WRAP_WIDTH              = ${GETTEXT_WRAP_WIDTH}")
message(STATUS "BRANCH_OF_WORKTREE              = ${BRANCH_OF_WORKTREE}")
message(STATUS "REMOTE_URL_OF_L10N              = ${REMOTE_URL_OF_L10N}")
message(STATUS "REMOTE_URL_OF_DOCS              = ${REMOTE_URL_OF_DOCS}")
message(STATUS "BASEURL_HREF                    = ${BASEURL_HREF}")
message(STATUS "VERSION_OF_PYTHON               = ${VERSION_OF_PYTHON} (CACHE: $CACHE{VERSION_OF_PYTHON})")
message(STATUS "---------- Directory-Related Variables ----------")
message(STATUS "PROJ_SOURCE_DIR                 = ${PROJ_SOURCE_DIR}")
message(STATUS "PROJ_BINARY_DIR                 = ${PROJ_BINARY_DIR}")
message(STATUS "PROJ_CMAKE_CUSTOM_DIR           = ${PROJ_CMAKE_CUSTOM_DIR}")
message(STATUS "PROJ_CMAKE_MODULES_DIR          = ${PROJ_CMAKE_MODULES_DIR}")
message(STATUS "PROJ_CMAKE_PLUGINS_DIR          = ${PROJ_CMAKE_PLUGINS_DIR}")
message(STATUS "PROJ_CMAKE_TARGETS_DIR          = ${PROJ_CMAKE_TARGETS_DIR}")
message(STATUS "PROJ_CONDA_DIR                  = ${PROJ_CONDA_DIR}")
message(STATUS "PROJ_OUT_DIR                    = ${PROJ_OUT_DIR}")
message(STATUS "PROJ_OUT_REPO_DIR               = ${PROJ_OUT_REPO_DIR}")
message(STATUS "PROJ_OUT_REPO_DOCS_DIR          = ${PROJ_OUT_REPO_DOCS_DIR}")
message(STATUS "PROJ_OUT_REPO_DOCS_CONFIG_DIR   = ${PROJ_OUT_REPO_DOCS_CONFIG_DIR}")
message(STATUS "PROJ_OUT_REPO_DOCS_SOURCE_DIR   = ${PROJ_OUT_REPO_DOCS_SOURCE_DIR}")
message(STATUS "PROJ_OUT_REPO_DOCS_LOCALE_DIR   = ${PROJ_OUT_REPO_DOCS_LOCALE_DIR}")
message(STATUS "PROJ_OUT_REPO_DOCS_TMPLS_DIR    = ${PROJ_OUT_REPO_DOCS_TMPLS_DIR}")
message(STATUS "PROJ_OUT_REPO_DOCS_EXTNS_DIR    = ${PROJ_OUT_REPO_DOCS_EXTNS_DIR}")
message(STATUS "PROJ_OUT_BUILDER_DIR            = ${PROJ_OUT_BUILDER_DIR}")
message(STATUS "PROJ_L10N_DIR                   = ${PROJ_L10N_DIR}")
message(STATUS "PROJ_L10N_VERSION_DIR           = ${PROJ_L10N_VERSION_DIR}")
message(STATUS "PROJ_L10N_VERSION_LOCALE_DIR    = ${PROJ_L10N_VERSION_LOCALE_DIR}")
message(STATUS "PROJ_L10N_VERSION_SINGLE_DIR    = ${PROJ_L10N_VERSION_SINGLE_DIR}")
message(STATUS "PROJ_L10N_VERSION_CROWDIN_DIR   = ${PROJ_L10N_VERSION_CROWDIN_DIR}")
message(STATUS "---------- Configuration Files ----------")
message(STATUS "LANGUAGES_JSON_PATH             = ${LANGUAGES_JSON_PATH}")
message(STATUS "PREV_PACKAGES_TXT_PATH          = ${PREV_PACKAGES_TXT_PATH}")
message(STATUS "PREV_REFERENCE_TXT_PATH         = ${PREV_REFERENCE_TXT_PATH}")
message(STATUS "REFERENCES_JSON_PATH            = ${REFERENCES_JSON_PATH}")
message(STATUS "STATISTICS_TXT_PATH             = ${STATISTICS_TXT_PATH}")
message(STATUS "CROWDIN_YML_PATH                = ${CROWDIN_YML_PATH}")
message(STATUS "---------- Other Variables ----------")
message(STATUS "LOCALE_TO_SOURCE_DIR            = ${LOCALE_TO_SOURCE_DIR}")
message(STATUS "TMPLS_TO_SOURCE_DIR             = ${TMPLS_TO_SOURCE_DIR}")
message(STATUS "TMPLS_TO_CONFIG_DIR             = ${TMPLS_TO_CONFIG_DIR}")
message(STATUS "SPHINX_VERBOSE_ARGS             = ${SPHINX_VERBOSE_ARGS}")
message(STATUS "LANGUAGE_LIST                   = ${LANGUAGE_LIST}")
message(STATUS "VERSION_TYPE                    = ${VERSION_TYPE}")
message(STATUS "BRANCH_NAME                     = ${BRANCH_NAME}")
message(STATUS "TAG_PATTERN                     = ${TAG_PATTERN}")
message(STATUS "TAG_SUFFIX                      = ${TAG_SUFFIX}")
message(STATUS "---------- Prerequisites ----------")
message(STATUS "CMAKE_VERSION                   = ${CMAKE_VERSION}")
message(STATUS "CMAKE_COMMAND                   = ${CMAKE_COMMAND}")
message(STATUS "CMAKE_GENERATOR                 = ${CMAKE_GENERATOR}")
message(STATUS "Git_FOUND                       = ${Git_FOUND}")
message(STATUS "Git_VERSION                     = ${Git_VERSION}")
message(STATUS "Git_EXECUTABLE                  = ${Git_EXECUTABLE}")
message(STATUS "Conda_FOUND                     = ${Conda_FOUND}")
message(STATUS "Conda_VERSION                   = ${Conda_VERSION}")
message(STATUS "Conda_EXECUTABLE                = ${Conda_EXECUTABLE}")
message(STATUS "Gettext_FOUND                   = ${Gettext_FOUND}")
message(STATUS "Gettext_VERSION                 = ${Gettext_VERSION}")
message(STATUS "Gettext_XGETTEXT_EXECUTABLE     = ${Gettext_XGETTEXT_EXECUTABLE}")
message(STATUS "Crowdin_FOUND                   = ${Crowdin_FOUND}")
message(STATUS "Crowdin_VERSION                 = ${Crowdin_VERSION}")
message(STATUS "Crowdin_EXECUTABLE              = ${Crowdin_EXECUTABLE}")
message(STATUS "----------------------------------------")

#[============================================================[
# Escape ';' characters for list variables.
#]============================================================]

string(REPLACE ";" "\\\\;" LANGUAGE_LIST "${LANGUAGE_LIST}")

#[============================================================[
# Environment variables passed to script mode.
#]============================================================]

set(SCRIPT_MODE_ENV
    LANG=${ENV_LANG}
    LANGUAGE=${ENV_LANGUAGE}
    PYTHONNOUSERSITE=${ENV_PYTHONNOUSERSITE})

#[============================================================[
# Cache variables passed to script mode.
#]============================================================]

set(SCRIPT_MODE_CACHE
    # [CMP<NNNN> Defaults]
    -D CMAKE_POLICY_DEFAULT_CMP0144=${CMAKE_POLICY_DEFAULT_CMP0144}
    # [Constants]
    -D LATEST_VERSION=${LATEST_VERSION}
    # [Cache Variables]
    -D LANGUAGE=${LANGUAGE}
    -D LANGUAGE_SOURCE=${LANGUAGE_SOURCE}
    -D VERSION=${VERSION}
    -D VERSION_COMPENDIUM=${VERSION_COMPENDIUM}
    -D AUTO_DEPEND=${AUTO_DEPEND}
    -D REMOVE_REDUNDANT=${REMOVE_REDUNDANT}
    -D MODE_OF_UPDATE=${MODE_OF_UPDATE}
    -D MODE_OF_INSTALL=${MODE_OF_INSTALL}
    -D SPHINX_BUILDER=${SPHINX_BUILDER}
    -D SPHINX_JOB_NUMBER=${SPHINX_JOB_NUMBER}
    -D SPHINX_VERBOSE_LEVEL=${SPHINX_VERBOSE_LEVEL}
    -D SPHINX_GETTEXT_COMPACT=${SPHINX_GETTEXT_COMPACT}
    -D SPHINX_GETTEXT_TARGETS=${SPHINX_GETTEXT_TARGETS}
    -D GETTEXT_WRAP_WIDTH=${GETTEXT_WRAP_WIDTH}
    -D BRANCH_OF_WORKTREE=${BRANCH_OF_WORKTREE}
    -D REMOTE_URL_OF_L10N=${REMOTE_URL_OF_L10N}
    -D REMOTE_URL_OF_DOCS=${REMOTE_URL_OF_DOCS}
    -D BASEURL_HREF=${BASEURL_HREF}
    -D VERSION_OF_PYTHON=${VERSION_OF_PYTHON}
    # [Directory-Related Variables]
    -D PROJ_SOURCE_DIR=${PROJ_SOURCE_DIR}
    -D PROJ_BINARY_DIR=${PROJ_BINARY_DIR}
    -D PROJ_CMAKE_CUSTOM_DIR=${PROJ_CMAKE_CUSTOM_DIR}
    -D PROJ_CMAKE_MODULES_DIR=${PROJ_CMAKE_MODULES_DIR}
    -D PROJ_CMAKE_PLUGINS_DIR=${PROJ_CMAKE_PLUGINS_DIR}
    -D PROJ_CMAKE_TARGETS_DIR=${PROJ_CMAKE_TARGETS_DIR}
    -D PROJ_CONDA_DIR=${PROJ_CONDA_DIR}
    -D PROJ_OUT_DIR=${PROJ_OUT_DIR}
    -D PROJ_OUT_REPO_DIR=${PROJ_OUT_REPO_DIR}
    -D PROJ_OUT_REPO_DOCS_DIR=${PROJ_OUT_REPO_DOCS_DIR}
    -D PROJ_OUT_REPO_DOCS_CONFIG_DIR=${PROJ_OUT_REPO_DOCS_CONFIG_DIR}
    -D PROJ_OUT_REPO_DOCS_SOURCE_DIR=${PROJ_OUT_REPO_DOCS_SOURCE_DIR}
    -D PROJ_OUT_REPO_DOCS_LOCALE_DIR=${PROJ_OUT_REPO_DOCS_LOCALE_DIR}
    -D PROJ_OUT_REPO_DOCS_TMPLS_DIR=${PROJ_OUT_REPO_DOCS_TMPLS_DIR}
    -D PROJ_OUT_REPO_DOCS_EXTNS_DIR=${PROJ_OUT_REPO_DOCS_EXTNS_DIR}
    -D PROJ_OUT_BUILDER_DIR=${PROJ_OUT_BUILDER_DIR}
    -D PROJ_L10N_DIR=${PROJ_L10N_DIR}
    -D PROJ_L10N_VERSION_DIR=${PROJ_L10N_VERSION_DIR}
    -D PROJ_L10N_VERSION_LOCALE_DIR=${PROJ_L10N_VERSION_LOCALE_DIR}
    -D PROJ_L10N_VERSION_SINGLE_DIR=${PROJ_L10N_VERSION_SINGLE_DIR}
    -D PROJ_L10N_VERSION_CROWDIN_DIR=${PROJ_L10N_VERSION_CROWDIN_DIR}
    # [Configuration Files]
    -D LANGUAGES_JSON_PATH=${LANGUAGES_JSON_PATH}
    -D PREV_PACKAGES_TXT_PATH=${PREV_PACKAGES_TXT_PATH}
    -D PREV_REFERENCE_TXT_PATH=${PREV_REFERENCE_TXT_PATH}
    -D REFERENCES_JSON_PATH=${REFERENCES_JSON_PATH}
    -D STATISTICS_TXT_PATH=${STATISTICS_TXT_PATH}
    -D CROWDIN_YML_PATH=${CROWDIN_YML_PATH}
    # [Other Variables]
    -D TMPLS_TO_SOURCE_DIR=${TMPLS_TO_SOURCE_DIR}
    -D TMPLS_TO_CONFIG_DIR=${TMPLS_TO_CONFIG_DIR}
    -D LOCALE_TO_SOURCE_DIR=${LOCALE_TO_SOURCE_DIR}
    -D SPHINX_VERBOSE_ARGS=${SPHINX_VERBOSE_ARGS}
    -D LANGUAGE_LIST=${LANGUAGE_LIST}
    -D VERSION_TYPE=${VERSION_TYPE}
    -D BRANCH_NAME=${BRANCH_NAME}
    -D TAG_PATTERN=${TAG_PATTERN}
    -D TAG_SUFFIX=${TAG_SUFFIX})

#[============================================================[
# Fundamental Targets.
#]============================================================]

add_custom_target(prepare_repositories
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/prepare_repositories.cmake
    VERBATIM)

add_custom_target(install_requirements
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/install_requirements.cmake
    VERBATIM)

add_custom_target(sphinx_update_pot
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/sphinx_update_pot.cmake
    VERBATIM)

add_custom_target(gettext_update_po
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/gettext_update_po.cmake
    VERBATIM)

add_custom_target(sphinx_build_docs ALL
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/sphinx_build_docs.cmake
    VERBATIM)

#[============================================================[
# Optional Targets.
#]============================================================]

add_custom_target(gettext_statistics
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/gettext_statistics.cmake
    VERBATIM)

add_custom_target(gettext_compendium
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/gettext_compendium.cmake
    VERBATIM)

add_custom_target(crowdin_upload_po
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/crowdin_upload_po.cmake
    VERBATIM)

add_custom_target(crowdin_upload_pot
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/crowdin_upload_pot.cmake
    VERBATIM)

add_custom_target(crowdin_download_po
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/crowdin_download_po.cmake
    VERBATIM)

add_custom_target(create_rtd_symlinks
    COMMAND ${CMAKE_COMMAND} -E env ${SCRIPT_MODE_ENV}
            ${CMAKE_COMMAND} ${SCRIPT_MODE_CACHE}
            -P ${PROJ_CMAKE_TARGETS_DIR}/create_rtd_symlinks.cmake
    VERBATIM)

#[============================================================[
# Add Dependencies for Fundamental Targets.
#]============================================================]

if (AUTO_DEPEND)
    add_dependencies(install_requirements   prepare_repositories)
    add_dependencies(sphinx_update_pot      install_requirements)
    add_dependencies(gettext_update_po      sphinx_update_pot)
    add_dependencies(sphinx_build_docs      gettext_update_po)
endif()
