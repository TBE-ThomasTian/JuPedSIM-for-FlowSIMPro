################################################################################
# Project setup
################################################################################
cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
project(JuPedSim VERSION 2.0.0 LANGUAGES CXX)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(helper_functions)

print_var(PROJECT_VERSION)

# Set default build type to release
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(USE_IPO ON)

check_prefix_path()

################################################################################
# Optional features
################################################################################
set(WERROR OFF CACHE BOOL "Build wth -Werror enabled. Not supported on Windows")
print_var(WERROR)

set(BUILD_TESTS OFF CACHE BOOL "Build tests")
print_var(BUILD_TESTS)

set(BUILD_WITH_ASAN OFF CACHE BOOL
  "Build with address sanitizer support. Linux / macOS only.")
print_var(BUILD_WITH_ASAN)
if(BUILD_WITH_ASAN AND ${CMAKE_SYSTEM} MATCHES "Windows")
    message(FATAL_ERROR "Address sanitizer builds are not supported on Windows")
endif()

set(BUILD_BENCHMARKS OFF CACHE BOOL "Build micro benchmark")
print_var(BUILD_BENCHMARKS)

set(BUILD_XML_CLI OFF CACHE BOOL "Build native XML CLI executable")
print_var(BUILD_XML_CLI)

set(BUILD_INSTALLER OFF CACHE BOOL
  "Create install rules and CPack packages for the FlowSIMPro Evac Add-On. Requires BUILD_XML_CLI")
print_var(BUILD_INSTALLER)
if(BUILD_INSTALLER AND NOT BUILD_XML_CLI)
    message(FATAL_ERROR "BUILD_INSTALLER=ON requires BUILD_XML_CLI=ON: the jupedsim CLI is what gets packaged")
endif()

set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build Python bindings")
print_var(BUILD_PYTHON_BINDINGS)

set(WITH_FORMAT OFF CACHE BOOL "Create format tools")
print_var(WITH_FORMAT)
if(WITH_FORMAT AND ${CMAKE_SYSTEM} MATCHES "Windows")
    message(FATAL_ERROR "Format target not supported on Windows")
endif()

################################################################################
# Compilation flags
################################################################################
# Note: Setting global compile flags via CMAKE_CXX_FLAGS has the drawback that
#       generator expressions cannot be used. This leads to all kind of
#       conditional adding of flags. It is generally preferable to use generator
#       expresssions.
#
# WARNING: Do not break the lines, each option has to be on its own line or
#          CMake will enclose multiple flags in '' which the compiler then
#          treats as a single flag and does not understand.
list(APPEND COMMON_COMPILE_OPTIONS
    $<$<AND:$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>,$<BOOL:${WERROR}>>:-Werror>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wall>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wextra>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-fdiagnostics-color=always>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-fPIC>
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-fno-omit-frame-pointer>
    $<$<CXX_COMPILER_ID:MSVC>:/W2>
    $<$<CXX_COMPILER_ID:MSVC>:/EHsc>
    $<$<CXX_COMPILER_ID:MSVC>:/MP>
    $<$<AND:$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>,$<BOOL:${BUILD_WITH_ASAN}>>:-fno-optimize-sibling-calls>
    $<$<AND:$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>,$<BOOL:${BUILD_WITH_ASAN}>>:-fsanitize=address>
    $<$<AND:$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>,$<BOOL:${BUILD_WITH_ASAN}>>:-shared-libasan>
)

################################################################################
# Dependencies
################################################################################
add_subdirectory(third-party)

################################################################################
# VCS info
################################################################################
find_package(Git QUIET)
find_program(GIT_SCM git DOC "Git version control")
mark_as_advanced(GIT_SCM)
find_file(GITDIR NAMES .git PATHS ${CMAKE_SOURCE_DIR} NO_DEFAULT_PATH)
if (GIT_SCM AND GITDIR)
    # the commit's SHA1, and whether the building workspace was dirty or not
    # describe --match=NeVeRmAtCh --always --tags --abbrev=40 --dirty
    execute_process(COMMAND
        "${GIT_EXECUTABLE}" --no-pager describe --match=Nevermatch --tags --always --dirty
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_SHA1
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    # branch
    execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # the date of the commit
    execute_process(COMMAND
    "${GIT_EXECUTABLE}" log -1 --format=%ad --date=local
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_DATE
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

  execute_process(COMMAND
    "${GIT_EXECUTABLE}" describe --tags --abbrev=0
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_TAG
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

    # the subject of the commit
    execute_process(COMMAND
    "${GIT_EXECUTABLE}" log -1 --format=%s
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_COMMIT_SUBJECT
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    # remove # from subject
    string(REGEX REPLACE "[\#\"]+"
       "" GIT_COMMIT_SUBJECT
       ${GIT_COMMIT_SUBJECT})
else()
    message(STATUS "Not in a git repo")
    set(GIT_SHA1 "UNKNOWN")
    set(GIT_DATE "UNKNOWN")
    set(GIT_COMMIT_SUBJECT "UNKNOWN")
    set(GIT_BRANCH "UNKNOWN")
    set(GIT_TAG "UNKNOWN")
endif()

add_library(git-info INTERFACE)
target_compile_definitions(git-info INTERFACE
    GIT_COMMIT_HASH="${GIT_SHA1}"
    GIT_COMMIT_DATE="${GIT_DATE}"
    GIT_TAG="${GIT_TAG}"
    GIT_COMMIT_SUBJECT="${GIT_COMMIT_SUBJECT}"
    GIT_BRANCH="${GIT_BRANCH}"
)
configure_file(cmake_templates/BuildInfo.hpp.in ${CMAKE_BINARY_DIR}/generated/build_info/BuildInfo.hpp @ONLY)
add_library(build_info INTERFACE)
target_include_directories(build_info INTERFACE
    ${CMAKE_BINARY_DIR}/generated/build_info
)
if(UNIX)
    configure_file(cmake_templates/unix_env.in ${CMAKE_BINARY_DIR}/environment @ONLY)
endif()

################################################################################
# Testing
################################################################################
if(BUILD_TESTS)
    if(BUILD_PYTHON_BINDINGS)
        if(UNIX)
            set(pytest-wrapper-in ${CMAKE_SOURCE_DIR}/cmake_templates/run-systemtests.unix.in)
            set(pytest-wrapper-out ${CMAKE_BINARY_DIR}/run-systemtests)
        else()
            set(pytest-wrapper-in ${CMAKE_SOURCE_DIR}/cmake_templates/run-systemtests.windows.in)
            set(pytest-wrapper-out ${CMAKE_BINARY_DIR}/run-systemtests.cmd)
        endif()
        configure_file(
            ${pytest-wrapper-in}
            ${pytest-wrapper-out}
            @ONLY
        )
        add_custom_target(tests
            DEPENDS systemtests unittests
        )
        add_custom_target(systemtests
            COMMENT "Running system tests"
            COMMAND ${pytest-wrapper-out} -vv -s
                    --basetemp=${CMAKE_BINARY_DIR}/systemtest-out
                    --junit-xml=result-systemtests.xml
            DEPENDS py_jupedsim
        )
    else()
        add_custom_target(tests
            DEPENDS unittests
        )
    endif()
    set(unittests_dependency_list libsimulator-unittests)
    add_custom_target(unittests
        DEPENDS ${unittests_dependency_list}
    )
    add_custom_target(libsimulator-unittests
        COMMENT "Running unit tests"
        COMMAND $<TARGET_FILE:libsimulator-tests>
                --gtest_output=xml:result-libsimulator-unittests.xml
        DEPENDS libsimulator-tests
    )
endif()

if(UNIX)
    set(performancetests-wrapper-in ${CMAKE_SOURCE_DIR}/cmake_templates/run-performancetests.unix.in)
    set(performancetests-wrapper-out ${CMAKE_BINARY_DIR}/run-performancetests)
    configure_file(
        ${performancetests-wrapper-in}
        ${performancetests-wrapper-out}
        @ONLY
    )
endif()

################################################################################
# Add libraries / executables
################################################################################
add_subdirectory(libcommon)
add_subdirectory(libsimulator)
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python_bindings_jupedsim)
endif()

################################################################################
# Code formatting
################################################################################
if(UNIX AND WITH_FORMAT)
    set(clang-format-version 21)
    find_program(CLANG_FORMAT
        NAMES
            clang-format-${clang-format-version}
            clang-format
        REQUIRED
    )
    if(CLANG_FORMAT)
        execute_process(
            COMMAND ${CLANG_FORMAT} --version
            OUTPUT_VARIABLE version_string
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(version_string MATCHES "^.*clang-format version .*")
            string(REGEX REPLACE
                "^.*clang-format version ([.0-9]+).*"
                "\\1"
                version
                "${version_string}"
            )
            if(version MATCHES "^${clang-format-version}.*")
                message(STATUS "Found clang-format ${version}, add format-check and reformat targets")
                set(folders libcommon libsimulator)
                add_custom_target(check-format
                    COMMENT "Checking format with clang-format"
                    COMMAND find ${folders}
                        -name '*.cpp'
                        -o -name '*.c'
                        -o -name '*.h'
                        -o -name '*.hpp' | xargs ${CLANG_FORMAT}
                        -n -Werror --style=file
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                )
                add_custom_target(reformat
                    COMMENT "Reformatting code with clang-format"
                    COMMAND find ${folders}
                        -name '*.cpp'
                        -o -name '*.c'
                        -o -name '*.h'
                        -o -name '*.hpp' | xargs ${CLANG_FORMAT}
                        -i --style=file
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                )
            else ()
                message(FATAL_ERROR "Could not create formatting target, clang-format version ${version} found, but ${clang-format-version} required")
            endif()
        else ()
            message(FATAL_ERROR "Could not create formatting target, clang-format ${version_string} does not yield a version number")
        endif()
    endif()
endif()

################################################################################
# Integration tests
################################################################################
if(BUILD_TESTS AND BUILD_PYTHON_BINDINGS)
  # Find libraries needed by python tests
  find_python_library(pytest)
endif()

################################################################################
# Installation / packaging (FlowSIMPro Evac Add-On)
################################################################################
# FlowSIMPro locates the solver by probing fixed paths, see
# FlowSIMProVulkan/utils/JuPedSimHelper.h::defaultCandidates():
#     Windows: C:/Program Files/FlowSIMProEvacAddOn/bin/jupedsim.exe
#     Linux:   /opt/FlowSIMProEvacAddOn/bin/jupedsim
# The package layout below puts the binary exactly there, so FlowSIMPro finds it
# without the user having to set Settings > Directories > JuPedSim.
if(BUILD_INSTALLER)
    install(TARGETS jupedsim-cli
        RUNTIME DESTINATION bin
    )
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/examples/xml/
        DESTINATION share/jupedsim/examples
        FILES_MATCHING
            PATTERN "*.xml"
            PATTERN "*.md"
    )
    install(FILES
            ${CMAKE_SOURCE_DIR}/LICENSE
            ${CMAKE_SOURCE_DIR}/README.md
        DESTINATION share/jupedsim
    )

    # Year-based release number of the add-on, independent of the upstream
    # JuPedSim version in project(JuPedSim VERSION ...).
    set(FLOWSIMPRO_ADDON_VERSION "2026" CACHE STRING
        "Version stamped into the add-on installer")

    set(CPACK_PACKAGE_NAME "flowsimpro-evac-addon")
    set(CPACK_PACKAGE_VERSION "${FLOWSIMPRO_ADDON_VERSION}")
    # Without these CPack falls back to the project() version (2.0.0) for the
    # component variables, so generators that read them would disagree with
    # CPACK_PACKAGE_VERSION.
    set(CPACK_PACKAGE_VERSION_MAJOR "${FLOWSIMPRO_ADDON_VERSION}")
    set(CPACK_PACKAGE_VERSION_MINOR "0")
    set(CPACK_PACKAGE_VERSION_PATCH "0")
    set(CPACK_PACKAGE_VENDOR "FlowSIM Pro")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
        "FlowSIM Pro Evac Add-On - JuPedSim evacuation solver")
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "FlowSIMProEvacAddOn")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
    set(CPACK_PACKAGE_CHECKSUM SHA256)
    set(CPACK_STRIP_FILES ON)

    if(WIN32)
        set(CPACK_GENERATOR "ZIP")
        find_program(MAKENSIS_EXECUTABLE makensis)
        if(MAKENSIS_EXECUTABLE)
            list(APPEND CPACK_GENERATOR "NSIS")
        else()
            message(STATUS "makensis not found - only a ZIP package will be generated")
        endif()
        # $PROGRAMFILES64 so INSTDIR becomes C:/Program Files/FlowSIMProEvacAddOn
        # and not the (x86) tree.
        set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
        set(CPACK_NSIS_PACKAGE_NAME "FlowSIM Pro Evac Add-On")
        set(CPACK_NSIS_DISPLAY_NAME "FlowSIM Pro Evac Add-On")
        set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
        set(CPACK_NSIS_MODIFY_PATH OFF)
    else()
        set(CPACK_GENERATOR "TGZ;DEB")
        find_program(RPMBUILD_EXECUTABLE rpmbuild)
        if(RPMBUILD_EXECUTABLE)
            list(APPEND CPACK_GENERATOR "RPM")
        else()
            message(STATUS "rpmbuild not found - no RPM package will be generated")
        endif()
        set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/FlowSIMProEvacAddOn")

        # jupedsim links against libstdc++/libm/libgcc_s/libc only; CGAL, Boost,
        # fmt, glm and libdeflate are all linked statically. Verified with
        # `readelf -d`, so the dependency list is written out explicitly instead
        # of paying for a dpkg-shlibdeps run.
        set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FlowSIM Pro <info@flowsimpro.com>")
        set(CPACK_DEBIAN_PACKAGE_SECTION "science")
        set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
        set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
        set(CPACK_DEBIAN_PACKAGE_DEPENDS
            "libc6 (>= 2.38), libgcc-s1 (>= 4.0), libstdc++6 (>= 14)")

        set(CPACK_RPM_PACKAGE_LICENSE "LGPLv3")
        set(CPACK_RPM_PACKAGE_GROUP "Applications/Engineering")
        set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
        set(CPACK_RPM_PACKAGE_AUTOREQ OFF)
        set(CPACK_RPM_PACKAGE_REQUIRES "glibc >= 2.38, libgcc, libstdc++ >= 14")
        # /opt itself belongs to the filesystem package, not to us.
        set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION "/opt")
    endif()

    include(CPack)
endif()
