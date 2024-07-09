######################################################################
# This is automatically generated file for package mf_sil
#
# It will be used to include package from module to other components
#
# This file will define the required targets and their include paths.
# It will also call a file custom_commands.cmake if existing in the
# current directory.
#
# The following variables are set:
#
# mf_sil_FOUND (if all requested sub-components have been found)
# mf_sil_<component>_FOUND
# mf_sil_PACKAGE_PATH
#
######################################################################

# we want to use at least CMake version 3.5
# when importing this package to other projects
if(CMAKE_VERSION VERSION_LESS 3.5)
    message(FATAL_ERROR "CMake version >= 3.5 requires")
endif()

cmake_policy(PUSH)
cmake_policy(VERSION 3.5)

# define backtrack steps for the 
# root directory of the package

# at the "mf_sil/share/cmake/mf_sil/mf_sil-config.cmake", have to
# go to parent directory

get_filename_component(
    mf_sil_PACKAGE_PATH
    "${CMAKE_CURRENT_LIST_FILE}"
    DIRECTORY
)


# at the "mf_sil/share/cmake/mf_sil", have to
# go to parent directory

get_filename_component(
    mf_sil_PACKAGE_PATH
    "${mf_sil_PACKAGE_PATH}"
    DIRECTORY
)


# at the "mf_sil/share/cmake", have to
# go to parent directory

get_filename_component(
    mf_sil_PACKAGE_PATH
    "${mf_sil_PACKAGE_PATH}"
    DIRECTORY
)


# at the "mf_sil/share", have to
# go to parent directory

get_filename_component(
    mf_sil_PACKAGE_PATH
    "${mf_sil_PACKAGE_PATH}"
    DIRECTORY
)


# at the "mf_sil", have to
# go to parent directory

get_filename_component(
    mf_sil_PACKAGE_PATH
    "${mf_sil_PACKAGE_PATH}"
    DIRECTORY
)


# mark package as found
# (and flip the variable to 0)
# if something is missing
set(mf_sil_FOUND 1)

# if pre_import.cmake is present then we include that one too
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/pre_import.cmake)
    include(${CMAKE_CURRENT_LIST_DIR}/pre_import.cmake)
endif()

# if user has not defined
# any specific component then
# we try import all of them
if (NOT (mf_sil_FIND_COMPONENTS))

    # define the variable (used as list)
    set(mf_sil_FIND_COMPONENTS)
    # find all exported target files
    file(
        GLOB
        __all_existing_target_files
        RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
        "${CMAKE_CURRENT_LIST_DIR}/*/exported-targets.cmake"
    )
    # for each target file get component name 
    foreach(__target_file IN LISTS __all_existing_target_files)
        get_filename_component(
            __component
            "${__target_file}"
            DIRECTORY
        )
        # add component to list of
        # components we want to search
        list(
            APPEND
            mf_sil_FIND_COMPONENTS
            "${__component}"
        )
        # mark component also as mandatory
        # one so backward compatibility is OK
        set(mf_sil_FIND_REQUIRED_${__component} 1)
    endforeach()
    unset(__component)
    unset(__target_file)
    unset(__all_existing_target_files)
endif()

###########################################################
#
# STEP: HANDLE SIBLINGS
#
###########################################################
foreach(__component IN LISTS mf_sil_FIND_COMPONENTS)
    set(__sibling_file "${CMAKE_CURRENT_LIST_DIR}/${__component}/required_siblings.cmake")
    if (EXISTS "${__sibling_file}")
        include("${__sibling_file}")

        # prepend packages to front of package list
        set(
            mf_sil_FIND_COMPONENTS
            ${mf_sil_${__component}_REQUIRED_SIBLINGS}
            ${mf_sil_FIND_COMPONENTS}
        )
        # if our original component is required then we mark
        # also dependency as mandatory
        if (
            (DEFINED "${${mf_sil_FIND_REQUIRED_${__component}}}")
            AND
            (${mf_sil_FIND_REQUIRED_${__component}})
        )
            foreach(__sibling IN LISTS mf_sil_${__component}_REQUIRED_SIBLINGS)
                set(mf_sil_FIND_REQUIRED_${__sibling} 1)
            endforeach()
        endif()
    endif()
    unset(__sibling_file)


endforeach()
unset(__component)

# cleanup the list
list(REMOVE_DUPLICATES mf_sil_FIND_COMPONENTS)

# mark all groups as found first (and mark them failed later)
foreach(__component IN LISTS mf_sil_FIND_COMPONENTS)
    set(mf_sil_${__component}_FOUND 1)
endforeach()
unset(__component)

###########################################################
#
# STEP: HANDLE DEPENDING PACKAGES
#
###########################################################
foreach(__component IN LISTS mf_sil_FIND_COMPONENTS)
    set(__required_pkgs "${CMAKE_CURRENT_LIST_DIR}/${__component}/required_packages.cmake")
    if (EXISTS "${__required_pkgs}")
        include("${__required_pkgs}")
    endif()
    unset(__required_pkgs)
endforeach()
unset(__component)

# include all components which
# are defined in the request
foreach(__BRICKS_GROUP_TO_IMPORT IN LISTS mf_sil_FIND_COMPONENTS)

    # group is marked as not-found already (most likely due
    # to failing package import
    if (${mf_sil_${__BRICKS_GROUP_TO_IMPORT}_FOUND})

        # by default inform that
        # sub-component is found
        # special file which Bricks tries to import
        set(__exported_targets_cmake "${CMAKE_CURRENT_LIST_DIR}/${__BRICKS_GROUP_TO_IMPORT}/exported-targets.cmake")

        if (EXISTS "${__exported_targets_cmake}")
            include("${__exported_targets_cmake}")
        else()
            # no exported targets so we can say
            # file is not included
            set(mf_sil_${__BRICKS_GROUP_TO_IMPORT}_FOUND 0)
        endif()
        unset(__exported_targets_cmake)
    endif()
endforeach()
unset(__BRICKS_GROUP_TO_IMPORT)

# check if we have missed any required
# packages
foreach(__component IN LISTS mf_sil_FIND_COMPONENTS)

    if (
        # check if the component
        # is marked as mandatory one
        ("${mf_sil_FIND_REQUIRED_${__component}}")
        AND
        # check also that component is not found
        (NOT "${mf_sil_${__component}_FOUND}")
    )
        # if required package is missing
        # then we mark entire package as not
        # found
        set(mf_sil_FOUND 0)
        # if there is actual message
        # for failure then set it to error message
        list(APPEND mf_sil_NOT_FOUND_MESSAGE "${mf_sil_${__component}_NOT_FOUND_MESSAGE}")
    endif()
endforeach()
unset(__component)

# if package is marked as not found then
# all components are also marked as not found (all or nothing)
if (NOT mf_sil_FOUND)
    foreach(__component IN LISTS mf_sil_FIND_COMPONENTS)
        set(mf_sil_${__component}_FOUND 0)
    endforeach()
endif()
unset(__component)

# if custom_commands.cmake is present then we include that one too
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/custom_commands.cmake)
    include(${CMAKE_CURRENT_LIST_DIR}/custom_commands.cmake)
endif()

# if post_import.cmake is present then we include that one too
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/post_import.cmake)
    include(${CMAKE_CURRENT_LIST_DIR}/post_import.cmake)
endif()


cmake_policy(POP)
