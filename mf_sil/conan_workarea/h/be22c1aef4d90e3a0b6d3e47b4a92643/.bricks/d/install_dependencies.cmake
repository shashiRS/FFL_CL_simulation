#######################################################################
#
# This is generated script. Please not not edit!
#
# This CMake script is called during installation of
# Script will parse exported projects and handle implicit dependencies
# between groups and packages. Vital for the proper package oriented
# structuring
#
# Triggered by D:/.bbs_conan/33193c/1/cmake/post/handle_implicit_dependencies.cmake
#
#######################################################################

set(CMAKE_MESSAGE_CONTEXT_SHOW TRUE)
list(APPEND CMAKE_MESSAGE_CONTEXT install_script)

# internal checks for script sanity. These files
# are removed so they must not be carelessly defined
string(TIMESTAMP start_time "%s")

if ("xD:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/intermediate.jsonx" STREQUAL "xx")
    message(FATAL_ERROR "Internal error INTERMEDIATE_JSON failing")
endif()

if ("xD:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/aggregated.jsonx" STREQUAL "xx")
    message(FATAL_ERROR "Internal error AGGREGATED_JSON failing")
endif()

if ("xD:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/code_install.cmakex" STREQUAL "xx")
    message(FATAL_ERROR "Internal error GENERATED_INSTALL_SCRIPT failing")
endif()

#########################################################
#
# CREATE INTERMEDIATE.JSON
#
# Configure package_importer project which calls
# find_package for both used and create packages. Project
# uses mocked version of target-related functions. The
# find_package includes created config files which
# in turn exported-targets.cmake. The exported-targets.cmake
# is then calling the mocked functions which update the
# intermediate.json to every call
#
#########################################################

# FIX: add also parameter for generator toolset
# SD-48666
set(__generator_toolset_argument)
if (NOT ("xx" STREQUAL "xx"))
    set(__generator_toolset_argument "-T" "")
endif()

message(STATUS "CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_LIST_DIR=${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_FIND_PACKAGE_PREFER_CONFIG=")

# create project which imports all dependencing
# packages
execute_process(
    COMMAND
        # some the platforms are not
        # playing nicely so take out the
        # hammer and set compiler with environment
        # variables
        # Related: SD-42636
        ${CMAKE_COMMAND}
        -E env
            "AS="
            "CC=c:/LegacyApp/Microsoft Visual Studio/VC/Tools/MSVC/14.16.27023/bin/Hostx86/x64/cl.exe"
            "CXX=c:/LegacyApp/Microsoft Visual Studio/VC/Tools/MSVC/14.16.27023/bin/Hostx86/x64/cl.exe"

        ${CMAKE_COMMAND}
        "-DCMAKE_BUILD_TYPE=$<CONFIG>"
	# if there are problems with find specific packages, please activate
	# the debug-find-pkg with the list of the relevant packages,
	# the sample is for ecal and protobuf
	# sample: "--debug-find-pkg=protobuf,Protobuf,ecal,eCAL"
        # use languages which are enabled in top-level project
        # also in the dependency calculation project
        "-D__BRICKS_ENABLED_LANGUAGES=C;CXX;RC"
        # define Bricks version since some custom_commands from
        # customer rely on Bricks version to exist
        "-DCIP_BUILD_SYSTEM_VERSION:STRING=4.29.0"
        "-DPARENT_BINARY_DIR:PATH=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643"
        "-DPARENT_CMAKE_PREFIX_PATH:PATH=D:/.bbs_conan/28fe39/1;D:/.bbs_conan/8e961b/1;D:/.bbs_conan/90cb95/1;D:/.bbs_conan/6bb422/1;D:/.bbs_conan/7a2262/1;D:/.bbs_conan/b60548/1;D:/.bbs_conan/7e3b03/1;D:/.bbs_conan/ec48d2/1;D:/.bbs_conan/986724/1;D:/.bbs_conan/388cda/1;D:/.bbs_conan/7143b1/1;D:/.bbs_conan/d32790/1;D:/.bbs_conan/c3aba2/1;D:/.bbs_conan/1e35b5/1;D:/.bbs_conan/5d0049/1;D:/.bbs_conan/c029a0/1;D:/.bbs_conan/57783c/1;D:/.bbs_conan/86a2a6/1;D:/.bbs_conan/ed40e4/1;D:/.bbs_conan/98d06f/1;D:/.bbs_conan/a327d1/1;D:/.bbs_conan/b2dfb9/1;D:/.bbs_conan/9e16b7/1;D:/.bbs_conan/6676fd/1;D:/.bbs_conan/509a7a/1;D:/.bbs_conan/69f023/1;D:/.bbs_conan/4d323d/1;D:/.bbs_conan/f84ef4/1;D:/.bbs_conan/b19af5/1;D:/.bbs_conan/46cf33/1;D:/.bbs_conan/107d41/1;D:/.bbs_conan/110f7b/1;D:/.bbs_conan/327338/1;D:/.bbs_conan/d0a5c3/1;D:/.bbs_conan/570adc/1;D:/.bbs_conan/482479/1;D:/.bbs_conan/0d4361/1;D:/.bbs_conan/28f71c/1;D:/.bbs_conan/563a8a/1;D:/.bbs_conan/61b547/1;D:/.bbs_conan/2f5451/1;D:/.bbs_conan/05c5de/1;D:/.bbs_conan/0b0c71/1;D:/.bbs_conan/17675f/1;D:/.bbs_conan/c1028e/1;D:/.bbs_conan/7321b6/1;D:/.bbs_conan/fb64d9/1;D:/.bbs_conan/416f46/1;D:/.bbs_conan/f7ff21/1;D:/.bbs_conan/ba3e12/1;D:/.bbs_conan/5408c8/1;D:/.bbs_conan/48db9d/1;D:/.bbs_conan/48c906/1;D:/.bbs_conan/016fe0/1;D:/.bbs_conan/65cc1b/1;D:/.bbs_conan/3511c8/1;D:/.bbs_conan/e60f01/1;D:/.bbs_conan/db9c90/1;D:/.bbs_conan/a49663/1"
        "-DPARENT_CMAKE_MODULE_PATH:PATH=D:/.bbs_conan/28fe39/1;D:/.bbs_conan/8e961b/1;D:/.bbs_conan/90cb95/1;D:/.bbs_conan/6bb422/1;D:/.bbs_conan/7a2262/1;D:/.bbs_conan/b60548/1;D:/.bbs_conan/7e3b03/1;D:/.bbs_conan/ec48d2/1;D:/.bbs_conan/986724/1;D:/.bbs_conan/388cda/1;D:/.bbs_conan/7143b1/1;D:/.bbs_conan/d32790/1;D:/.bbs_conan/c3aba2/1;D:/.bbs_conan/1e35b5/1;D:/.bbs_conan/5d0049/1;D:/.bbs_conan/c029a0/1;D:/.bbs_conan/57783c/1;D:/.bbs_conan/86a2a6/1;D:/.bbs_conan/ed40e4/1;D:/.bbs_conan/98d06f/1;D:/.bbs_conan/a327d1/1;D:/.bbs_conan/b2dfb9/1;D:/.bbs_conan/9e16b7/1;D:/.bbs_conan/6676fd/1;D:/.bbs_conan/509a7a/1;D:/.bbs_conan/69f023/1;D:/.bbs_conan/4d323d/1;D:/.bbs_conan/f84ef4/1;D:/.bbs_conan/b19af5/1;D:/.bbs_conan/46cf33/1;D:/.bbs_conan/107d41/1;D:/.bbs_conan/110f7b/1;D:/.bbs_conan/327338/1;D:/.bbs_conan/d0a5c3/1;D:/.bbs_conan/570adc/1;D:/.bbs_conan/482479/1;D:/.bbs_conan/0d4361/1;D:/.bbs_conan/28f71c/1;D:/.bbs_conan/563a8a/1;D:/.bbs_conan/61b547/1;D:/.bbs_conan/2f5451/1;D:/.bbs_conan/05c5de/1;D:/.bbs_conan/0b0c71/1;D:/.bbs_conan/17675f/1;D:/.bbs_conan/c1028e/1;D:/.bbs_conan/7321b6/1;D:/.bbs_conan/fb64d9/1;D:/.bbs_conan/416f46/1;D:/.bbs_conan/f7ff21/1;D:/.bbs_conan/ba3e12/1;D:/.bbs_conan/5408c8/1;D:/.bbs_conan/48db9d/1;D:/.bbs_conan/48c906/1;D:/.bbs_conan/016fe0/1;D:/.bbs_conan/65cc1b/1;D:/.bbs_conan/3511c8/1;D:/.bbs_conan/e60f01/1;D:/.bbs_conan/db9c90/1;D:/.bbs_conan/a49663/1"
	# If enabled, this ensures, that cmake will first search for <Package>-config.cmake
	# and afterwards for Find<Packe> modules.
	# This ensures, that the package configs provided in artefactory will be seen
	# before a generic Find<Package> from cmake.
	# This is essential only for common open source packages, like protobuf.
	"-DCMAKE_FIND_PACKAGE_PREFER_CONFIG="
        "-DPARENT_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
        "-DINTERMEDIATE_OUTPUT_FILE:FILEPATH=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/intermediate.json"
        "-DCMAKE_MAKE_PROGRAM:FILEPATH=c:/LegacyApp/Microsoft Visual Studio/MSBuild/15.0/Bin/MSBuild.exe"
        # tell packages which were found in parent project
        # explicitly
        "-DBRICKS_PACKAGES_FOUND=appl_srv_layer_types;adas_platforms;Java;eco;geoml;mf_common_types;vehicle_types;mf_common;appdemo_parksm_types;mf_parksm_core_types;mf_trjctl_types;mf_trjpla_types;si_types;mf_parksm_core;plp_log;mf_taposd;jsoncpp;mf_trjpla;dmc_types;mf_manager_types;mf_vedodo_types;mf_trjctl;tce_types;mf_vedodo;tce;avga_swc_types;mf_lsca_types;mf_manager;plp_hpsd_types;mf_lsca;appdemo_drvwarnsm_types;appdemo_hmih_types;mf_drvwarnsm_core_types;mf_lvmd_types;mf_memory_parking_types;mf_pdwarnproc_types;mf_whlprotectproc_types;parking_hmi_types;viewcontroller_types;appdemo_hmih;appdemo_parksm;appdemo_drvwarnsm;mf_drvwarnsm_core;mf_pdwarnproc;appdemo_tonh_types;appdemo_tonh;mf_whlprotectproc;us_drv_types;us_processing_types;us_processing;us_em_types;us_em;carmaker_build_dep;si_core;avga_swc;Protobuf;mf_plot;Doxygen"
        # tell to use same system as parent project
        "-DCMAKE_SYSTEM_NAME=Windows"

        # add compiler exclusion/inclusion flags
        # definitions for language C
        "-DCMAKE_C_COMPILER=c:/LegacyApp/Microsoft Visual Studio/VC/Tools/MSVC/14.16.27023/bin/Hostx86/x64/cl.exe"
        "-DCMAKE_C_COMPILER_WORKS:BOOL=TRUE"
        # definitions for language CXX
        "-DCMAKE_CXX_COMPILER=c:/LegacyApp/Microsoft Visual Studio/VC/Tools/MSVC/14.16.27023/bin/Hostx86/x64/cl.exe"
        "-DCMAKE_CXX_COMPILER_WORKS:BOOL=TRUE"
        # definitions for language RC
        "-DCMAKE_RC_COMPILER=C:/Program Files (x86)/Windows Kits/10/bin/10.0.17763.0/x64/rc.exe"
        "-DCMAKE_RC_COMPILER_WORKS:BOOL=TRUE"
        

        # tell to use our toolchain from
        # main project also in dependency
        # calculation project
        "-DCMAKE_TOOLCHAIN_FILE:FILEPATH=D:/.bbs_conan/43c208/1/__system/cip_build_system_platforms/toolchains/conan/conan_toolchain.cmake"
        -G "Visual Studio 15 2017 Win64"
        ${__generator_toolset_argument}
        -B "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/dbuild"
        -S "D:/.bbs_conan/33193c/1/cmake/post/implicit_dependencies/package_importer"
    COMMAND_ECHO STDOUT
    RESULT_VARIABLE res_code
)
if (NOT (res_code EQUAL 0))
    message(WARNING "mf_sil: Cannot create D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/intermediate.json, potentially you should consider to set the variable CMAKE_FIND_PACKAGE_PREFER_CONFIG in your CMakeLists.txt to prefer packages from artefactory before any default installed packages. Alternative you can define own pre_import.cmake which is included before package is imported (for details please see user documentation).")
endif()

#########################################################
#
# CREATE AGGREGATED.JSON
#
# Call aggregate.py with intermediate.json to create
# aggregated.json which is easier for code generator
#
#########################################################

# calculate aggregated.json by
# combining the individual property
# settings to one, more coherent
# data
execute_process(
    COMMAND
        C:/Users/uig44320/cip_config_dir/bricks/4.29.0/windows/Scripts/python.exe
        -u
        "D:/.bbs_conan/33193c/1/cmake/post/implicit_dependencies/aggregate.py"
        "--intermediate-json=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/intermediate.json"
        "--aggregated-json=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/aggregated.json"
    COMMAND_ECHO STDOUT
    RESULT_VARIABLE res_code
)
if (NOT (res_code EQUAL 0))
    message(FATAL_ERROR "mf_sil: Cannot create D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/aggregated.json")
endif()

#########################################################
#
# GENERATE CMAKE CODE FOR TRANSITIVE DEPENDENCIES
#
# Call aggregate.py with intermediate.json to create
# aggregated.json which is easier for code generator
#
#########################################################

# use aggregated.json to calculate implicit
# dependencies between packages
execute_process(
    COMMAND
        C:/Users/uig44320/cip_config_dir/bricks/4.29.0/windows/Scripts/python.exe
        -u
        "D:/.bbs_conan/33193c/1/cmake/post/implicit_dependencies/generate.py"
        "--aggregated-json=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/aggregated.json"
        "--workdir=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/workdir"
        "--install-script=D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/code_install.cmake"
    COMMAND_ECHO STDOUT
    RESULT_VARIABLE res_code
)
if (NOT (res_code EQUAL 0))
    message(FATAL_ERROR "mf_sil: Cannot create D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/code_install.cmake")
endif()

string(TIMESTAMP end_time "%s")
math(EXPR elapsed "${end_time}-${start_time}")
message(STATUS "Calculation took ${elapsed}")

#########################################################
#
# INSTALL GENERATED FILES
#
# Code generation also updates list of files which
# should be installed for transient dependencies. It is
# enough to simply include the file and install commands
# inside are triggered
#
#########################################################
include("D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/code_install.cmake")

# install also generated JSON
file(
    INSTALL "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/aggregated.json"
    # we are inside installation script so we do
    # have to define the install prefix
    # normally (in the CMakeLists.txt) one should not
    # use absolute paths
    DESTINATION ${CMAKE_INSTALL_PREFIX}/.bricks
)

list(POP_BACK CMAKE_MESSAGE_CONTEXT)
