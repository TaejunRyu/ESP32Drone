idf_component_register(
                       # We need the dummy source file so that the component
                       # library is not an interface library. This allows to
                       # get the list of include directories from other components
                       # via INCLUDE_DIRECTORIES property later on.
                       SRCS dummy.c)

# Determine compilation flags used for building Mavlink
# Flags inherited from IDF build system and other IDF components:
set(idf_include_directories $<TARGET_PROPERTY:idf::mavlink,INCLUDE_DIRECTORIES>)
set(includes "-I$<JOIN:${idf_include_directories}, -I>")

if(CONFIG_COMPILER_OPTIMIZATION_DEFAULT)
    set(opt_args -DCMAKE_BUILD_TYPE=Debug)
elseif(CONFIG_COMPILER_OPTIMIZATION_SIZE)
    set(opt_args -DCMAKE_BUILD_TYPE=MinSizeRel)
elseif(CONFIG_COMPILER_OPTIMIZATION_PERF)
    set(opt_args -DCMAKE_BUILD_TYPE=Release)
elseif(CONFIG_COMPILER_OPTIMIZATION_NONE)
    set(opt_args -DCMAKE_BUILD_TYPE=Debug)
else()
    message(FATAL_ERROR "Unsupported optimization level")
endif()

include(ExternalProject)

# Build Mavlink in this directory:
set(BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/mavlink-build)

# Attach generated libraries and header files to an interface library:
add_library(mavlink_interface_lib INTERFACE)
set_target_properties(mavlink_interface_lib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${COMPONENT_DIR})
add_dependencies(mavlink_interface_lib mavlink_proj)

# Finally, link the interface library to the component library:
target_link_libraries(${COMPONENT_LIB} INTERFACE mavlink_interface_lib)
