# Template version: 0.0.0
# Sets the version of CMakeLists.txt file.
# Never, under any circumstances, change this line. It is used by
# the module generation script to identify the template version.

#======================== Library configuration ===============================#
# Publicly included directories to provide headers include
set( I2c_PublicIncludeDirs
    
)


# Privately included directories to provide headers include
set( I2c_PrivateIncludeDirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)


# Source files list for current library generation
set( I2c_SourceFiles
    I2c.c
    I2c_Dma.c
    I2c_Isr.c
    I2c_Poll.c
)


# List of public header files provided by the current library
set( I2c_PublicHeaders 
    I2c_Types.h
    I2c_Port.h
)


# List of dependent libraries that are publicly used by the current library
set( I2c_PublicDependentLibs
    Ral_Lib
    Gpio_Lib
    Gpdma_Lib
)


# List of dependent libraries that are privately used by the current library
set( I2c_PrivateDependentLibs
    Rcc_Lib
    Nvic_Lib
)


#========================== Library generation ================================#
# Create library
add_library(I2c_Lib STATIC ${I2c_SourceFiles})


# Include necessary libraries to the target
target_link_libraries( I2c_Lib
    PUBLIC 
        ${I2c_PublicDependentLibs} 
    PRIVATE 
        ${I2c_PrivateDependentLibs}
)


# Set public headers directory path in build folder
set(PUBLIC_HEADERS_DIR ${CMAKE_CURRENT_BINARY_DIR}/PublicHeaders)

# Generates the public headers directory in build folder
file(MAKE_DIRECTORY ${PUBLIC_HEADERS_DIR})

# Process all public headers
foreach(PublicHeader IN LISTS I2c_PublicHeaders)
    
    # Copy (or configure) only the public header there
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${PublicHeader} 
        ${PUBLIC_HEADERS_DIR}/${PublicHeader}
        COPYONLY
    )
    
endforeach()


# Set include directories for the created target
target_include_directories( I2c_Lib
    PUBLIC
        $<BUILD_INTERFACE:${PUBLIC_HEADERS_DIR}>
        $<INSTALL_INTERFACE:include>
        I2c_PublicIncludeDirs
    PRIVATE
        ${I2c_IncludeDirs}
        I2c_PrivateIncludeDirs
)


# Print status message
message(STATUS "I2c_Lib library created")

#========================= Documentation configuration ========================#
if(DOXYGEN_AVAILABLE STREQUAL "ON")
    Doxygen_AddPath("${CMAKE_CURRENT_SOURCE_DIR}/")
endif()
