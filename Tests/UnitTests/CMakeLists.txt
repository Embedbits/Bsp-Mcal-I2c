#======================== Unit tests configuration ============================#
# Unit tests of I2c module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(I2C_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${I2C_DIR}/.." ABSOLUTE)

UnitTesting_Add_Test(
    NAME            I2c
    TEST_SOURCE     Test_I2c.c
    UUT_SOURCES     ${I2C_DIR}/I2c.c
                    ${I2C_DIR}/I2c_Dma.c
                    ${I2C_DIR}/I2c_Isr.c
                    ${I2C_DIR}/I2c_Poll.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Dma/Dma_Port.h
    INCLUDE_DIRS    ${I2C_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Dma
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
)
