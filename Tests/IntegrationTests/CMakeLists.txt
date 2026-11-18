#===================== Integration tests configuration ========================#
# Integration tests of I2c module on target (Nucleo-144 boards). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Master I2C1 (PB8 SCL / PB9 SDA - Arduino D15 / D14) is wired to the slave I2C peripheral
# of the same MCU (feature I2C_LOOP of the board inventory): I2C2 PB10 / PB11 on NUCLEO-H7A3ZI-Q
# and NUCLEO-H7S3L8, I2C4 PB6 / PB7 on NUCLEO-H723ZG and NUCLEO-H755ZI-Q. The slave of the
# I2c module (register device) is the counterpart of the master tests; a board without the
# wiring ignores the master + slave tests.
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_I2c.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_I2c.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")

    # STM32H7R / STM32H7S (NUCLEO-H7S3L8)
    # The firmware needs 113 % of the 64 KB flash at -O0 and does not link, optimization is required.
    if(CMAKE_C_FLAGS_INTEGRATIONTEST MATCHES "-O0")
        message(WARNING "Integration test IT_I2c of STM32H7R / STM32H7S does not fit into 64 KB flash at -O0 - configure with -DINTEGRATION_TEST_OPTIMIZATION=-Os")
    endif()

    IntegrationTesting_Add_Test(
        NAME            I2c
        TEST_SOURCE     ItTest_I2c_H7RS.c
        SOURCES         ItTarget_I2c.c      # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        I2c_Lib
        TIMEOUT         60
    )

else()

    # Classic STM32H7 (NUCLEO-H723ZG, NUCLEO-H7A3ZI-Q, NUCLEO-H755ZI-Q)
    IntegrationTesting_Add_Test(
        NAME            I2c
        TEST_SOURCE     ItTest_I2c.c
        SOURCES         ItTarget_I2c.c      # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        I2c_Lib             # Module under test
                        Rcc_Lib             # Kernel clock source verification
        TIMEOUT         120
    )

endif()
