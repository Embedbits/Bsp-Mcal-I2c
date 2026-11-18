#===================== Integration tests configuration ========================#
# Integration tests of I2c module on target (NUCLEO-G474RE, NUCLEO-G431RB, NUCLEO-G491RE).
# Registered in module CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Master I2C3 (PC8 SCL / PC9 SDA) and slave I2C1 (PA15 SCL / PB7 SDA) are both peripherals
# of the module under test. Wiring PC8 (CN10-2) - PA15 (CN7-17) and PC9 (CN10-1) - PB7
# (CN7-21), same on all boards: the board fixture feature I2C_LOOP of the inventory, the
# master + slave tests are ignored on a board with a fixture without it.
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

IntegrationTesting_Add_Test(
    NAME            I2c
    TEST_SOURCE     ItTest_I2c.c
    SOURCES         ItTarget_I2c.c      # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget)
                    I2c_Lib             # Module under test
                    Rcc_Lib             # Kernel clock source verification
    TIMEOUT         90
)
