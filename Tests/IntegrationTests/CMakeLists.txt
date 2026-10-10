#===================== Integration tests configuration ========================#
# Integration tests of I2c module on target (Nucleo-144 / 32F746GDISCOVERY). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Master I2C1 (PB8 SCL / PB9 SDA - Arduino D15 / D14) is the module under test. Slave
# I2C2 (PF1 SCL / PF0 SDA) of Nucleo-144 boards emulates a register device in the
# interrupt of the test set (LL). Wiring PB8 - PF1 and PB9 - PF0, without it (and on
# 32F746GDISCOVERY) the slave tests are ignored.
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
                    Nvic_Lib            # Fault handlers, system reset (ItTarget), slave interrupt
                    I2c_Lib             # Module under test
                    Rcc_Lib             # Kernel clock source verification
    TIMEOUT         90
)
