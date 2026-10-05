#===================== Integration tests configuration ========================#
# Integration tests of I2c module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).

IntegrationTesting_Add_Test(
    NAME            I2c
    TEST_SOURCE     ItTest_I2c.c
    LINK_LIBS       I2c_Lib
    TIMEOUT         60
)
