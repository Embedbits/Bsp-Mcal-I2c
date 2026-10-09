/**
 * \author Mr.Nobody
 * \file BspMain.h
 * \ingroup I2c
 * \brief Entry point of I2c integration test firmware (called by StartUp).
 *
 * StartUp calls BspMain() and gets this header through the interface library
 * BspMain_Lib (created by Tests/IntegrationTests/CMakeLists.txt). BspMain() is
 * implemented by ItTarget_I2c.c.
 */

#ifndef I2C_ITTEST_BSPMAIN_H
#define I2C_ITTEST_BSPMAIN_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* ======================== EXPORTED FUNCTIONS ============================== */

void BspMain( void );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* I2C_ITTEST_BSPMAIN_H */
