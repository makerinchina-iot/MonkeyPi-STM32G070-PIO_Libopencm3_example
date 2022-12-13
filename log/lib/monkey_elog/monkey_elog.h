/**
 * @file monkey_elog.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _MONKEY_ELOG_HEAD_H_
#define _MONKEY_ELOG_HEAD_H_

#include "monkey_elog_cfg.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum {
  LOG_LEVEL_DEBUG = 0,
  LOG_LEVEL_INFO = 1,
  LOG_LEVEL_WARN  = 2,
  LOG_LEVEL_ERR   = 3
}elog_level_t;

//keil c51 not supoort __VA__ARGS

#ifdef MONKEY_ELOG_ENABLE
    #define     log_init()          elog_initialize()
    #define     log_d(...)          elog_message(LOG_LEVEL_DEBUG,__VA_ARGS__)
    #define     log_i(...)          elog_message(LOG_LEVEL_DEBUG,__VA_ARGS__)
    #define     log_w(...)          elog_message(LOG_LEVEL_DEBUG,__VA_ARGS__)
    #define     log_e(...)          elog_message(LOG_LEVEL_DEBUG,__VA_ARGS__)
#else 
    #define     elog_init()         ((void)0)
    #define     log_d(...)          ((void)0)      
    #define     log_i(...)          ((void)0)
    #define     log_w(...)          ((void)0)
    #define     log_e(...)          ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif //!_MONKEY_ELOG_HEAD_H_