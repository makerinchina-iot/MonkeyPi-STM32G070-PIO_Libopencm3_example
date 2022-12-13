/**
 * @file modbus_cb.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _MODBUS_CB_HEAD_H_
#define _MODBUS_CB_HEAD_H_

#include "mb.h"
#include "mbport.h"

/// CMD4
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs );

/// CMD6、3、16
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode );

/// CMD1、5、15
eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode );

/// CMD4
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete );


#endif //!_MODBUS_CB_HEAD_H_