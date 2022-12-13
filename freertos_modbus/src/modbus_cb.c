/**
 * @file modbus_cb.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "modbus_cb.h"
#include "stdbool.h"

extern log(const char* fmt, ...);

// 输入寄存器
#define REG_INPUT_SIZE  32
uint16_t REG_INPUT_BUF[REG_INPUT_SIZE];

// 保持寄存器
#define REG_HOLD_SIZE   32
uint16_t REG_HOLD_BUF[REG_HOLD_SIZE];

// 线圈寄存器
#define REG_COILS_SIZE 16
uint8_t REG_COILS_BUF[REG_COILS_SIZE];

// 离散量
#define REG_DISC_SIZE  8
uint8_t REG_DISC_BUF[REG_DISC_SIZE];

/// CMD4
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    USHORT usRegIndex = usAddress - 1; 

	// 非法检测
	if((usRegIndex + usNRegs) > REG_INPUT_SIZE)
	{
		return MB_ENOREG;
	}

    log(" CMD4, 寄存器输入.");

	// 填充数据
    REG_INPUT_BUF[0] = 0x01;
	REG_INPUT_BUF[1] = 0x02;

    // 循环读取
	while ( usNRegs > 0 ) {
		*pucRegBuffer++ = ( unsigned char )( REG_INPUT_BUF[usRegIndex] >> 8 );
		*pucRegBuffer++ = ( unsigned char )( REG_INPUT_BUF[usRegIndex] & 0xFF );
		usRegIndex++;
		usNRegs--;
	}

	return MB_ENOERR;
}

/// CMD6、3、16
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    USHORT usRegIndex = usAddress - 1;  

	// 非法检测
	if((usRegIndex + usNRegs) > REG_HOLD_SIZE) {
		return MB_ENOREG;
	}

    log(" CMD3,6,16, 保持寄存器读写.");
    
	// 写寄存器
	if (eMode == MB_REG_WRITE) {
		while ( usNRegs > 0 ) {
			uint16_t value;

			value = (pucRegBuffer[0] << 8) | pucRegBuffer[1];

			log("  写寄存器值:%d", value);

			pucRegBuffer += 2;
			usRegIndex++;
			usNRegs--;

        }

    }
    // 读寄存器
	else {

		log("  读寄存器.");
		
		REG_HOLD_BUF[0] = 0x32;
		REG_HOLD_BUF[1] = 0x33;

        while ( usNRegs > 0 ) {
			*pucRegBuffer++ = ( unsigned char )( REG_HOLD_BUF[usRegIndex] >> 8 );
			*pucRegBuffer++ = ( unsigned char )( REG_HOLD_BUF[usRegIndex] & 0xFF );
			usRegIndex++;
			usNRegs--;
		}
	}

	return MB_ENOERR;
}

/// CMD1、5、15
eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{

    USHORT usRegIndex   = usAddress - 1;
	USHORT usCoilGroups = ((usNCoils - 1) / 8 + 1);
	UCHAR  ucStatus     = 0;
	UCHAR  ucBits       = 0;
	UCHAR  ucDisp       = 0;

	// 非法检测
	if ((usRegIndex + usNCoils) > REG_COILS_SIZE) {
		return MB_ENOREG;
	}

    log("  CMD1,5,15, 线圈读写.");

	// 写线圈
	if (eMode == MB_REG_WRITE) {

		while (usCoilGroups--) {

			ucStatus = *pucRegBuffer++;
			ucBits   = 8;

			while((usNCoils) != 0 && (ucBits) != 0) {
				bool flag = ucStatus & 0x01;

				switch (usRegIndex) {
					
                    case 0:
						log(" 线圈0 : %d", flag);//
                    break;

					case 1:
						log(" 线圈1 : %d", flag);
					break;

					default:

					break;

				}

				usRegIndex++;
				ucStatus >>= 1;
				usNCoils--;
				ucBits--;
			}

		}
	}
    // 读线圈
	else {
		
		REG_COILS_BUF[0]  = 1;
		REG_COILS_BUF[1]  = 0;

		while (usCoilGroups--) {
			ucDisp = 0;
			ucBits = 8;
			ucStatus = 0;

			while((usNCoils) != 0 && (ucBits) != 0) {
				ucStatus |= (REG_COILS_BUF[usRegIndex++] << (ucDisp++));
				usNCoils--;
				ucBits--;
			}

			*pucRegBuffer++ = ucStatus;
		}
	}

	return MB_ENOERR;
}

/// CMD4
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    USHORT usRegIndex   = usAddress - 1;
	USHORT usCoilGroups = ((usNDiscrete - 1) / 8 + 1);
	UCHAR  ucStatus     = 0;
	UCHAR  ucBits       = 0;
	UCHAR  ucDisp       = 0;

	// 非法检测
	if ((usRegIndex + usNDiscrete) > REG_DISC_SIZE) {
		return MB_ENOREG;
	}

    log("  CMD4, 离散寄存器写入.");

	// 读离散输入
	while (usCoilGroups--) {
		ucDisp = 0;
		ucBits = 8;
		ucStatus = 0;

		while((usNDiscrete != 0) && (ucBits != 0))
		{
			switch (usRegIndex) {
			case 0:
				ucStatus = 0x10;
				break;
			}

			usRegIndex++;
			ucDisp++;
			usNDiscrete--;
			ucBits--;
		}
		*pucRegBuffer++ = ucStatus;
	}

    	return MB_ENOERR;
}
