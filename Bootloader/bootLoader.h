#ifndef __DWLASER_BOOTLOADER_H__
#define __DWLASER_BOOTLOADER_H__
/*****************************************************************************/
#define DEBUG_BOOTLOADER					0
#define DEBUG_NOBEEP						1
#define BOARD_STM32F413H_DISCO				0//STM32F413H 官方开发板
#define BOARD_STM32F407_DEV					1//STM32F407 淘宝开发板
#define BOARD_STM32F413RH_DWLASER			0//STM32F413RH 产品板
/*****************************************************************************/
#include "stm32f4xx_hal.h"
/*****************************************************************************/
#include "cmsis_armcc.h"
/*****************************************************************************/
#include "usbh_platform.h"
#include "usbh_core.h"
#include "usbh_msc.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "flash_if.h"
#include "usart.h"
/*****************************************************************************/
#include "crc16.h"
/*****************************************************************************/
extern uint8_t usbReady;//USB DISK就绪
/*****************************************************************************/
void resetInit(void);
void bootLoadInit(void);
void bootLoadProcess(void);
#endif




