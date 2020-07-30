#ifndef __DWLASER_BOOTLOADER_H__
#define __DWLASER_BOOTLOADER_H__
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
#include "deviceConfig.h"
/*****************************************************************************/
extern uint8_t usbReady;//USB DISK����
/*****************************************************************************/
void resetInit(void);
void bootLoadInit(void);
void bootLoadProcess(void);
#endif




