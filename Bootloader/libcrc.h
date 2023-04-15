#ifndef __LIBCRC_H__
#define __LIBCRC_H__
/*****************************************************************************/
#include "stm32f4xx_hal.h"
/*****************************************************************************/
uint16_t crc16Calculate(uint8_t *buf, uint32_t len);//CRC16 ��������
uint16_t crc16CalculateAdd(uint8_t dat);//CRC16 ���������ֽ�
void crc16Clear(void);//��վ�CRC16���
void crc16SetCrcOld(uint16_t old);
/*****************************************************************************/
uint32_t crc32Calculate(uint8_t *buf, uint32_t len);//CRC32 ��������
uint32_t crc32CalculateAdd(uint8_t dat);//CRC32 ���������ֽ�
void crc32Clear(void);//��վ�CRC32���
void crc32SetCrcOld(uint32_t old);
#endif





