#include "exdac.h"
/*****************************************************************************/
static void setSYNC(uint8_t dat){
	if(dat){
		HAL_GPIO_WritePin(DA_SYNC_GPIO_Port, DA_SYNC_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(DA_SYNC_GPIO_Port, DA_SYNC_Pin, GPIO_PIN_RESET);
	}
}
static void setCLK(uint8_t dat){
	if(dat){
		HAL_GPIO_WritePin(DA_SCLK_GPIO_Port, DA_SCLK_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(DA_SCLK_GPIO_Port, DA_SCLK_Pin, GPIO_PIN_RESET);
	}
}
static void setDIN(uint8_t dat){
	if(dat){
		HAL_GPIO_WritePin(DA_DIN_GPIO_Port, DA_DIN_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(DA_DIN_GPIO_Port, DA_DIN_Pin, GPIO_PIN_RESET);
	}
}
static void spiWrite(uint32_t dat){//DAC8568 SPI写入
	uint8_t tmp, i;
	setSYNC(TRUE);
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	setCLK(TRUE);
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	setSYNC(FALSE);
	for(i = 0;i < 32;i ++){
		tmp = (uint8_t)(dat >> (31 - i)) & 0x01;
		setDIN(tmp);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		setCLK(FALSE);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		setCLK(TRUE);
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	setSYNC(TRUE);
}

static void dac8568_Init(void){//DAC8568初始化
	uint32_t tmp;
	tmp = 0x07000000;//Software Reset
	spiWrite(tmp);
	tmp = 0x08000001;//Write Sequence for Enabling Internal Reference (Static Mode)
	spiWrite(tmp);
	//覆盖LDAC引脚
	tmp = 0x0600000F;
	spiWrite(tmp);
	//覆盖CLR引脚
	tmp = 0x05000003;
	spiWrite(tmp);
}
static void dac8568_WriteDacRegister(uint8_t ch, uint16_t dat){//写入输入寄存器并更新输出
	uint32_t tmp;
	ch &= 0x0F;
	tmp = 0x03000000;
	tmp |= (uint32_t)((uint32_t)ch << 20);
	tmp |= (uint32_t)((uint32_t)dat << 4);
	spiWrite(tmp);
}

void initChipDac(void){//DAC初始化
	dac8568_Init();
}









