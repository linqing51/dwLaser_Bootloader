#include "bootLoader.h"
/*****************************************************************************/
//SECTOR0->16K:BOOTLOADER
//SECTOR1->16K:BOOTLOADER
//SECTOR2->16K:BOOTLOADER
//SECTOR3->16K:BOOTLOADER
/*****************************************************************************/
#define BT_STATE_IDLE						0//����
#define BT_STATE_LOAD_FWINFO				1//EPROM����̼���Ϣ
#define BT_STATE_USBHOST_INIT				2//FATFS ��ʼ��
#define BT_STATE_WAIT_UDISK					3//�ȴ�USB DISK����
#define BT_STATE_READ_CFG					4//��ȡ�����ļ�
#define BT_STATE_UPDATE_MCU_BOT				5//����BOOTLOAD
#define BT_STATE_UPDATE_MCU_APP				6//���µ�Ƭ��Ӧ�ù̼�
#define BT_STATE_UPDATE_LCD_APP				7//������ĻӦ�ù̼�
#define BT_STATE_UPDATE_BOTH_APP			8//���µ�Ƭ�������̼�
#define BT_STATE_UPDATE_EPROM				9//����UDISK->EPROM
#define BT_STATE_DUMP_EPROM					10//����ȫ��EPROM��UDISK
#define BT_STATE_CLEAT_ALL					11//���FLASH��EPROMȫ��
#define BT_STATE_RESET						90//����
#define BT_STATE_RUN_APP					99//��ת��APPӦ�ó���
/*****************************************************************************/
#define BT_FAIL_READ_CFG					'0'//��U�̶�ȡCFGʧ��
#define BT_FAIL_READ_LMCU_APP				'1'//��U�̶�ȡMCU APPʧ��
#define BT_FAIL_READ_LLCD_APP				'2'//��U�̶�ȡLCD APPʧ��
#define BT_FAIL_READ_LEROM_BIN				'3'//��U�̶�ȡEPROM BINʧ��
#define BT_FAIL_WRITE_SEROM_BIN				'4'//��U��д��EPROM BINʧ��
#define BT_FAIL_ERASE_MCU_APP				'5'//����FLASHʧ��
#define BT_FAIL_READ_EPROM					'6'//��ȡEPROMʧ��
#define BT_FAIL_WRITE_EPROM					'7'//д��EPROMʧ��
#define BT_FAIL_LMCU_APP_CHECK				'8'//ld_mcu.bin CRC������
#define BT_FAIL_LMCU_BOT_CHECK				'9'//ld_bot.bin CRC������
#define BT_FAIL_LLCD_APP_CHECK				'A'//ld_lcd.bin CRC������
#define BT_FAIL_CHECKSUM_MCU_APP_FLASH		'B'//У�� mcu app ����
#define BT_FAIL_WRITE_MCU_APP_FLASH			'C'//дmcu app flash����
#define BT_FAIL_LCD_NOT_RESPOND				'D'//LCD����ͨ�ų�ʱ�����
#define BT_FAIL_LCD_DOWNLOAD				'E'//LCD����ʧ��
#define BT_FAIL_VECTOR_TABLE_INVALID		'F'//APP ���������
#define BT_FAIL_CHECK_BLANK					'G'//FLASH��մ���
#define BT_DONE_CLEAR_ALL					'H'//FLASH��EPROM������
#define BT_DONE_UPDATE_EPROM				'I'//����EPROM���
#define BT_DONE_DUMP_EPROM					'J'//����EPROM���

//#define BT_FAIL_READ_LEROM_BIN
/*****************************************************************************/
#define GDDC_LCD_NORMAL_BAUDRATE			(115200UL)//LCD����������������
#define GDDC_LCD_UPDATE_BAUDRATE			(115200UL)//LCD���������²�����
/*****************************************************************************/
#define GDDC_UART_HANDLE					huart5
#define GDDC_UART_IRQ						USART1_IRQn
#define GDDC_RX_BUF_SIZE					64
#define GDDC_TX_BUF_SIZE					(2048 + 4)
#define GDDC_HEX 							0
#define GDDC_DEC 							1 
#define GDDC_CHR 							2
#define GDDC_RX_TIMEOUT						0xFFFF
#define GDDC_TX_TIMEOUT						0xFFFF
#define GDDC_RETRY_TIMES					10//�������Դ���
#define GDDC_UPDATE_BAUDRATE				115200//���ı䲨����
/*****************************************************************************/
#define MORSECODE_SPACE_TIME				1000
#define MORSECODE_LONG_TIME					750
#define MORSECODE_SHORT_TIME				150
/*****************************************************************************/
#define SET_TEC(b)							HAL_GPIO_WritePin(TEC_OUT_GPIO_Port, TEC_OUT_Pin, b)
#define FLIP_TEC()							HAL_GPIO_TogglePin(TEC_OUT_GPIO_Port, TEC_OUT_Pin)

#define SET_FAN5V(b)						HAL_GPIO_WritePin(FAN5V_OUT_GPIO_Port, FAN5V_OUT_Pin, b)
#define FLIP_FAN5V()						HAL_GPIO_TogglePin(FAN5V_OUT_GPIO_Port, FAN5V_OUT_Pin)

#define SET_FAN24V(b)						HAL_GPIO_WritePin(FAN24V_OUT_GPIO_Port, FAN24V_OUT_Pin, b)
#define FLIP_FAN24V()						HAL_GPIO_TogglePin(FAN24V_OUT_GPIO_Port, FAN24V_OUT_Pin)

#define SET_LCD(b)							HAL_GPIO_WritePin(LCD_OUT_GPIO_Port, LCD_OUT_Pin, b)
#define FLIP_LCD()							HAL_GPIO_WritePin(LCD_OUT_GPIO_Port, LCD_OUT_Pin, b)

#define SET_LPA0(b)							HAL_GPIO_WritePin(LPA_PWM0_GPIO_Port, LPA_PWM0_Pin, b)
#define FLIP_LPA0()							HAL_GPIO_TogglePin(LPA_PWM0_GPIO_Port, LPA_PWM0_Pin)

#define SET_LPA1(b)							HAL_GPIO_WritePin(LPA_PWM1_GPIO_Port, LPA_PWM1_Pin, b)
#define FLIP_LPA1()							HAL_GPIO_TogglePin(LPA_PWM1_GPIO_Port, LPA_PWM1_Pin)

#define SET_LPB0(b)							HAL_GPIO_WritePin(LPB_PWM0_GPIO_Port, LPB_PWM0_Pin, b)
#define FLIP_LPB0()							HAL_GPIO_TogglePin(LPB_PWM0_GPIO_Port, LPB_PWM0_Pin)

#define SET_LPB1(b)							HAL_GPIO_WritePin(LPB_PWM1_GPIO_Port, LPB_PWM1_Pin, b)
#define FLIP_LPB1()							HAL_GPIO_TogglePin(LPB_PWM1_GPIO_Port, LPB_PWM1_Pin)

#define SET_LPC0(b)							HAL_GPIO_WritePin(LPC_PWM0_GPIO_Port, LPC_PWM0_Pin, b)
#define FLIP_LPC0()							HAL_GPIO_TogglePin(LPC_PWM0_GPIO_Port, LPC_PWM0_Pin)

#define SET_RED(b)							HAL_GPIO_WritePin(RED_OUT_GPIO_Port, RED_OUT_Pin, b)
#define FLIP_RED()							HAL_GPIO_TogglePin(RED_OUT_GPIO_Port, RED_OUT_Pin)

#define SET_GREEN(b)						HAL_GPIO_WritePin(GREEN_OUT_GPIO_Port, GREEN_OUT_Pin, b)
#define FLIP_GREEN()						HAL_GPIO_TogglePin(GREEN_OUT_GPIO_Port, GREEN_OUT_Pin)

#define SET_BLUE(b)							HAL_GPIO_WritePin(BLUE_OUT_GPIO_Port, BLUE_OUT_Pin, b)
#define FLIP_BLUE()							HAL_GPIO_TogglePin(BLUE_OUT_GPIO_Port, BLUE_OUT_Pin)

#define SET_AIM(b)							HAL_GPIO_WritePin(AIM_OUT_GPIO_Port, AIM_OUT_Pin, b)
#define FLIP_AIM()							HAL_GPIO_TogglePin(AIM_OUT_GPIO_Port, AIM_OUT_Pin)
/*****************************************************************************/
typedef enum {
	CLEAR_EPROM_ALL 			= 0x01,
	CLEAR_EPROM_NVRAM			= 0x02,
	CLEAR_EPROM_FDRAM			= 0x03,
	CLEAR_EPROM_FIRMWARE_INFO	= 0x04,
	CLEAR_EPROM_DEVICE_CONFIG	= 0x05,
	CLEAR_EPROM_LOG_INFO		= 0x06,
}clarmEpromCmd_t;
/*****************************************************************************/
static uint32_t TmpReadSize = 0x00;
static uint32_t RamAddress = 0x00;
static __IO uint32_t LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
static uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};//�ļ���д����
/*****************************************************************************/
static uint8_t gddcRxBuf[GDDC_RX_BUF_SIZE];//��Ļ���ڽ��ջ�����
static uint8_t gddcTxBuf[GDDC_TX_BUF_SIZE];//��Ļ���ڷ��ͻ�����
/*****************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern USBH_HandleTypeDef hUsbHostFS;
/*****************************************************************************/
FRESULT retUsbH;
FATFS	USBH_fatfs;
/*****************************************************************************/
FIL LogFile;//FATFS File Object ��¼��Ϣ
FIL CfgFile;//FATFS File Object ���������Ϣ
FIL McuFile;//FATFS File Object ��Ƭ���̼�
FIL LcdFile;//FATFS File Object ��Ļ�̼�
FIL BotFile;//FATFS File Object BOOTLOAD�̼�
FIL SepromFile;//FATFS File Object EPROM->UDISK
FIL LepromFile;//FATFS File Object UDISK->EPROM
/*****************************************************************************/
DIR	FileDir;//FATFS �ļ�Ŀ¼
FILINFO FileInfo;//FATFS �ļ���Ϣ
static uint8_t bootLoadState;
uint8_t usbReady;//USB DISK����
int32_t releaseTime0, releaseTime1, overTime, releaseCounter;
uint32_t JumpAddress;
pFunction Jump_To_Application;
static uint32_t oldcrc32;
const uint32_t crc32Tab[] = { /* CRC polynomial 0xedb88320 */
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};
/*****************************************************************************/
static void bootLoadFailHandler(uint8_t ftype);//�������ϳ���
static uint32_t updateMcuApp(void);
static uint32_t updateLcdApp(void);
static void checkBlank(uint32_t adr, uint32_t size);
static void DBGU_Printk(uint8_t *buffer);
static void DBGU_Printk_num(uint8_t *buffer, uint16_t datanum);
static void dp_display_text(uint8_t *text);
static void dp_display_text_num(uint8_t *text,uint16_t datanum);	
static void dp_display_value(uint32_t value,int descriptive);
static void dp_display_array(uint8_t *value,int bytes, int descriptive);	
static void beepDiag(uint8_t diag);
static void SystemClock_Reset(void);
static void UsbGpioReset(void);
static void clearFlash(void);
static uint32_t getOriginAppCrc(void);
static uint32_t getNewMcuAppCrc(void);
static uint32_t getNewLcdAppCrc(void);
static void clearEprom(clarmEpromCmd_t cmd);//���EPROM����
static void updateEprom(void);
static void dumpEprom(void);
/*****************************************************************************/
static uint32_t crc32Calculate(uint8_t *buf, uint32_t len);//CRC32 ��������
static uint32_t crc32CalculateAdd(uint8_t dat);//CRC32���������ֽ�
static void crc32Clear(void);//CRC32�������ֵ
static void crc32SetCrcOld(uint32_t old);//CRC32���ü���ֵ
/*****************************************************************************/
static void softDelayMs(uint16_t ms);
/*****************************************************************************/
static HAL_StatusTypeDef epromReadByte(uint16_t ReadAddr, uint8_t *rdat);//��AT24CXXָ����ַ����һ������
static HAL_StatusTypeDef epromWriteByte(uint16_t WriteAddr, uint8_t wdat);//��AT24CXXָ����ַд��8λ����
static HAL_StatusTypeDef epromRead(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
static HAL_StatusTypeDef epromWrite(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);
/*****************************************************************************/
//HAL ��ʼ��
static void UsbGpioReset(void){//ģ��USB�β嶯�����ر�VBUS����
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	/*Configure GPIO pin : PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);                                            
	softDelayMs(100);
	//�Ȱ�PA12���������ߣ�����D+ģ��USB�İβ嶯��   
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	softDelayMs(100);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	softDelayMs(200);
	HAL_GPIO_DeInit(GPIOG, GPIO_PIN_12);
	__HAL_RCC_GPIOG_CLK_DISABLE();	
	__HAL_RCC_USB_OTG_FS_CLK_DISABLE();//�ر�USBʱ��
	HAL_NVIC_DisableIRQ(OTG_FS_IRQn);//�ر�USB �ж�
	HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);//��� USB �жϱ�־
}
static void SystemClock_Reset(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	__HAL_RCC_BACKUPRESET_RELEASE();
	__HAL_RCC_BACKUPRESET_FORCE();
	__HAL_RCC_PLL_DISABLE();
	__HAL_RCC_HSI_DISABLE();
	/** Configure the main internal regulator output voltage */
	__HAL_RCC_PWR_CLK_DISABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK){
		Error_Handler();
	}
}
void resetInit(void){//��λ���ʼ��
	HAL_DeInit();
	//��λRCCʱ��
	SystemClock_Reset();
	UsbGpioReset();
	__enable_irq();
}
/*****************************************************************************/
//CRC32�������
static uint32_t crc32Calculate(uint8_t *buf, uint32_t len){//CRC32 ��������
    uint32_t i;  
    for (i = 0; i < len; i++){  
       oldcrc32 = crc32Tab[(oldcrc32 ^ buf[i]) & 0xff] ^ (oldcrc32 >> 8);  
    }  
	return (oldcrc32 ^ 0xFFFFFFFF);  
}
static uint32_t crc32CalculateAdd(uint8_t dat){//CRC32���������ֽ�
	oldcrc32 = crc32Tab[(oldcrc32 ^ dat) & 0xff] ^ (oldcrc32 >> 8);
	return (oldcrc32 ^ 0xFFFFFFFF);
}
static void crc32Clear(void){//CRC32�������ֵ
	oldcrc32 = 0xFFFFFFFF;
}
static void crc32SetCrcOld(uint32_t old){//CRC32���ü���ֵ
	oldcrc32 = old;
}
static void softDelayMs(uint16_t ms){
	uint32_t i;
	for(i = 0;i < 1000;i ++){
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
}
/******************************************************************************/
void bootLoadInit(void){//���������ʼ��
	SET_FAN5V(GPIO_PIN_SET);//��5V����
	SET_FAN24V(GPIO_PIN_SET);//��24V����
	SET_LCD(GPIO_PIN_SET);//��LCD����
	SET_TEC(GPIO_PIN_RESET);//�ر�����
	SET_LPA0(GPIO_PIN_RESET);//�ر����м���
	SET_LPA1(GPIO_PIN_RESET);
	SET_LPB0(GPIO_PIN_RESET);
	SET_LPB1(GPIO_PIN_RESET);
	SET_LPC0(GPIO_PIN_RESET);
	//
	SET_RED(GPIO_PIN_RESET);//����R LED����
	SET_GREEN(GPIO_PIN_RESET);//����G LED����
	SET_BLUE(GPIO_PIN_RESET);//����B LED����
	SET_AIM(GPIO_PIN_RESET);
	//R
	SET_RED(GPIO_PIN_SET);//����R LED����
	SET_GREEN(GPIO_PIN_RESET);//����G LED����
	SET_BLUE(GPIO_PIN_RESET);//����B LED����
	HAL_Delay(200);
	//G
	SET_BLUE(GPIO_PIN_RESET);//����B LED����
	SET_RED(GPIO_PIN_RESET);//����R LED����
	SET_GREEN(GPIO_PIN_SET);//����G LED����
	HAL_Delay(200);
	//B
	SET_BLUE(GPIO_PIN_SET);//����B LED����
	SET_RED(GPIO_PIN_RESET);//����R LED����
	SET_GREEN(GPIO_PIN_RESET);//����G LED����
	HAL_Delay(200);
	SET_BLUE(GPIO_PIN_RESET);//����B LED����
	SET_RED(GPIO_PIN_RESET);//����R LED����
	SET_GREEN(GPIO_PIN_SET);//����G LED����
	
	overTime = HAL_GetTick() + CONFIG_JUMP_DELAY;
	releaseTime0 = 0;
	releaseTime1 = 0;
	usbReady = FALSE;
	bootLoadState = BT_STATE_IDLE; 
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");   
	//��ʾ����IO״̬
	if(HAL_GPIO_ReadPin(PWR_KEY_GPIO_Port, PWR_KEY_Pin) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->PWR_ON        = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->PWR_ON        = Close!\n");
	}
	if(HAL_GPIO_ReadPin(FSWITCH_NC_GPIO_Port, FSWITCH_NC_Pin) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->FSWITCH_NC    = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->FSWITCH_NC    = Close!\n");
	}
	if(HAL_GPIO_ReadPin(FSWITCH_NO_GPIO_Port, FSWITCH_NO_Pin) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->FSWITCH_NO    = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->FSWITCH_NO    = Close!\n");
	}
	if(HAL_GPIO_ReadPin(ESTOP_IN_GPIO_Port, ESTOP_IN_Pin) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->ESTOP         = Close!\n");
	}
	else{
		printf("Bootloader:INPUT->ESTOP         = Open!\n");
	}
	if(HAL_GPIO_ReadPin(INTLOCK_IN_GPIO_Port, INTLOCK_IN_Pin) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->INTLOCK       = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->INTLOCK       = Close!\n");
	}
	if(HAL_GPIO_ReadPin(PM_ALARM_GPIO_Port, PM_ALARM_Pin) == GPIO_PIN_SET){//PM_ALARM
		printf("Bootloader:INPUT->PM_ALARM      = High!\n");
	}
	else{
		printf("Bootloader:INPUT->PM_ALARM      = Low!\n");
	}
	//��ʾ���IO״̬
	if(HAL_GPIO_ReadPin(LPA_PWM0_GPIO_Port, LPA_PWM0_Pin) == GPIO_PIN_SET){//LPA_PWM0
		printf("Bootloader:OUTPUT->LPA_PWM0     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LPA_PWM0     = Low!\n");
	}
	if(HAL_GPIO_ReadPin(LPA_PWM1_GPIO_Port, LPA_PWM1_Pin) == GPIO_PIN_SET){//LPA_PWM1
		printf("Bootloader:OUTPUT->LPA_PWM1     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LPA_PWM1     = Low!\n");
	}
	if(HAL_GPIO_ReadPin(LPB_PWM0_GPIO_Port, LPB_PWM0_Pin) == GPIO_PIN_SET){//LPB_PWM0
		printf("Bootloader:OUTPUT->LPB_PWM0     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LPB_PWM0     = Low!\n");
	}
	if(HAL_GPIO_ReadPin(LPA_PWM1_GPIO_Port, LPA_PWM1_Pin) == GPIO_PIN_SET){//LPB_PWM1
		printf("Bootloader:OUTPUT->LPA_PWM1     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LPA_PWM1     = Low!\n");
	}
	if(HAL_GPIO_ReadPin(LPC_PWM0_GPIO_Port, LPC_PWM0_Pin) == GPIO_PIN_SET){//LPC_PWM0
		printf("Bootloader:OUTPUT->LPC_PWM0     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LPC_PWM0     = Low!\n");
	}
	if(HAL_GPIO_ReadPin(FAN5V_OUT_GPIO_Port, FAN5V_OUT_Pin) == GPIO_PIN_SET){//FAN
		printf("Bootloader:OUTPUT->FAN5V_OUT    = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->FAN5V_OUT    = Low!\n");
	}
	if(HAL_GPIO_ReadPin(FAN24V_OUT_GPIO_Port, FAN24V_OUT_Pin) == GPIO_PIN_SET){//FAN
		printf("Bootloader:OUTPUT->FAN24V_OUT   = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->FAN24V_OUT   = Low!\n");
	}
	if(HAL_GPIO_ReadPin(TEC_OUT_GPIO_Port, TEC_OUT_Pin) == GPIO_PIN_SET){//TEC
		printf("Bootloader:OUTPUT->TEC_OUT      = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->TEC_OUT      = Low!\n");
	}
	HAL_Delay(10);
	
}
void bootLoadProcess(void){//bootload ִ�г���
	HAL_StatusTypeDef ret;
	uint8_t fileBuff[64];
	uint32_t crcFlash, crcUdisk;
	uint32_t brByte;//ʵ�ʶ�ȡ���ֽ���
	//uint32_t bwByte;//ʵ��д����ֽ���
	//ע��һ��FATFS�ļ�ϵͳ
	switch(bootLoadState){
		case BT_STATE_IDLE:{//�����ȴ�U��ʶ��                             
			SET_GREEN(GPIO_PIN_SET);
			SET_BLUE(GPIO_PIN_RESET);
			SET_RED(GPIO_PIN_RESET);
			printf("Bootloader:Start...............\n");
			readStm32UniqueID();
			printf("Bootloader:UniqueID->0x%08X%08X%08X\n", UniqueId[0], UniqueId[1], UniqueId[2]);
			printf("Bootloader:Mcu flash size:%d Kbytes\n", cpuGetFlashSize());
			printf("Bootloader:Ver:0x%08X Build:%s:%s\n", BOOTLOADER_VER, __DATE__, __TIME__);
			if(HAL_GPIO_ReadPin(INTLOCK_IN_GPIO_Port, INTLOCK_IN_Pin) == GPIO_PIN_SET &&//��ȫ����δ����
			   HAL_GPIO_ReadPin(FSWITCH_NC_GPIO_Port, FSWITCH_NC_Pin) == GPIO_PIN_RESET &&//��̤����
			   HAL_GPIO_ReadPin(FSWITCH_NO_GPIO_Port, FSWITCH_NO_Pin) == GPIO_PIN_RESET){//��̤����
				bootLoadState = BT_STATE_LOAD_FWINFO;//����USB����APP����
			}
			else{//��ȫ��������
				bootLoadState = BT_STATE_RUN_APP;//��������APP����
			}
			break;
		}
		case BT_STATE_LOAD_FWINFO:{
			ret = epromRead(CONFIG_EPROM_FWINFO_START, (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));//��EPROM�����豸����
			if(ret == HAL_OK){
				bootLoadState = BT_STATE_USBHOST_INIT;
				printf("Bootloader:Load eprom done!\n");
			}
			else{
				bootLoadState = BT_STATE_RUN_APP;//��������APP����
			}
			break;
		}
		case BT_STATE_USBHOST_INIT:{//��USB HOST�Ϲ���FATFS
			retUsbH = f_mount(&USBH_fatfs, FATFS_ROOT, 0);
			if(retUsbH != FR_OK){//����U��ʧ��
				printf("Bootloader:Mount Fatfs errror:%d!\n", retUsbH);
				bootLoadState = BT_STATE_RUN_APP;//��ת��APP
			}
			else{//����U�̳ɹ�
				printf("Bootloader:Mount Fatfs sucess!\n");
				bootLoadState = BT_STATE_WAIT_UDISK;
			}
			break;
		}
		case BT_STATE_WAIT_UDISK:{//�ȴ�USB DISK����
			//��ʾ����ʱ
			releaseTime0 = (overTime - (int32_t)HAL_GetTick()) / 1000;
			if(releaseTime0 != releaseTime1){
				printf("Bootloader:Wait usb disk init:%d Second!\n", releaseTime0);
				releaseTime1 = releaseTime0;
			} 
			if(releaseTime0 <= 0){
				bootLoadState = BT_STATE_READ_CFG;
			}
			else if(releaseTime0 <=2 && usbReady == TRUE){//U�̾���
				bootLoadState = BT_STATE_READ_CFG;
			}
			break;
		}
		case BT_STATE_READ_CFG:{
			//���U���Ƿ����Flash Down�ļ�
			retUsbH = f_open(&CfgFile, CFG_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);//��ȡ�����Ϣ�ļ�
			if(retUsbH != FR_OK){//��ȡʧ�������̼�����ֱ�����г���
				printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", CFG_FIRMWARE_FILENAME, retUsbH);
				bootLoadState = BT_STATE_RUN_APP;//��ת������MCU APP�̼�
			}
			else{//��ȡ�ɹ�����ļ�����
				printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", CFG_FIRMWARE_FILENAME, retUsbH);
				f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
				retUsbH = f_read(&CfgFile, fileBuff, 5, &brByte);
				if((retUsbH != FR_OK) || (brByte < 5)){//��ȡ�ļ���ͷ4���ֽ�
					bootLoadFailHandler(BT_FAIL_READ_CFG);//��ȡCFG����
				}
				if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '1'){//U01 ���� MCUӦ��
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Start upgrade mcu application!\n");
						bootLoadState = BT_STATE_UPDATE_MCU_APP;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '2'){//U02 ���� ������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Start upgrade lcd application!\n");
						bootLoadState = BT_STATE_UPDATE_LCD_APP;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '3'){//U03 ���� Ӧ��&������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Start upgrade mcu application & lcd application!\n");
						bootLoadState = BT_STATE_UPDATE_BOTH_APP;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '4'){//U04 ���� EPROM
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Start update eprom!\n");
						bootLoadState = BT_STATE_UPDATE_EPROM;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'D' && fileBuff[1] == '0' && fileBuff[2] == '1'){//D1 ��ȡEPROM
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Start dump eprom to udisk!\n");
						bootLoadState = BT_STATE_DUMP_EPROM;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'C' && fileBuff[1] == '0' && fileBuff[2] == '1'){//U01 �������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Start clear flash and eprom!\n");
						bootLoadState = BT_STATE_CLEAT_ALL;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else{//�������������ת��ִ��APP
					bootLoadState = BT_STATE_RUN_APP;
				}
				f_close(&CfgFile);
			}
			break;
		}
		case BT_STATE_UPDATE_MCU_APP:{//����ld_mcu.bin
			crcFlash = getOriginAppCrc();//����FLASH��APP�̼�CRC32
			crcUdisk = getNewMcuAppCrc();//����U����MCU APP�̼�CRC32
			if((crcUdisk == firmwareInfo.mucAppCrc) && (crcFlash == firmwareInfo.mucAppCrc)){//У������ͬ��������
				printf("Bootloader:Check mcu app crc same,skip!\n");
				bootLoadState = BT_STATE_RESET;
				break;
			}
			crcUdisk = updateMcuApp();//д��MCU FLASH
			crcFlash = getOriginAppCrc();//У��MCU FLASH
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
			}
			else{
				printf("Bootloader:Checksum mcu app sucess.\n");
				firmwareInfo.mucAppCrc = crcFlash;//CRCֵд��EPROM
				epromWrite(CONFIG_EPROM_FWINFO_START, (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
				clearEprom(CLEAR_EPROM_NVRAM);//���NVRAM���索����
				printf("Bootloader:Update new crc32 sucess.\n");
			}
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_LCD_APP:{//����LCDӦ�ó���			
			crcUdisk = getNewLcdAppCrc();//����U����MCU APP�̼�CRC32
			if(crcUdisk == firmwareInfo.lcdAppCrc){//У������ͬ��������
				printf("Bootloader:Check lcd app crc same,skip!\n");
				bootLoadState = BT_STATE_RUN_APP;
				break;
			}
			crcUdisk = updateLcdApp();
			firmwareInfo.lcdAppCrc = crcUdisk;//����EPROM��LCD APP CRCֵ
			epromWrite(CONFIG_EPROM_FWINFO_START, (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
			printf("Bootloader:Update new crc32 sucess.\n");
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_BOTH_APP:{//����ȫ��Ӧ�ó���
			//MCU ����
			crcFlash = getOriginAppCrc();//����FLASH��APP�̼�CRC32
			crcUdisk = getNewMcuAppCrc();//����U����MCU APP�̼�CRC32
			if((crcUdisk == firmwareInfo.mucAppCrc) && (crcFlash == firmwareInfo.mucAppCrc)){//У������ͬ��������
				printf("Bootloader:Check mcu app crc same,skip!\n");
			}
			else{//��U�̸��¹̼�
				crcUdisk = updateMcuApp();	
				crcFlash = getOriginAppCrc();//У��MCU FLASH
				if(crcUdisk != crcFlash){
					bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
				}
				firmwareInfo.mucAppCrc = crcFlash;//CRCֵд��EPROM
				printf("Bootloader:Check mcu app sucess.\n");
				epromWrite(CONFIG_EPROM_FWINFO_START, (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
				clearEprom(CLEAR_EPROM_NVRAM);
				printf("Bootloader:Update mcu app new crc32 sucess.\n");
			}
			//LCD ����
			crcUdisk = getNewLcdAppCrc();//����U����MCU APP�̼�CRC32
			if(crcUdisk == firmwareInfo.lcdAppCrc){//У������ͬ��������
				printf("Bootloader:Check lcd app crc same,skip!\n");
			}
			else{
				updateLcdApp();
				firmwareInfo.lcdAppCrc = crcUdisk;
				epromWrite(CONFIG_EPROM_FWINFO_START, (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
				printf("Bootloader:Update lcd app new crc32 sucess,wait lcd upgrade done\n");
				HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
				HAL_Delay(5000);HAL_Delay(5000);
			}
			bootLoadState = BT_STATE_RESET;
			break;
		}
		case BT_STATE_UPDATE_EPROM:{//����EPROM
			updateEprom();
			break;
		}
		case BT_STATE_DUMP_EPROM:{
			dumpEprom();
			break;
		}
		case BT_STATE_CLEAT_ALL:{//���FLASH��EPROMȫ��
			clearFlash();
			clearEprom(CLEAR_EPROM_ALL);
			bootLoadFailHandler(BT_DONE_CLEAR_ALL);
			break;
		}
		case BT_STATE_RESET:{//��λ
			printf("Bootloader:System Reset\n");
			__set_FAULTMASK(1);
			NVIC_SystemReset();/* Software reset */
			break;
		}
		case BT_STATE_RUN_APP:{//����Ӧ�ó���
			HAL_FLASH_Lock();//����FLASH
			/* Check Vector Table: Test if user code is programmed starting from* address "APPLICATION_ADDRESS" */
			if((((*(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS) & 0xFF000000) == 0x20000000) || (((*(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS) & 0xFF000000) == 0x10000000)){
				/* Jump to user application */
				printf("Bootloader:Jump application address.\n");
				JumpAddress = *(__IO uint32_t *) (APPLICATION_FLASH_START_ADDRESS + 4);
				printf("Bootloader:Jump Address:0x%X\n", JumpAddress);
				Jump_To_Application = (pFunction) JumpAddress;
				/* Initialize user application's Stack Pointer */
				__set_MSP(*(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS);
				printf("Bootloader:Stack Pointer:0x%X\n", *(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS);
				printf("\n\n\n\r\r\r");
				
				SET_GREEN(GPIO_PIN_RESET);
				SET_BLUE(GPIO_PIN_RESET);
				SET_RED(GPIO_PIN_RESET);
				__disable_irq();
				SysTick->CTRL = 0;//�ؼ�����
				//�ر��ж�                                    				
				HAL_NVIC_DisableIRQ(SysTick_IRQn); 
				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
				HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
				HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
				HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
				HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
				//ȡ����������
				//HAL_DeInit();
				//HAL_I2C_MspDeInit(&hi2c1);
				//HAL_UART_MspDeInit(&huart1);
				//HAL_UART_MspDeInit(&huart5);
				//USBH_DeInit(&hUsbHostFS);
				//SystemClock_Reset();//��λRCCʱ��
				//UsbGpioReset();
				Jump_To_Application();
			}
			bootLoadFailHandler(BT_FAIL_VECTOR_TABLE_INVALID);
		}
		default:break;
	}
}
static void beepDiag(uint8_t diag){//������������� Ħ��˹����
	//�ر�USB VBUS
	switch(diag){
		case '0':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '1':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		};
		case '2':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		};
		case '3':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '4':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '5':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '6':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '7':{	
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '8':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case '9':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'A':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'B':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;			
		}
		case 'C':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'D':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'E':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'F':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'G':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'H':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;			
		}
		case 'I':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'J':{
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'K':{
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'L':{//���� ����
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_SHORT_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		case 'M':{//�� ��
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RED(GPIO_PIN_SET);HAL_Delay(MORSECODE_LONG_TIME);SET_RED(GPIO_PIN_RESET);
			break;
		}
		default:break;
	}
	HAL_Delay(3000);
}
static void bootLoadFailHandler(uint8_t ftype){//�����������
	MX_DriverVbusFS(FALSE);//�ر�USB VBUS
	SET_GREEN(GPIO_PIN_RESET);
	SET_BLUE(GPIO_PIN_RESET);
	SET_RED(GPIO_PIN_RESET);
	switch(ftype){
		case BT_FAIL_READ_CFG:{//��U�̶�ȡCFGʧ��
			printf("Bootloader:FailHandler,Read %s fail!.\n", CFG_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_CFG);
			};
		}
		case BT_FAIL_READ_LMCU_APP:{//��U�̶�ȡMCU APPʧ��
			printf("Bootloader:FailHandler,Read %s fail!.\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_LMCU_APP);
			};
		}
		case BT_FAIL_READ_LLCD_APP:{//��U�̶�ȡLCD APPʧ��
			printf("Bootloader:FailHandler,Read %s fail!.\n", LLCD_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_LLCD_APP);
			};
		}
		case BT_FAIL_ERASE_MCU_APP:{//����MCU APP FLASH����ʧ��
			printf("Bootloader:FailHandler,Erase mcu application fail\n");
			while(1){
				beepDiag(BT_FAIL_ERASE_MCU_APP);
			};
		}
		case BT_FAIL_READ_LEROM_BIN:{
			printf("Bootloader:FailHandler,Read %s fail!\n", LOAD_EPROM_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_LEROM_BIN);
			}
		}
		case BT_FAIL_WRITE_SEROM_BIN:{
			printf("Bootloader:FailHandler,Write %s fail!\n", SAVE_EPROM_FILENAME);
			while(1){
				beepDiag(BT_FAIL_WRITE_SEROM_BIN);
			}
		}
		case BT_FAIL_LMCU_APP_CHECK:{//lmcu.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_LMCU_APP_CHECK);
			};
		}
		case BT_FAIL_LLCD_APP_CHECK:{//llcd.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LLCD_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_LLCD_APP_CHECK);
			};
		}
		case BT_FAIL_CHECKSUM_MCU_APP_FLASH:{//У�� lmcu.bin ����
			printf("Bootloader:FailHandler,Verify %s fail!.\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
			};
		}
		case BT_FAIL_LCD_NOT_RESPOND:{//LCD ��������Ӧ�����
			printf("Bootloader:FailHandler,LCD is not responsed!.\n");
			while(1){
				beepDiag(BT_FAIL_LCD_NOT_RESPOND);
			};
		}
		case BT_FAIL_LCD_DOWNLOAD:{//LCD ��������Ӧ
			printf("Bootloader:FailHandler,LCD download fail!.\n");
			while(1){
				beepDiag(BT_FAIL_LCD_DOWNLOAD);
			};
		}
		case BT_FAIL_VECTOR_TABLE_INVALID:{//APP��Ч������
			printf("Bootloader:FailHandler,App vector table invalid.\n");
			while(1){
				beepDiag(BT_FAIL_VECTOR_TABLE_INVALID);
			};
		}
		case BT_FAIL_CHECK_BLANK:{//FLASH ��մ���
			printf("Bootloader:FailHandler,Flash is not blank!.\n");
			while(1){
				beepDiag(BT_FAIL_CHECK_BLANK);
			};
		}
		case BT_DONE_CLEAR_ALL:{
			printf("Bootloader:DoneHandler,Flash and Eprom easer done!.\n");
			while(1){
				beepDiag(BT_DONE_CLEAR_ALL);
			}
		}
		case BT_DONE_UPDATE_EPROM:{
			printf("Bootloader:DoneHandler,Update eprom form udisk done!.\n");
			while(1){
				beepDiag(BT_DONE_UPDATE_EPROM);
			}
		}
		case BT_DONE_DUMP_EPROM:{
			printf("Bootloader:DoneHandler,Dump eprom to udisk done!\n");
			while(1){
				beepDiag(BT_DONE_DUMP_EPROM);
			}
		}
		default:{
			break;
		}
	}
}
static void clearFlash(void){//���MCU FLASH
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
	if (FLASH_If_EraseApplication() != 0x00){//����APP FLASH����ʧ��
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_SIZE);//FLASH ���
}
static uint32_t getNewMcuAppCrc(void){//��ȡ������MCU APP CRC32
	uint32_t crc32;
	uint32_t i;
	uint8_t readflag = TRUE;
	uint16_t bytesread;//ʵ���ļ���ȡ�ֽ���
	retUsbH = f_open(&McuFile, LMCU_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LMCU_APP);			
	}
	if(f_size(&McuFile) > APPLICATION_FLASH_SIZE){//MCU�̼�����FLSAH����
		bootLoadFailHandler(BT_FAIL_LMCU_APP_CHECK);
	}
	crc32 = 0;
	crc32Clear();
	while(readflag){
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&McuFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		/* Temp variable */
		TmpReadSize = bytesread;
		/* The read data < "BUFFER_SIZE" Kbyte */
		if(TmpReadSize < BUFFER_SIZE){
			readflag = FALSE;
		}
		LastPGAddress += TmpReadSize;
	}
	for(i = LastPGAddress;i < APPLICATION_FLASH_END_ADDRESS;i ++){//����ʣ��CRC
		crc32 = crc32CalculateAdd(0xFF);
	}
	f_close(&McuFile);
	return crc32;
}
static uint32_t getNewLcdAppCrc(void){//��ȡ������LCD APP CRC16
	uint32_t crc32;
	uint8_t readflag = TRUE;
	uint16_t bytesread;//ʵ���ļ���ȡ�ֽ���
	retUsbH = f_open(&LcdFile, LLCD_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LLCD_APP);			
	}
	crc32 = 0;
	crc32Clear();
	while(readflag){
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&LcdFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		/* Temp variable */
		TmpReadSize = bytesread;
		/* The read data < "BUFFER_SIZE" Kbyte */
		if(TmpReadSize < BUFFER_SIZE){
			readflag = FALSE;
		}
		LastPGAddress += TmpReadSize;
	}
	f_close(&LcdFile);
	return crc32;
}
static uint32_t updateMcuApp(void){//����MCU APP
	uint32_t crc32;
	uint32_t i;
	uint32_t programcounter = 0x00;
	uint8_t readflag = TRUE;
	uint32_t bytesread;//ʵ���ļ���ȡ�ֽ���
	retUsbH = f_open(&McuFile, LMCU_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LMCU_APP);			
	}
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", LMCU_FIRMWARE_FILENAME, retUsbH);
	if(f_size(&McuFile) > APPLICATION_FLASH_SIZE){//MCU�̼�����FLSAH����
		bootLoadFailHandler(BT_FAIL_LMCU_APP_CHECK);
	}
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
	if (FLASH_If_EraseApplication() != 0x00){//����APP FLASH����ʧ��
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_SIZE);//FLASH ���
	printf("Bootloader:Erase mcu application sucess.\n");
	RamAddress = (uint32_t)&RAM_Buf;//��ȡRAM��������ַ
	/* Erase address init */
	LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
	/* While file still contain data */
	crc32 = 0;
	crc32Clear();
	while(readflag){
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&McuFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		/* Temp variable */
		TmpReadSize = bytesread;
		/* The read data < "BUFFER_SIZE" Kbyte */
		if(TmpReadSize < BUFFER_SIZE){
			readflag = FALSE;
		}
		/* Program flash memory */
		FLIP_GREEN();//�̵�
		for(programcounter = 0; programcounter < TmpReadSize; programcounter += 4){
			/* Write word into flash memory */
			if(FLASH_If_Write((LastPGAddress + programcounter), *(uint32_t *) (RamAddress + programcounter)) != 0x00){
				bootLoadFailHandler(BT_FAIL_WRITE_MCU_APP_FLASH);//д��FLASH����
			}
		}
		/* Update last programmed address value */
		LastPGAddress += TmpReadSize;
	}
	for(i = LastPGAddress;i < APPLICATION_FLASH_END_ADDRESS;i ++){//����ʣ��CRC
		crc32 = crc32CalculateAdd(0xFF);
	}
	HAL_FLASH_Lock();
	printf("Bootloader:Write mcu app finish.\n");
	f_close(&McuFile);
	return crc32;
}
static uint32_t updateLcdApp(void){//����LCD APP
	UART_HandleTypeDef *puart;
	HAL_StatusTypeDef uRet;
	uint32_t crc32;
	uint8_t baudrateSelect;
	uint8_t signName;
    uint8_t preCmd[] =		   {0x61,0x78,0x72,0x63,0x65,0x6b,0x67,0x64,
					            0x79,0x68,0x74,0x73,0x75,0x6e,0x71,0x77,
								0x70,0x6a,0x62,0x76,0x69,0x66,0x6f,0x6d,
								0x7a,0x6c};//����ǰ����
	uint8_t cmd[] = 			{0xEE,0xF1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFC,0xFF,0xFF};        //��������
	uint8_t cmdSnake[] = 		{0xEE,0x04,0xFF,0xFC,0xFF,0xFF};//��Ļ��������
	uint8_t cmdSnakeBack[] = 	{0xEE,0x55,0xFF,0xFC,0xFF,0xFF};//���ַ�������
	uint32_t bufIndex;
	uint32_t baudrateTable[]={9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};//���ò����ʷ�ǰ��
	uint32_t fileSize;//�ļ���С
	uint32_t blockSize = 2048;//��Ҫ��ȡ�����ݰ���С
	uint32_t transferByte;//��ȡ�ֽ���
	uint32_t actualByte;//ʵ�ʶ�ȡ���ֽ���
	uint32_t upSpeedBaudrate = GDDC_UPDATE_BAUDRATE;//����������
	uint32_t fileIndex;
	uint16_t checkSum;//У����
	uint8_t lcdRetry;//�����ظ�����
	puart = &GDDC_UART_HANDLE;
	retUsbH = f_open(&LcdFile, LLCD_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LLCD_APP);
	}
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", LLCD_FIRMWARE_FILENAME, retUsbH);
	for(baudrateSelect = 0; baudrateSelect < (sizeof(baudrateTable) / 4); baudrateSelect ++){//�����ʲ��ԣ�����		
		if(baudrateSelect >= (sizeof(baudrateTable) / 4)){
			bootLoadFailHandler(BT_FAIL_LCD_NOT_RESPOND);
		}
		__HAL_UART_DISABLE(puart);//�رմ���
		HAL_NVIC_DisableIRQ(GDDC_UART_IRQ);//�رմ����ж�
		HAL_NVIC_ClearPendingIRQ(GDDC_UART_IRQ);//��������жϱ�־
		puart->Init.BaudRate = baudrateTable[baudrateSelect];
		puart->Init.WordLength = UART_WORDLENGTH_8B;
		puart->Init.StopBits = UART_STOPBITS_1;
		puart->Init.Parity = UART_PARITY_NONE;
		puart->Init.Mode = UART_MODE_TX_RX;
		puart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		puart->Init.OverSampling = UART_OVERSAMPLING_16;
		if(HAL_UART_Init(puart) != HAL_OK){
			Error_Handler();
		}
		__HAL_UART_ENABLE(puart);//�򿪴���
		memset(gddcRxBuf, 0x0, GDDC_RX_BUF_SIZE);
		printf("Bootloader->updateLcdApp:Try lcd serial baudrate %d,send cmdSnake.\n", baudrateTable[baudrateSelect]);
		dp_display_text_num(cmdSnake, 6);//������������
		uRet = HAL_UART_Receive(puart, gddcRxBuf, 6, 2000);//��ѯ���ڽ������� ��ʱ1000
		if(strcmp((char *)cmdSnakeBack, (char *)gddcRxBuf) == 0){
			printf("Bootloader->updateLcdApp:Received cmdSnakeBack,set lcd serial baudrate %d.\n", baudrateTable[baudrateSelect]);
			break;
		}
	}
	dp_display_text_num(preCmd, sizeof(preCmd));
	retUsbH = f_open(&LcdFile, LLCD_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LLCD_APP);
	}
    fileSize =  f_size(&LcdFile);   
    //�ļ���С
	cmd[2] = (fileSize >> 24) & 0xff;
	cmd[3] = (fileSize >> 16) & 0xff;
	cmd[4] = (fileSize >>  8) & 0xff;
	cmd[5] = (fileSize) & 0xff;    
	cmd[6] = (baudrateTable[baudrateSelect] >> 24) & 0xff;
	cmd[7] = (baudrateTable[baudrateSelect] >> 16) & 0xff;
	cmd[8] = (baudrateTable[baudrateSelect] >>  8) & 0xff;
	cmd[9] = (baudrateTable[baudrateSelect]) & 0xff;    
	//У���
	cmd[10] = cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6] + cmd[7] + cmd[8] + cmd[9];
    //������������
	memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));
	printf("Bootloader->updateLcdApp:Send filesize and up speed baudrate.\n");
	dp_display_text_num(cmd, 15);
	uRet = HAL_UART_Receive(puart, gddcRxBuf, 1, 10000);//��ѯ���ڽ������� ��ʱ10000mS
	if(uRet != HAL_OK || gddcRxBuf[0] != 0xAA){
		bootLoadFailHandler(BT_FAIL_LCD_NOT_RESPOND);
	}
	printf("Bootloader->updateLcdApp:Set up baudrate done.\n");
	if(baudrateTable[baudrateSelect] != upSpeedBaudrate){//�����������뵱ǰ�����ʲ���ͬ
		//���贮�ڵ�����������
		printf("Bootloader->updateLcdApp:Set lcd serial up baudrate %d.\n", upSpeedBaudrate);
		__HAL_UART_DISABLE(puart);//�رմ���
		HAL_NVIC_DisableIRQ(GDDC_UART_IRQ);//�رմ����ж�
		HAL_NVIC_ClearPendingIRQ(GDDC_UART_IRQ);//��������жϱ�־
		GDDC_UART_HANDLE.Init.BaudRate = upSpeedBaudrate;
		GDDC_UART_HANDLE.Init.WordLength = UART_WORDLENGTH_8B;
		GDDC_UART_HANDLE.Init.StopBits = UART_STOPBITS_1;
		GDDC_UART_HANDLE.Init.Parity = UART_PARITY_NONE;
		GDDC_UART_HANDLE.Init.Mode = UART_MODE_TX_RX;
		GDDC_UART_HANDLE.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		GDDC_UART_HANDLE.Init.OverSampling = UART_OVERSAMPLING_16;
		if(HAL_UART_Init(puart) != HAL_OK){
			Error_Handler();
		}
		__HAL_UART_ENABLE(puart);//�򿪴���
	}
	HAL_Delay(200);//�ȴ�200mS
    //һ�����ݰ��̶�Ϊ2052�ֽ�
    //��ʽ:SN ~SN DATA CHECKSUM
    //SNΪ�����,1�ֽ�0~255ѭ��ʹ��
    //~SNΪ�������ȡ��
    //DATA�̶�Ϊ2048�ֽ�,������0
    //CHECKSUM,2�ֽ�,ǰ��2050�ֽ���ͺ�ȡ��
	signName = 0;
	crc32 = 0;
	crc32Clear();
	for(fileIndex = 0; fileIndex < fileSize; fileIndex += blockSize){
		memset(gddcTxBuf, 0x0, sizeof(gddcTxBuf));  
		gddcTxBuf[0] = signName;
		gddcTxBuf[1] = ~signName;
        //��ȡ2048���ֽڵ��������ļ���С
         transferByte = blockSize;
		if(fileIndex + transferByte > fileSize){
			transferByte = fileSize - fileIndex;
		}
		retUsbH = f_read(&LcdFile, &gddcTxBuf[2], transferByte, &actualByte);
		crc32 = crc32Calculate(&gddcTxBuf[2], actualByte);
		if(retUsbH != FR_OK){
			bootLoadFailHandler(BT_FAIL_READ_LLCD_APP);
        }
        //����У���
		checkSum = 0x0;
		bufIndex = 0;
		do{
			checkSum += (uint16_t)gddcTxBuf[bufIndex];
			bufIndex ++;
		}while(bufIndex < 2050);
        //ȡ������ڰ�ĩβ
		checkSum = (uint16_t)~(checkSum);
		gddcTxBuf[2050] = (checkSum >> 8) & 0xFF;
		gddcTxBuf[2051] = (checkSum) & 0xFF;   
		do{
			lcdRetry ++;
			//�������ݰ���ֱ���ɹ��������������
			memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));
			printf("Bootloader->updateLcdApp:Send file block at 0x%08XH,", fileIndex);
			dp_display_text_num(gddcTxBuf, (blockSize + 4));//send data
			uRet = HAL_UART_Receive(puart, gddcRxBuf, 2, 1000);//��ѯ���ڽ������� ��ʱ1000            
			if(uRet == HAL_OK){//��������
				if(gddcRxBuf[1] == (uint8_t)(~(signName + 1)) || gddcRxBuf[0] == (signName+1)){
					signName = signName + 1;
					printf("ok!\n");
					break;
				}
				else{
					printf("SignName is not invalid\n");
				}
			}
			else{//��Ӧ��ʱ�����
				printf("fail!\n");
				HAL_Delay(100);
				if(lcdRetry > GDDC_RETRY_TIMES){//���ʹ�����ʱ
					bootLoadFailHandler(BT_FAIL_LCD_DOWNLOAD);
				}
			}
		}while(1);
	}
	f_close(&LcdFile);
	return crc32;
}
static void updateEprom(void){//UDISK->EPROM
	HAL_StatusTypeDef ret;
	uint32_t brByte;
	retUsbH = f_open(&LepromFile, LOAD_EPROM_FILENAME, FA_OPEN_EXISTING | FA_READ);//��ȡ�����Ϣ�ļ�
	if(retUsbH != FR_OK){//��ȡʧ�������̼�����ֱ�����г���
		printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", LOAD_EPROM_FILENAME, retUsbH);
				bootLoadFailHandler(BT_FAIL_READ_LEROM_BIN);
	}
	else{//��ȡ�ɹ�����ļ�����
		printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", LOAD_EPROM_FILENAME, retUsbH);
		f_lseek(&LepromFile, 0);//��ȡָ���ƶ�����ͷ
		retUsbH = f_read(&LepromFile, RAM_Buf, CONFIG_EPROM_SIZE, &brByte);
		if((retUsbH != FR_OK) || (brByte !=  CONFIG_EPROM_SIZE)){
			bootLoadFailHandler(BT_FAIL_READ_LEROM_BIN);
		}
		f_close(&LepromFile);
		ret = epromWrite(0, RAM_Buf, CONFIG_EPROM_SIZE);//д��EPROM
		if(ret != HAL_OK){
			bootLoadFailHandler(BT_FAIL_WRITE_EPROM);
		}
		bootLoadFailHandler(BT_DONE_UPDATE_EPROM);
	}
}
static void dumpEprom(void){//����EPROM��Ϣ��U��
	uint32_t wrByte;
	epromRead(0x0, RAM_Buf, CONFIG_EPROM_SIZE);
	retUsbH = f_open(&SepromFile, LOAD_EPROM_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
	if(retUsbH != FR_OK){//��ʧ��
		bootLoadFailHandler(BT_FAIL_WRITE_SEROM_BIN);
	}
	retUsbH = f_write(&SepromFile, RAM_Buf, CONFIG_EPROM_SIZE, &wrByte);
	if(retUsbH != FR_OK){//д��ʧ��
		bootLoadFailHandler(BT_FAIL_WRITE_SEROM_BIN);
	}
	bootLoadFailHandler(BT_DONE_DUMP_EPROM);
}
static uint32_t getOriginAppCrc(void){//����MCU APP CRC32
	uint8_t val;
	uint32_t i;
	uint32_t crc32;
	crc32Clear();
	for(i = APPLICATION_FLASH_START_ADDRESS;i < APPLICATION_FLASH_END_ADDRESS;i ++){
		val = *(__IO uint8_t*)(i);
		crc32 = crc32CalculateAdd(val);//CRC32���������ֽ�
	}
	return crc32;	
}
/*****************************************************************************/
//����������
static void DBGU_Printk(uint8_t *buffer){//arg pointer to a string ending by
	while(*buffer != '\0'){
		HAL_UART_Transmit(&GDDC_UART_HANDLE, buffer, 1, GDDC_TX_TIMEOUT);
		buffer ++;
    }
}
static void DBGU_Printk_num(uint8_t *buffer, uint16_t datanum){//arg pointer to a string ending by
    while(datanum != 0){
		HAL_UART_Transmit(&GDDC_UART_HANDLE, buffer, 1, GDDC_TX_TIMEOUT);
		buffer ++;
        datanum--;
    }
}
static void dp_display_text(uint8_t *text){
    /* User Specific Code   */
    DBGU_Printk(text);
}
static void dp_display_text_num(uint8_t *text,uint16_t datanum){
    /* User Specific Code   */
    DBGU_Printk_num(text, datanum);
}
static void dp_display_value(uint32_t value, int descriptive){
    /* User Specific Code   */
    uint8_t print_buf[10];
    if (descriptive == GDDC_HEX){
        sprintf((char *)print_buf, "%lX", value);
        DBGU_Printk(print_buf);
    }
    else if(descriptive == GDDC_DEC){
        sprintf((char *)print_buf, "%ld", value);
        DBGU_Printk(print_buf);
    }
    else if(descriptive == GDDC_CHR){
        sprintf((char *)print_buf, "%c", (uint8_t)value);
        DBGU_Printk(print_buf);
    }
}
static void dp_display_array(uint8_t *value, int bytes, int descriptive){
    /* User Specific Code */
    uint8_t print_buf[10];
    int i;
    for(i=0; i < bytes; i++){
        if (descriptive == GDDC_HEX){
            sprintf((char *)print_buf, "%lX", (uint8_t)value[i]);
            DBGU_Printk(print_buf);
        }
        else if(descriptive == GDDC_DEC){
            sprintf((char *)print_buf, "%ld", (uint8_t)value[i]);
            DBGU_Printk(print_buf);
        }
        else if(descriptive == GDDC_CHR){
            sprintf((char *)print_buf, "%c", (uint8_t)value[i]);
            DBGU_Printk(print_buf);
        }
    }
}
/*****************************************************************************/
//MCU FLASH����
static void checkBlank(uint32_t adr, uint32_t size){//MCU Flash ���
	uint8_t val;
	uint32_t i;
	for(i = 0;i < size;i ++){
		val = *(__IO uint8_t*)(adr + i);
		if(val != 0xFF){
			bootLoadFailHandler(BT_FAIL_CHECK_BLANK);
		}
	}
}
/*****************************************************************************/
//EEPROM ��д����
static HAL_StatusTypeDef epromReadByte(uint16_t ReadAddr, uint8_t *rdat){//��ָ����ַ����8λ����
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����				  
	HAL_StatusTypeDef ret;
	if(ReadAddr > (CONFIG_EPROM_SIZE - 1)){//д��ַ��������
		ret = HAL_ERROR;
		return ret;
	}	
	ret = HAL_I2C_Mem_Read(&hi2c1,
	                       CONFIG_EPROM_READ_ADDR,
	                       ReadAddr,
	                       I2C_MEMADD_SIZE_16BIT,
	                       (uint8_t*)(rdat),
	                       1,
	                       CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);//�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);//������³�ʼ��I2C������
		printf("sPlc->sPlcEprom:Eprom read byte fail!\n");
	}
	return ret;
}
static HAL_StatusTypeDef epromWriteByte(uint16_t WriteAddr, uint8_t wdat){//��ָ����ַд��8λ����
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������				   	  	    																 
	HAL_StatusTypeDef ret;
	if(WriteAddr > (CONFIG_EPROM_SIZE - 1)){//д��ַ��������
		ret = HAL_ERROR;
		return ret;
	}
	ret = HAL_I2C_Mem_Write(&hi2c1, 
	                        CONFIG_EPROM_WRITE_ADDR,
	                        WriteAddr, 
	                        I2C_MEMADD_SIZE_16BIT, 
	                        &wdat, 
	                        1, 
	                        CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);//�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);//������³�ʼ��I2C������
		printf("sPlc->sPlcEprom:Eprom write byte fail!\n");
	}
	return ret;
}
static HAL_StatusTypeDef epromRead(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead){//�ڵ�ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
	HAL_StatusTypeDef ret;
	uint16_t rAddr, rBlock, rByte, doBlock;
	uint8_t* rBuffer;
	if(ReadAddr + NumToRead >= (CONFIG_EPROM_SIZE - 1)){//����ַ��������
		ret = HAL_ERROR;
		return ret;
	}
	rBlock = NumToRead / CONFIG_EPROM_PAGE_SIZE;
	rByte = NumToRead % CONFIG_EPROM_PAGE_SIZE;
	rAddr = ReadAddr;
	rBuffer = pBuffer;
	for(doBlock = 0;doBlock < rBlock;doBlock ++){
		ret = HAL_I2C_Mem_Read(&hi2c1, CONFIG_EPROM_READ_ADDR, rAddr, I2C_MEMADD_SIZE_16BIT, rBuffer, CONFIG_EPROM_PAGE_SIZE, CONFIG_EPROM_TIMEOUT);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);//�ͷ�IO��ΪGPIO����λ���״̬��־
			ret = HAL_I2C_Init(&hi2c1);//������³�ʼ��I2C������
			printf("sPlc->sPlcEprom:Eprom read block fail!\n");
		}
		rAddr += CONFIG_EPROM_PAGE_SIZE;
		rBuffer += CONFIG_EPROM_PAGE_SIZE;
	}
	if(rByte != 0x0){
		ret = HAL_I2C_Mem_Read(&hi2c1, CONFIG_EPROM_READ_ADDR, rAddr, I2C_MEMADD_SIZE_16BIT, rBuffer, rByte ,CONFIG_EPROM_TIMEOUT);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);        //�ͷ�IO��ΪGPIO����λ���״̬��־
			ret = HAL_I2C_Init(&hi2c1);          //������³�ʼ��I2C������
		}
		printf("sPlc->sPlcEprom:Eprom read rbyte fail\n");
	}
	return ret;	
}  
static HAL_StatusTypeDef epromWrite(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite){//�ڵ�ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
	HAL_StatusTypeDef ret;
	uint16_t wAddr, wBlock, wByte, doBlock;
	uint8_t* wBuffer;
	if(WriteAddr + NumToWrite >= (CONFIG_EPROM_SIZE - 1)){//����ַ��������
		ret = HAL_ERROR;
		return ret;
	}
	wBlock = NumToWrite / CONFIG_EPROM_PAGE_SIZE;
	wByte = NumToWrite % CONFIG_EPROM_PAGE_SIZE;
	wAddr = WriteAddr;
	wBuffer = pBuffer;
	for(doBlock = 0;doBlock < wBlock;doBlock ++){
		ret = HAL_I2C_Mem_Write(&hi2c1, CONFIG_EPROM_WRITE_ADDR, wAddr, I2C_MEMADD_SIZE_16BIT, wBuffer, CONFIG_EPROM_PAGE_SIZE, CONFIG_EPROM_TIMEOUT);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);        //�ͷ�IO��ΪGPIO����λ���״̬��־
			ret = HAL_I2C_Init(&hi2c1);          //������³�ʼ��I2C������
			printf("sPlc->sPlcEprom:Eprom write block fail!\n");	
		}
		wAddr += CONFIG_EPROM_PAGE_SIZE;
		wBuffer += CONFIG_EPROM_PAGE_SIZE;
#if CONFIG_EPROM_WRITE_DELAY > 0
		HAL_Delay(CONFIG_EPROM_WRITE_DELAY);
#endif
	}
	if(wByte != 0x0){
		ret = HAL_I2C_Mem_Write(&hi2c1, CONFIG_EPROM_WRITE_ADDR, wAddr, I2C_MEMADD_SIZE_16BIT, wBuffer, wByte, CONFIG_EPROM_TIMEOUT);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);        //�ͷ�IO��ΪGPIO����λ���״̬��־
			ret = HAL_I2C_Init(&hi2c1);          //������³�ʼ��I2C������
			printf("sPlc->sPlcEprom:Eprom write remain byte fail!\n");
		}
	}
#if CONFIG_EPROM_WRITE_DELAY > 0
	HAL_Delay(CONFIG_EPROM_WRITE_DELAY);
#endif
	return ret;
}
static void clearEprom(clarmEpromCmd_t cmd){//���EPROM����
	uint8_t var = 0;
	uint32_t i;	
	switch(cmd){
		case CLEAR_EPROM_ALL:{
			for(i = 0;i < CONFIG_EPROM_SIZE;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase all eprom sucess!\n");
			break;
		}
		case CLEAR_EPROM_FIRMWARE_INFO:{
			for(i = CONFIG_EPROM_FWINFO_START;i <= CONFIG_EPROM_FWINFO_END;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom firmware info sucess!\n");
			break;
		}
		case CLEAR_EPROM_DEVICE_CONFIG:{
			for(i = CONFIG_EPROM_CONFIG_START;i <= CONFIG_EPROM_CONFIG_END;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom device info sucess!\n");
			break;
		}
		case CLEAR_EPROM_LOG_INFO:{
			for(i = CONFIG_EPROM_LOGINFO_START;i <= CONFIG_EPROM_LOGINFO_END;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom log info sucess!\n");
			break;
		}
		case CLEAR_EPROM_NVRAM:{
			for(i = CONFIG_EPROM_LOGINFO_START;i <= CONFIG_EPROM_NVRAM_END;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom nvram sucess!\n");
			for(i = CONFIG_EPROM_FDRAM_START;i <= CONFIG_EPROM_FDRAM_END;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom fdram info sucess!\n");
			break;
		}
		default:break;
	}
}




