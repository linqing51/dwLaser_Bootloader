#include "bootLoader.h"
/*****************************************************************************/
//SECTOR0->16K:BOOTLOADER
//SECTOR1->16K:BOOTLOADER
//SECTOR2->16K:BOOTLOADER
//SECTOR3->16K:BOOTLOADER
/*****************************************************************************/
#pragma import(__use_no_semihosting_swi)
/*****************************************************************************/
#define SET_EDAC0_CS(b)													HAL_GPIO_WritePin(EDAC0_CS_GPIO_Port, EDAC0_CS_Pin, b)
#define SET_EDAC1_CS(b)													HAL_GPIO_WritePin(EDAC1_CS_GPIO_Port, EDAC1_CS_Pin, b)
#define SET_EDAC2_CS(b)													HAL_GPIO_WritePin(EDAC2_CS_GPIO_Port, EDAC2_CS_Pin, b)
#define SET_EDAC3_CS(b)													HAL_GPIO_WritePin(EDAC3_CS_GPIO_Port, EDAC3_CS_Pin, b)
#define SET_EDAC0_SCK(b)												HAL_GPIO_WritePin(EDAC0_SCK_GPIO_Port, EDAC0_SCK_Pin, b)
#define SET_EDAC1_SCK(b)												HAL_GPIO_WritePin(EDAC1_SCK_GPIO_Port, EDAC1_SCK_Pin, b)
#define SET_EDAC2_SCK(b)												HAL_GPIO_WritePin(EDAC2_SCK_GPIO_Port, EDAC2_SCK_Pin, b)
#define SET_EDAC3_SCK(b)												HAL_GPIO_WritePin(EDAC3_SCK_GPIO_Port, EDAC3_SCK_Pin, b)
#define SET_EDAC0_SDI(b)												HAL_GPIO_WritePin(EDAC0_SDI_GPIO_Port, EDAC0_SDI_Pin, b)
#define SET_EDAC1_SDI(b)												HAL_GPIO_WritePin(EDAC1_SDI_GPIO_Port, EDAC1_SDI_Pin, b)
#define SET_EDAC2_SDI(b)												HAL_GPIO_WritePin(EDAC2_SDI_GPIO_Port, EDAC2_SDI_Pin, b)
#define SET_EDAC3_SDI(b)												HAL_GPIO_WritePin(EDAC3_SDI_GPIO_Port, EDAC3_SDI_Pin, b)
#define GET_ESTOP_NC														HAL_GPIO_ReadPin(ESTOP_NC_GPIO_Port, ESTOP_NC_Pin)
#define GET_INTERLOCK_NC												HAL_GPIO_ReadPin(INTERLOCK_NC_GPIO_Port, INTERLOCK_NC_Pin)
#define GET_FSWITCH_NO													HAL_GPIO_ReadPin(FS_NO_GPIO_Port, FS_NO_Pin)
#define GET_FSWITCH_NC													HAL_GPIO_ReadPin(FS_NC_GPIO_Port, FS_NC_Pin)

#define SET_RED_LED_ON													HAL_GPIO_WritePin(RED_LED_PWM_GPIO_Port, GPIO_PIN_7, GPIO_PIN_SET)
#define SET_RED_LED_OFF													HAL_GPIO_WritePin(RED_LED_PWM_GPIO_Port, GPIO_PIN_7, GPIO_PIN_RESET)
#define FLIP_RED_LED											 			HAL_GPIO_TogglePin(RED_LED_PWM_GPIO_Port, GPIO_PIN_7)

#define SET_GREEN_LED_ON												HAL_GPIO_WritePin(GREEN_LED_PWM_GPIO_Port, GREEN_LED_PWM_Pin, GPIO_PIN_SET)
#define SET_GREEN_LED_OFF												HAL_GPIO_WritePin(GREEN_LED_PWM_GPIO_Port, GREEN_LED_PWM_Pin, GPIO_PIN_RESET)
#define FLIP_GREEN_LED  												HAL_GPIO_TogglePin(GREEN_LED_PWM_GPIO_Port, GREEN_LED_PWM_Pin)

#define SET_BLUE_LED_ON													HAL_GPIO_WritePin(BLUE_LED_PWM_GPIO_Port, BLUE_LED_PWM_Pin, GPIO_PIN_SET)
#define SET_BLUE_LED_OFF												HAL_GPIO_WritePin(BLUE_LED_PWM_GPIO_Port, BLUE_LED_PWM_Pin, GPIO_PIN_RESET)
#define FLIP_BLUE_LED  													HAL_GPIO_TogglePin(BLUE_LED_PWM_GPIO_Port, BLUE_LED_PWM_Pin)

#define SET_ERR_LED_ON													HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_SET)
#define SET_ERR_LED_OFF													HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_RESET)
#define GET_ERR_LED															HAL_GPIO_ReadPin(ERR_LED_GPIO_Port, ERR_LED_Pin)
#define FLIP_ERR_LED														HAL_GPIO_TogglePin(ERR_LED_GPIO_Port, ERR_LED_Pin)
#define SET_TICK_LED_ON													HAL_GPIO_WritePin(TICK_LED_GPIO_Port, TICK_LED_Pin, GPIO_PIN_SET)
#define SET_TICK_LED_OFF												HAL_GPIO_WritePin(TICK_LED_GPIO_Port, TICK_LED_Pin, GPIO_PIN_RESET)
#define GET_TICK_LED														HAL_GPIO_ReadPin(TICK_LED_GPIO_Port, TICK_LED_Pin)
#define FLIP_TICK_LED														HAL_GPIO_TogglePin(TICK_LED_GPIO_Port, TICK_LED_Pin)
#define SET_LASER_CH0_ON												HAL_GPIO_WritePin(LAS_PWM0_GPIO_Port, LAS_PWM0_Pin, GPIO_PIN_SET)
#define SET_LASER_CH0_OFF												HAL_GPIO_WritePin(LAS_PWM0_GPIO_Port, LAS_PWM0_Pin, GPIO_PIN_RESET)
#define SET_LASER_CH1_ON												HAL_GPIO_WritePin(LAS_PWM1_GPIO_Port, LAS_PWM1_Pin, GPIO_PIN_SET)
#define SET_LASER_CH1_OFF												HAL_GPIO_WritePin(LAS_PWM1_GPIO_Port, LAS_PWM1_Pin, GPIO_PIN_RESET)
#define SET_LASER_CH2_ON												HAL_GPIO_WritePin(LAS_PWM2_GPIO_Port, LAS_PWM2_Pin, GPIO_PIN_SET)
#define SET_LASER_CH2_OFF												HAL_GPIO_WritePin(LAS_PWM2_GPIO_Port, LAS_PWM2_Pin, GPIO_PIN_RESET)
#define SET_LASER_CH3_ON												HAL_GPIO_WritePin(LAS_PWM3_GPIO_Port, LAS_PWM3_Pin, GPIO_PIN_SET)
#define SET_LASER_CH3_OFF												HAL_GPIO_WritePin(LAS_PWM3_GPIO_Port, LAS_PWM3_Pin, GPIO_PIN_RESET)
#define FLIP_LASER_CH0													HAL_GPIO_TogglePin(LAS_PWM0_GPIO_Port, LAS_PWM0_Pin)
#define FLIP_LASER_CH1													HAL_GPIO_TogglePin(LAS_PWM1_GPIO_Port, LAS_PWM1_Pin)
#define FLIP_LASER_CH2													HAL_GPIO_TogglePin(LAS_PWM2_GPIO_Port, LAS_PWM2_Pin)
#define FLIP_LASER_CH3													HAL_GPIO_TogglePin(LAS_PWM3_GPIO_Port, LAS_PWM3_Pin)
#define GET_LASER_CH0														HAL_GPIO_ReadPin(LAS_PWM0_GPIO_Port, LAS_PWM0_Pin)
#define GET_LASER_CH1														HAL_GPIO_ReadPin(LAS_PWM1_GPIO_Port, LAS_PWM1_Pin)
#define GET_LASER_CH2														HAL_GPIO_ReadPin(LAS_PWM2_GPIO_Port, LAS_PWM2_Pin)
#define GET_LASER_CH3														HAL_GPIO_ReadPin(LAS_PWM3_GPIO_Port, LAS_PWM3_Pin)
#define SET_SPEAKER_ON													HAL_GPIO_WritePin(SPK_EN_GPIO_Port, SPK_EN_Pin, GPIO_PIN_RESET)
#define SET_SPEAKER_OFF													HAL_GPIO_WritePin(SPK_EN_GPIO_Port, SPK_EN_Pin, GPIO_PIN_SET)
#define SET_AIM_ON										   				HAL_GPIO_WritePin(LAS_AIM_GPIO_Port, LAS_AIM_Pin, GPIO_PIN_SET)
#define SET_AIM_OFF															HAL_GPIO_WritePin(LAS_AIM_GPIO_Port, LAS_AIM_Pin, GPIO_PIN_RESET)
#define SET_FAN_ON															HAL_GPIO_WritePin(LAS_FAN_GPIO_Port, LAS_FAN_Pin, GPIO_PIN_SET)
#define SET_FAN_OFF															HAL_GPIO_WritePin(LAS_FAN_GPIO_Port, LAS_FAN_Pin, GPIO_PIN_RESET)
#define SET_TEC_ON															HAL_GPIO_WritePin(LAS_TEC_GPIO_Port, LAS_TEC_Pin, GPIO_PIN_SET)
#define SET_TEC_OFF															HAL_GPIO_WritePin(LAS_TEC_GPIO_Port, LAS_TEC_Pin, GPIO_PIN_RESET)
/*****************************************************************************/
#define BT_STATE_IDLE														0//����
#define BT_STATE_LOAD_FWINFO										1//EPROM����̼���Ϣ
#define BT_STATE_USBHOST_INIT										2//FATFS ��ʼ��
#define BT_STATE_WAIT_UDISK											3//�ȴ�USB DISK����
#define BT_STATE_READ_CFG												4//��ȡ�����ļ�
#define BT_STATE_UPDATE_MCU_BOT									5//����BOOTLOAD
#define BT_STATE_UPDATE_MCU_APP									6//���µ�Ƭ��Ӧ�ù̼�
#define BT_STATE_UPDATE_LCD_APP									7//������ĻӦ�ù̼�
#define BT_STATE_UPDATE_BOTH_APP								8//���µ�Ƭ�������̼�
#define BT_STATE_UPDATE_EPROM										9//����UDISK->EPROM
#define BT_STATE_DUMP_EPROM											10//����ȫ��EPROM��UDISK
#define BT_STATE_CLEAT_ALL											11//���FLASH��EPROMȫ��
#define BT_STATE_RESET													90//����
#define BT_STATE_RUN_APP												99//��ת��APPӦ�ó���

#define BT_FAIL_READ_CFG												'0'//��U�̶�ȡCFGʧ��
#define BT_FAIL_READ_LMCU_APP										'1'//��U�̶�ȡMCU APPʧ��
#define BT_FAIL_READ_LCD_APP										'2'//��U�̶�ȡLCD APPʧ��
#define BT_FAIL_READ_EPROM_BIN									'3'//��U�̶�ȡEPROM BINʧ��
#define BT_FAIL_WRITE_EPROM_BIN									'4'//��U��д��EPROM BINʧ��
#define BT_FAIL_ERASE_MCU_APP										'5'//����FLASHʧ��
#define BT_FAIL_READ_EPROM											'6'//��ȡEPROMʧ��
#define BT_FAIL_WRITE_EPROM											'7'//д��EPROMʧ��
#define BT_FAIL_LMCU_APP_CHECK									'8'//ld_mcu.bin CRC������
#define BT_FAIL_LMCU_BOT_CHECK									'9'//ld_bot.bin CRC������
#define BT_FAIL_LCD_APP_CHECK										'A'//ld_lcd.bin CRC������
#define BT_FAIL_CHECKSUM_MCU_APP_FLASH					'B'//У�� mcu app ����
#define BT_FAIL_WRITE_MCU_APP_FLASH							'C'//дmcu app flash����
#define BT_FAIL_LCD_NOT_RESPOND									'D'//LCD����ͨ�ų�ʱ�����
#define BT_FAIL_LCD_DOWNLOAD										'E'//LCD����ʧ��
#define BT_FAIL_VECTOR_TABLE_INVALID						'F'//APP ���������
#define BT_FAIL_CHECK_BLANK											'G'//FLASH��մ���
#define BT_DONE_CLEAR_ALL												'H'//FLASH��EPROM������
#define BT_DONE_UPDATE_EPROM										'I'//����EPROM���
#define BT_DONE_DUMP_EPROM											'J'//����EPROM���
#define BT_FAIL_LCD_RESPOND_ERROR								'K'//LCD ��Ӧ����
/*****************************************************************************/
#define GDDC_UART_HANDLE												huart4
#define GDDC_UART_IRQ														UART4_IRQn
#define GDDC_RX_BUF_SIZE												128
#define GDDC_TX_BUF_SIZE												(2048 + 4)
#define GDDC_HEX 																0
#define GDDC_DEC 																1 
#define GDDC_CHR 																2
#define GDDC_RX_TIMEOUT													0xFFFF
#define GDDC_TX_TIMEOUT													0xFFFF
#define GDDC_RETRY_TIMES												10//�������Դ���
/*****************************************************************************/
typedef enum {
	CLEAR_EPROM_ALL 														= 0x01,
	CLEAR_EPROM_NVRAM														= 0x02,
	CLEAR_EPROM_FDRAM														= 0x03,
	CLEAR_EPROM_MCU_FIRMWARE_CRC								= 0x04,
	CLEAR_EPROM_LCD_FIRMWARE_CRC								= 0x05,
	CLEAR_EPROM_DEVICE_CONFIG										= 0x06,
	CLEAR_EPROM_LOG_INFO												= 0x07
}clarmEpromCmd_t;
/*****************************************************************************/
extern UART_HandleTypeDef huart1;//����
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim2;//FAN PWM
extern TIM_HandleTypeDef htim7;//DAC DMA ��ʱ��
extern TIM_HandleTypeDef htim10;//Laser Timer
extern TIM_HandleTypeDef htim12;//FAN PWM
extern TIM_HandleTypeDef htim14;//sPlc Timer
extern I2C_HandleTypeDef hi2c1;
extern USBH_HandleTypeDef hUsbHostFS;
/*****************************************************************************/
uint32_t crcEpromMcu, crcEpromLcd;//EPROM�д����CRC��¼ֵ
uint32_t TmpReadSize = 0x00;
uint32_t RamAddress = 0x00;
static __IO uint32_t LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};//�ļ���д����
/*****************************************************************************/
const char BootLoadMainVer __attribute__((at(BOOTLOAD_MAIN_ADDRESS)))   		= '1';
const char BootLoadMinorVer __attribute__((at(BOOTLAOD_MINOR_ADDRESS)))  		= '2';
/*****************************************************************************/
uint8_t cmdShakeHandOp[] = {0xEE,0x04,0xFF,0xFC,0xFF,0xFF};
uint8_t cmdShakeHandRespondOp[] = {0xEE,0x55,0xFF,0xFC,0xFF,0xFF};
	//��λ���ָ�
uint8_t cmdResetOp[] = {0x61,0x78,0x72,0x63, 0x65,0x6b,0x67,0x64, 0x79,0x68,0x74,0x73, 0x75,0x6e,0x71,0x77, 0x70,0x6a,0x62,0x76, 0x69,0x66,0x6f,0x6d, 0x7a,0x6c};
	//��ʽ���ļ�ϵͳ
uint8_t cmdFormatOp[] = {0xee,0xab,0xba,0xaa,0xbb,0x36,0x3f,0xff,0xfc,0xff,0xff};         
uint8_t cmdResetHmiOp[] = {0xEE,0x07,0x35,0x5A,0x53,0xA5,0xFF,0xFC,0xFF,0xFF};
uint8_t gddcRxBuf[GDDC_RX_BUF_SIZE];//��Ļ���ڽ��ջ�����
uint8_t gddcTxBuf[GDDC_TX_BUF_SIZE];//��Ļ���ڷ��ͻ�����
uint16_t gddcRxIndex, gddcTxIndex;//���ͽ���ָ��
/*****************************************************************************/
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
/*****************************************************************************/
FRESULT retUsbH;
FATFS	USBH_fatfs;
FIL LogFile;//FATFS File Object ��¼��Ϣ
FIL CfgFile;//FATFS File Object ���������Ϣ
FIL McuFile;//FATFS File Object ��Ƭ���̼�
FIL LcdFile;//FATFS File Object ��Ļ�̼�
FIL BotFile;//FATFS File Object BOOTLOAD�̼�
FIL SepromFile;//FATFS File Object EPROM->UDISK
FIL LepromFile;//FATFS File Object UDISK->EPROM
DIR	FileDir;//FATFS �ļ�Ŀ¼
FILINFO FileInfo;//FATFS �ļ���Ϣ
/*****************************************************************************/
static uint8_t bootLoadState;
static uint8_t usbReady;//USB DISK����
static uint32_t crcFlash, crcUdisk;
int32_t releaseTime0, releaseTime1, overTime, releaseCounter;
uint32_t JumpAddress;
pFunction Jump_To_Application;
/*****************************************************************************/
void resetInit(void);
static void bootLoadFailHandler(uint8_t ftype);//�������ϳ���
static void prepareUpdateLcd(void);//LCDԶ������׼��
static uint32_t getOriginAppCrc(void);
static uint32_t getNewMcuAppCrc(void);
static uint32_t getNewLcdAppCrc(char* filePath);//��ȡ�ļ�У���롡CRC32
static uint32_t updateMcuApp(void);
static uint32_t updateLcdApp(char* filePath);//����LCD APP
static void DBGU_Printk(uint8_t *buffer);
static void DBGU_Printk_num(uint8_t *buffer, uint16_t datanum);
static void dp_display_text(uint8_t *text);
static void dp_display_text_num(uint8_t *text,uint16_t datanum);	
static void dp_display_value(uint32_t value,int descriptive);
static void dp_display_array(uint8_t *value,int bytes, int descriptive);	
static void clearFlash(void);
static void updateEprom(void);
static void dumpEprom(void);
static HAL_StatusTypeDef epromReadByte(uint16_t ReadAddr, uint8_t *rdat);//��AT24CXXָ����ַ����һ������
static HAL_StatusTypeDef epromReadHword(uint16_t ReadAddr, uint16_t *rdat);//��AT24CXX�����ָ����ַ��ʼ����16λ��
static HAL_StatusTypeDef epromReadDword(uint16_t ReadAddr, uint32_t *rdat);////��AT24CXX�����ָ����ַ��ʼ����32λ��
static HAL_StatusTypeDef epromWriteByte(uint16_t WriteAddr, uint8_t *wdat);//��AT24CXXָ����ַд��8λ����
static HAL_StatusTypeDef epromWriteHword(uint16_t WriteAddr, uint16_t *wdat);//��AT24CXX�����ָ����ַ��ʼд��16λ��
static HAL_StatusTypeDef epromWriteDword(uint16_t WriteAddr, uint32_t *wdat);//��AT24CXX�����ָ����ַ��ʼд��32λ��
static HAL_StatusTypeDef epromRead(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
static HAL_StatusTypeDef epromWrite(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);
static uint8_t checkBlank(uint32_t adr, uint32_t size);//MCU Flash ���
static void clearEprom(clarmEpromCmd_t cmd);//���EPROM����
static void listEpromTable(void);
static uint8_t cmpByte(uint8_t *psrc, uint8_t *pdist, uint16_t len);
static FRESULT crcLcdFile(char* scanPath);
static FRESULT updateLcdFile(char* scanPath);
/******************************************************************************/
static uint8_t cmpByte(uint8_t *psrc, uint8_t *pdist, uint16_t len){
	uint16_t i;
	for(i = 0;i < len;i ++){
		if(*(psrc + i) != *(pdist + i)){
			return false;
		}
	}
	return true;
}

void bootLoadInit(void){//���������ʼ��
	SET_SPEAKER_OFF;//�رշ�����
	SET_AIM_OFF;//�ر�ָʾ����
	SET_FAN_OFF;//�򿪼�������ȴ����
	SET_TEC_OFF;//�ر�����
	//�ر����м���
	SET_LASER_CH0_OFF;
	SET_LASER_CH1_OFF;
	SET_LASER_CH2_OFF;
	SET_LASER_CH3_OFF;
	//�ر�����LED
	SET_RED_LED_OFF;
	SET_GREEN_LED_OFF;
	SET_BLUE_LED_OFF;
	SET_TICK_LED_OFF;
	SET_ERR_LED_OFF;
	//R-G-Y��ˮ
	//R
	SET_RED_LED_ON;
	SET_GREEN_LED_OFF;
	SET_BLUE_LED_OFF;
	HAL_Delay(500);
	//G
	SET_RED_LED_OFF;
	SET_GREEN_LED_ON;
	SET_BLUE_LED_OFF;
	HAL_Delay(500);
	//Y
	SET_RED_LED_OFF;
	SET_GREEN_LED_OFF;
	SET_BLUE_LED_ON;
	HAL_Delay(500);
	//G
	SET_RED_LED_OFF;
	SET_GREEN_LED_ON;
	SET_BLUE_LED_OFF;
	overTime = HAL_GetTick() + CONFIG_JUMP_DELAY;
	releaseTime0 = 0;
	releaseTime1 = 0;
	usbReady = FALSE;
	bootLoadState = BT_STATE_IDLE; 
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");   
	//��ʾ����IO״̬
	if(GET_ESTOP_NC == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->ESTOP_NC      = Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->ESTOP_NC      = Close!\n");
	}
	if(GET_FSWITCH_NC == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->FSWITCH_NC    = Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->FSWITCH_NC    = Close!\n");
	}
	if(GET_FSWITCH_NO == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->FSWITCH_NO    = Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->FSWITCH_NO    = Close!\n");
	}
	if(GET_INTERLOCK_NC == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->INTERLOCK     = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->INTERLOCK     = Close!\n");
	}
	//��ʾ���IO״̬
	if(GET_LASER_CH0 == GPIO_PIN_SET){//LPA_PWM0
		printf("Bootloader:OUTPUT->LAS_PWM0     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LAS_PWM0     = Low!\n");
	}
	if(GET_LASER_CH1 == GPIO_PIN_SET){//LPA_PWM1
		printf("Bootloader:OUTPUT->LAS_PWM1     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LAS_PWM1     = Low!\n");
	}
	if(GET_LASER_CH2 == GPIO_PIN_SET){//LPB_PWM2
		printf("Bootloader:OUTPUT->LAS_PWM2     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LAS_PWM2     = Low!\n");
	}
	if(GET_LASER_CH3 == GPIO_PIN_SET){//LPB_PWM3
		printf("Bootloader:OUTPUT->LAS_PWM3     = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LAS_PWM3     = Low!\n");
	}
	
	printf("Bootloader:OUTPUT->AIM_PWM				= OFF!\n");
	printf("Bootloader:OUTPUT->LAS_FAN				= OFF!\n");
	printf("Bootloader:OUTPUT->LAS_TEC				= OFF!\n");
	HAL_Delay(10);
	
}
void bootLoadProcess(void){//bootload ִ�г���
	HAL_StatusTypeDef ret;
	uint8_t fileBuff[256];
	uint32_t brByte;//ʵ�ʶ�ȡ���ֽ���
	crcFlash = 0;
	crcUdisk = 0;;
	switch(bootLoadState){
		case BT_STATE_IDLE:{//�����ȴ�U��ʶ��     
			SET_AIM_OFF;
			SET_FAN_OFF;		
			SET_RED_LED_OFF;
			SET_GREEN_LED_OFF;
			SET_BLUE_LED_OFF;
			printf("\n\n\n\n");
			printf("Bootloader:Start...............\n");
			listEpromTable();
			readStm32UniqueID();
			crcEpromMcu = 0xFFFFFFFF;
			ret = epromReadDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);//��EPROM�����豸����
			if(ret == HAL_OK){
				printf("Bootloader:Read eprom MCU FW CRC32:0x%08X\n", crcEpromMcu);
			}
			else{
				printf("Bootloader:Read eprom MCU FW CRC32 fail!\n");
			}
			crcEpromLcd = 0xFFFFFFFF;
			ret = epromReadDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);//��EPROM�����豸����
			if(ret == HAL_OK){
				printf("Bootloader:Read eprom LCD FW CRC32:0x%08X\n", crcEpromLcd);
			}
			else{
				printf("Bootloader:Read eprom LCD FW CRC32 fail!\n");
			}
			printf("Bootloader:Version->%c.%c\n", BootLoadMainVer, BootLoadMinorVer);
			printf("Bootloader:UniqueID->0x%08X%08X%08X\n", UniqueId[0], UniqueId[1], UniqueId[2]);
			printf("Bootloader:Mcu flash size->%d Kbytes\n", cpuGetFlashSize());
			printf("Bootloader:Build->%s:%s\n", __DATE__, __TIME__);
			printf("Bootloader:Bootload Start  :0x%08X,End:0x%08X,Size:0x%08X\n", BOOTLOADER_FLASH_START_ADDRESS, BOOTLOADER_FLASH_END_ADDRESS ,BOOTLOADER_FLASH_SIZE);
			printf("Bootloader:Applicent Start :0x%08X,End:0x%08X,Size:0x%08X\n", APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_END_ADDRESS, APPLICATION_FLASH_SIZE);
			if(	(GET_INTERLOCK_NC == GPIO_PIN_SET) &&//��ȫ����δ����
				(GET_FSWITCH_NC == GPIO_PIN_RESET) &&//��̤����
				(GET_FSWITCH_NO == GPIO_PIN_RESET)){//��̤����		
				SET_FAN_ON;
				SET_RED_LED_ON;
				SET_GREEN_LED_OFF;
				SET_BLUE_LED_OFF;	
				HAL_Delay(500);
				SET_RED_LED_OFF;
				SET_GREEN_LED_ON;
				SET_BLUE_LED_OFF;	
				HAL_Delay(500);
				SET_RED_LED_OFF;
				SET_GREEN_LED_OFF;
				SET_BLUE_LED_ON;	
				HAL_Delay(500);	
				SET_RED_LED_OFF;
				SET_GREEN_LED_OFF;
				SET_BLUE_LED_OFF;
				bootLoadState = BT_STATE_USBHOST_INIT;//����USB����APP����
			}
			else{//��ȫ��������
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
			SET_RED_LED_ON;
			retUsbH = f_open(&CfgFile, CFG_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);//��ȡ�����Ϣ�ļ�
			SET_RED_LED_OFF;
			if(retUsbH != FR_OK){//��ȡʧ�������̼�����ֱ�����г���
				printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", CFG_FIRMWARE_FILENAME, retUsbH);
				bootLoadState = BT_STATE_RUN_APP;//��ת������MCU APP�̼�
			}
			else{//��ȡ�ɹ�����ļ�����
				printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", CFG_FIRMWARE_FILENAME, retUsbH);
				f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
				SET_RED_LED_ON;
				retUsbH = f_read(&CfgFile, fileBuff, 5, &brByte);
				SET_RED_LED_OFF;
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
			printf("Bootloader:MCU crcFlash:%08XH,crcUdisk:%08XH!\n", crcFlash, crcUdisk);
			if((crcUdisk == crcEpromMcu) && (crcFlash == crcEpromMcu)){//У������ͬ��������
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
				clearEprom(CLEAR_EPROM_NVRAM);//���NVRAM���索����
				clearEprom(CLEAR_EPROM_MCU_FIRMWARE_CRC);
				epromWriteDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);
				printf("Bootloader:Update new crc32 sucess,0x08%XH\n", crcEpromMcu);
			}
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_LCD_APP:{//����LCDӦ�ó���			
			crcUdisk = 0;
			strcpy((char*)fileBuff, "/private");
			crcLcdFile((char*)fileBuff);//ɨ���ļ�
			printf("Bootloader:LCD crcLcd:%08XH,crcUdisk:%08XH!\n", crcEpromLcd, crcUdisk);
			if(crcUdisk == crcEpromLcd){//У������ͬ��������
				printf("Bootloader:Check lcd app crc same,skip!\n");
				bootLoadState = BT_STATE_RUN_APP;
				break;
			}
			prepareUpdateLcd();//LCDԶ������׼��
			
			crcUdisk = 0;
			strcpy((char*)fileBuff, "/private");
			updateLcdFile((char*)fileBuff);
			
			crcEpromLcd = crcUdisk;//����EPROM��LCD APP CRCֵ
			clearEprom(CLEAR_EPROM_LCD_FIRMWARE_CRC);
			epromWriteDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);
			printf("Bootloader:Update lcd app new crc32 sucess,wait 60s lcd upgrade done\n");
			//�ȴ�60�� LCD FLASHд����ɺ�����
			//����HMI
			dp_display_text_num(cmdResetHmiOp, strlen((char*)cmdResetHmiOp));	
			HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_BOTH_APP:{//����ȫ��Ӧ�ó���
			//MCU ����
			crcFlash = getOriginAppCrc();//����FLASH��APP�̼�CRC32
			crcUdisk = getNewMcuAppCrc();//����U����MCU APP�̼�CRC32
			printf("Bootloader:MCU crcFlash:%08XH,crcUdisk:%08XH,crcEprom:%08XH\n", crcFlash, crcUdisk, crcEpromMcu);
			if((crcUdisk == crcEpromMcu) && (crcFlash == crcEpromMcu)){//У������ͬ��������
				printf("Bootloader:Check mcu app crc same,skip!\n");
			}
			else{//��U�̸��¹̼�
				crcUdisk = updateMcuApp();	
				crcFlash = getOriginAppCrc();//У��MCU FLASH
				if(crcUdisk != crcFlash){
					bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
				}
				crcEpromMcu = crcFlash;//CRCֵд��EPROM
				printf("Bootloader:Check mcu app sucess.\n");
				clearEprom(CLEAR_EPROM_NVRAM);
				clearEprom(CLEAR_EPROM_MCU_FIRMWARE_CRC);
				epromWriteDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);
				printf("Bootloader:Update mcu app new crc32 sucess.\n");
			}
			
			//LCD ����
			crcUdisk = 0;
			strcpy((char*)fileBuff, "/private");
			crcLcdFile((char*)fileBuff);//ɨ���ļ�
			printf("Bootloader:LCD crcLcd:%08XH,crcUdisk:%08XH!\n", crcEpromLcd, crcUdisk);
			if(crcUdisk == crcEpromLcd){//У������ͬ��������
				printf("Bootloader:Check lcd app crc same,skip!\n");
				bootLoadState = BT_STATE_RUN_APP;
				break;
			}
			prepareUpdateLcd();//LCDԶ������׼��
			
			crcUdisk = 0;
			strcpy((char*)fileBuff, "/private");
			updateLcdFile((char*)fileBuff);
			
			crcEpromLcd = crcUdisk;//����EPROM��LCD APP CRCֵ
			clearEprom(CLEAR_EPROM_LCD_FIRMWARE_CRC);
			epromWriteDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);
			printf("Bootloader:Update lcd app new crc32 sucess,wait 60s lcd upgrade done\n");
			//�ȴ�60�� LCD FLASHд����ɺ�����
			//����HMI
			dp_display_text_num(cmdResetHmiOp, strlen((char*)cmdResetHmiOp));	
			HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
			bootLoadState = BT_STATE_RESET;//����APP
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
				SET_RED_LED_OFF;
				SET_GREEN_LED_OFF;
				SET_BLUE_LED_OFF;
				SET_FAN_OFF;
				__disable_irq();
				SysTick->CTRL = 0;//�ؼ�����
				//�ر��ж�                                    				
				HAL_NVIC_DisableIRQ(SysTick_IRQn); 
				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
				HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
				HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
				Jump_To_Application();
			}
			bootLoadFailHandler(BT_FAIL_VECTOR_TABLE_INVALID);
		}
		default:break;
	}
}

static void bootLoadFailHandler(uint8_t ftype){//�����������
	MX_DriverVbusFS(FALSE);//�ر�USB VBUS
	printf("Bootloader:SYS_ERR_LED->On!\n");
	SET_RED_LED_ON;
	SET_GREEN_LED_OFF;
	SET_BLUE_LED_OFF;
	switch(ftype){
		case BT_FAIL_READ_CFG:{//��U�̶�ȡCFGʧ��
			printf("Bootloader:FailHandler,Read config file fail!.\n");
			while(1);
		}
		case BT_FAIL_READ_LMCU_APP:{//��U�̶�ȡMCU APPʧ��
			printf("Bootloader:FailHandler,Read mcu firmware fail!.\n");
			while(1);
		}
		case BT_FAIL_READ_LCD_APP:{//��U�̶�ȡLCD APPʧ��
			printf("Bootloader:FailHandler,Read lcd firmware fail!.\n");
			while(1);
		}
		case BT_FAIL_ERASE_MCU_APP:{//����MCU APP FLASH����ʧ��
			printf("Bootloader:FailHandler,Erase mcu application fail\n");
			while(1);
		}
		case BT_FAIL_READ_EPROM_BIN:{
			printf("Bootloader:FailHandler,Read eprom file fail!\n");
			while(1);
		}
		case BT_FAIL_WRITE_EPROM_BIN:{
			printf("Bootloader:FailHandler,Write eprom file fail!\n");
			while(1);
		}
		case BT_FAIL_LMCU_APP_CHECK:{//lmcu.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LMCU_FIRMWARE_FILENAME);
			while(1);
		}
		case BT_FAIL_LCD_APP_CHECK:{//llcd.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LLCD_FIRMWARE_FILENAME);
			while(1);
		}
		case BT_FAIL_CHECKSUM_MCU_APP_FLASH:{//У�� lmcu.bin ����
			printf("Bootloader:FailHandler,Verify %s fail!.\n", LMCU_FIRMWARE_FILENAME);
			while(1);
		}
		case BT_FAIL_LCD_NOT_RESPOND:{//LCD ��������Ӧ�����
			printf("Bootloader:FailHandler,LCD is not responsed!.\n");
			while(1);
		}
		case BT_FAIL_LCD_DOWNLOAD:{//LCD ��������Ӧ
			printf("Bootloader:FailHandler,LCD download fail!.\n");
			while(1);
		}
		case BT_FAIL_VECTOR_TABLE_INVALID:{//APP��Ч������
			printf("Bootloader:FailHandler,App vector table invalid.\n");
			while(1);
		}
		case BT_FAIL_CHECK_BLANK:{//FLASH ��մ���
			printf("Bootloader:FailHandler,Flash is not blank!.\n");
			while(1);
		}
		case BT_DONE_CLEAR_ALL:{
			printf("Bootloader:DoneHandler,Flash and Eprom easer done!.\n");
			while(1);
		}
		case BT_DONE_UPDATE_EPROM:{
			printf("Bootloader:DoneHandler,Update eprom form udisk done!.\n");
			while(1);
		}
		case BT_DONE_DUMP_EPROM:{
			printf("Bootloader:DoneHandler,Dump eprom to udisk done!\n");
			while(1);
		}
		default:{
			printf("Bootloader:DoneHandler,unknown error!\n");
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
		SET_RED_LED_ON;
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&McuFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		SET_RED_LED_OFF;
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
static uint32_t getNewLcdAppCrc(char* filePath){//��ȡ������LCD APP CRC16
	uint32_t crc32;
	uint8_t readflag = TRUE;
	uint16_t bytesread;//ʵ���ļ���ȡ�ֽ���
	SET_RED_LED_ON;
	retUsbH = f_open(&LcdFile, filePath, FA_OPEN_EXISTING | FA_READ);
	SET_RED_LED_OFF;
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LCD_APP);			
	}
	f_lseek(&LcdFile, 0);//��ȡָ���ƶ�����ͷ
	crc32 = 0;
	crc32Clear();
	while(readflag){
		/* Read maximum 512 Kbyte from the selected file */
		SET_RED_LED_ON;
		f_read(&LcdFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		SET_RED_LED_OFF;
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
		SET_RED_LED_ON;
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&McuFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		SET_RED_LED_OFF;
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		/* Temp variable */
		TmpReadSize = bytesread;
		/* The read data < "BUFFER_SIZE" Kbyte */
		if(TmpReadSize < BUFFER_SIZE){
			readflag = FALSE;
		}
		/* Program flash memory */
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

static void prepareUpdateLcd(void){//LCDԶ������׼��        
	memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));
	dp_display_text_num(cmdShakeHandOp, strlen((char*)cmdShakeHandOp));		
	HAL_UART_Receive(&huart4, gddcRxBuf, 6, 1000);//��ѯ���ڽ������� ��ʱ1000       
	if(cmpByte(gddcRxBuf, cmdShakeHandRespondOp, sizeof(cmdShakeHandRespondOp))){
		printf("Bootloader:DoneHandler,lcd shake hand respond!\n");
	}
	dp_display_text_num(cmdResetOp, strlen((char*)cmdResetOp));//�л��ɴ��ָ�
	HAL_Delay(10);
	dp_display_text_num(cmdFormatOp, strlen((char*)cmdFormatOp));//��ʽ���ļ�ϵͳ
	HAL_Delay(3000);
}
static uint32_t updateLcdApp(char* filePath){//����LCD APP�����ļ�
	UART_HandleTypeDef *puart;
	HAL_StatusTypeDef uRet;
	uint32_t crc32, i;
	uint8_t signName;  
	uint8_t cmdResponse[] = {0xEE,0xFB,0x01,0xFF,0xFC,0xFF,0xFF};//��Ļ��Ӧ
	uint32_t bufIndex;
	uint32_t fileSize;//�ļ���С
	uint32_t blockSize = 2048;//��Ҫ��ȡ�����ݰ���С
	uint32_t transferByte;//��ȡ�ֽ���
	uint32_t actualByte;//ʵ�ʶ�ȡ���ֽ���
	uint32_t fileIndex;
	uint16_t checkSum;//У����
	uint8_t lcdRetry;//�����ظ�����
	char* fileCmdPath;    
	puart = &GDDC_UART_HANDLE;
	retUsbH = f_open(&LcdFile, filePath, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LCD_APP);
	}
	f_lseek(&LcdFile, 0);//��ȡָ���ƶ�����ͷ
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", filePath, retUsbH);
  fileSize =  f_size(&LcdFile);
	gddcTxBuf[0] = 0xEE;
	gddcTxBuf[1] = 0xFB;
	//���ذ���С
	gddcTxBuf[2] = ((blockSize + 4) >> 8) & 0xFF;
	gddcTxBuf[3] = ((blockSize + 4) >> 0) & 0xFF;
	//�����ʲ��ı�
	gddcTxBuf[4] = 0;
	gddcTxBuf[5] = 0;
	//�ļ���С
	gddcTxBuf[6] = (fileSize >> 24) & 0xff;
	gddcTxBuf[7] = (fileSize >> 16) & 0xff;
	gddcTxBuf[8] = (fileSize >>  8) & 0xff;
	gddcTxBuf[9] = (fileSize) & 0xff;
	//�ļ���
	gddcTxBuf[10] = 0x33;
	gddcTxBuf[11] = 0x3A;
	//���������е�·����ʵ��·���ĵ�һ����/����ʼ�����ԡ�private��
  //����ʵ��·��Ϊ private/bin/image.bin
  //�����е�·��Ϊ 3:/bin/image.bin
  fileCmdPath = strchr((filePath + 1),'/');                           
	for(i = 0;i < strlen(fileCmdPath);i++){
		if(fileCmdPath[i] == '/'){
			gddcTxBuf[12 + i] = 0x5C;
		}
		else{
			gddcTxBuf[12 + i] = fileCmdPath[i];
		}
	}
	gddcTxBuf[strlen(fileCmdPath) + 12] = 0xFF;
  gddcTxBuf[strlen(fileCmdPath) + 13] = 0xFC;
  gddcTxBuf[strlen(fileCmdPath) + 14] = 0xFF;
  gddcTxBuf[strlen(fileCmdPath) + 15] = 0xFF;

  //������������
	printf("Bootloader->updateLcdApp:Send download command.\n");
	lcdRetry = 0;
	do{
		lcdRetry++;
		uRet = HAL_UART_Receive(puart, gddcRxBuf, sizeof(gddcRxBuf), 1);//��ս��ջ�����
		memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));	
		gddcRxIndex = 0;
		dp_display_text_num(gddcTxBuf, (strlen(fileCmdPath) + 16));//����õ������
		uRet = HAL_UART_Receive(puart, gddcRxBuf, 7, 5000);//���ڽ������� ��ʱ10000mS
		HAL_Delay(10);
		if(cmpByte(gddcRxBuf, cmdResponse, 7) == true){
			break;
		}
		else{
			HAL_Delay(10);
			if(lcdRetry > GDDC_RETRY_TIMES){//���ʹ�����ʱ
				bootLoadFailHandler(BT_FAIL_LCD_DOWNLOAD);
			}
		}
	}while(1);
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
    transferByte = blockSize;
		if(fileIndex + transferByte > fileSize){//��ȡ2048���ֽڵ��������ļ���С
			transferByte = fileSize - fileIndex;
		}
		SET_RED_LED_ON;
		retUsbH = f_read(&LcdFile, &gddcTxBuf[2], transferByte, &actualByte);
		SET_RED_LED_OFF;
		crc32 = crc32Calculate(&gddcTxBuf[2], actualByte);
		if(retUsbH != FR_OK){
			bootLoadFailHandler(BT_FAIL_READ_LCD_APP);
    }
    //����У���
		checkSum = 0x0;
		bufIndex = 0;
		do{
			checkSum += (uint16_t)gddcTxBuf[bufIndex];
			bufIndex ++;
		}while(bufIndex < 2050);
		checkSum = (uint16_t)~(checkSum);//ȡ������ڰ�ĩβ
		gddcTxBuf[2050] = (checkSum >> 8) & 0xFF;
		gddcTxBuf[2051] = (checkSum >> 0) & 0xFF;   
		lcdRetry = 0;
		do{
			lcdRetry ++;
			//�������ݰ���ֱ���ɹ��������������	
			printf("Bootloader->updateLcdApp:Send file block at 0x%08XH,", fileIndex);
			HAL_UART_Receive(puart, gddcRxBuf, sizeof(gddcRxBuf), 1);//��ս��ջ�����
			memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));
			dp_display_text_num(gddcTxBuf, (blockSize + 4));//send data
			uRet = HAL_UART_Receive(puart, gddcRxBuf, 2, 3000);//��ѯ���ڽ������� ��ʱ3000            
			if(uRet == HAL_OK){
				if(gddcRxBuf[1] == (uint8_t)(~(signName + 1)) || gddcRxBuf[0] == (signName+1) && gddcRxBuf[1] == (uint8_t)(~(signName+1))){//wait ack
					signName = signName + 1;
					printf("ok!\n");
					break;
				}
			}
			else{//��ʱ����ЧSN
				printf("timeout or invalid SN!\n");
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
				bootLoadFailHandler(BT_FAIL_READ_EPROM_BIN);
	}
	else{//��ȡ�ɹ�����ļ�����
		printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", LOAD_EPROM_FILENAME, retUsbH);
		f_lseek(&LepromFile, 0);//��ȡָ���ƶ�����ͷ
		SET_RED_LED_ON;
		retUsbH = f_read(&LepromFile, RAM_Buf, CONFIG_EPROM_SIZE, &brByte);
		SET_RED_LED_OFF;
		if((retUsbH != FR_OK) || (brByte !=  CONFIG_EPROM_SIZE)){
			bootLoadFailHandler(BT_FAIL_READ_EPROM_BIN);
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
		bootLoadFailHandler(BT_FAIL_WRITE_EPROM_BIN);
	}
	retUsbH = f_write(&SepromFile, RAM_Buf, CONFIG_EPROM_SIZE, &wrByte);
	if(retUsbH != FR_OK){//д��ʧ��
		bootLoadFailHandler(BT_FAIL_WRITE_EPROM_BIN);
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
	HAL_UART_Transmit(&GDDC_UART_HANDLE, (const uint8_t*)buffer, datanum, GDDC_TX_TIMEOUT);
}
static void dp_display_text(uint8_t *text){
    /* User Specific Code   */
    DBGU_Printk(text);
}
static void dp_display_text_num(uint8_t *text, uint16_t datanum){
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
int fputc(int ch,FILE *f){
	uint8_t dat;
	dat = (ch & 0xFF);
	HAL_UART_Transmit(&huart1, &dat, 1, 1000);
	return ch;
}

int fgetc(FILE *f){
	uint8_t dat;
	HAL_UART_Receive(&huart1, &dat, 1, 100);
	return (dat);
}

void _sys_exit(int x){
	x = x;
}

void _ttywrch(int ch){
	ch = ch;
}
/*****************************************************************************/
static void softDelayMs(uint16_t ms){//�����ʱ
	uint32_t i;
	for(i = 0;i < 1000;i ++){
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
}

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
static void SystemClock_Reset(void){//��λϵͳʱ��
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
	}
	return ret;
}
static HAL_StatusTypeDef epromReadHword(uint16_t ReadAddr, uint16_t *rdat){//��ָ����ַ��ʼ����16λ��
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����  	
	HAL_StatusTypeDef ret;
	if((ReadAddr + 1) > (CONFIG_EPROM_SIZE - 1)){//д��ַ��������
		ret = HAL_ERROR;
		return ret;
	}	
	ret = HAL_I2C_Mem_Read(&hi2c1, 
						   CONFIG_EPROM_READ_ADDR,
	                       ReadAddr,
	                       I2C_MEMADD_SIZE_16BIT,
	                       (uint8_t*)(rdat),
	                       2,
	                       CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);        //�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);          //������³�ʼ��I2C������
	}
	return ret;
}
static HAL_StatusTypeDef epromReadDword(uint16_t ReadAddr, uint32_t *rdat){////��ָ����ַ��ʼ����32λ��
//�ú������ڶ���32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����  	
	HAL_StatusTypeDef ret;
	if((ReadAddr + 3) > (CONFIG_EPROM_SIZE - 1)){//д��ַ��������
		ret = HAL_ERROR;
		return ret;
	}	
	ret = HAL_I2C_Mem_Read(&hi2c1, 
	                       CONFIG_EPROM_READ_ADDR,
	                       ReadAddr,
	                       I2C_MEMADD_SIZE_16BIT,
	                       (uint8_t*)(rdat),
	                       4,
	                       CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);        //�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);          //������³�ʼ��I2C������
	}
	return ret;
}
static HAL_StatusTypeDef epromWriteByte(uint16_t WriteAddr, uint8_t *wdat){//��ָ����ַд��8λ����
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
	                        (uint8_t*)(wdat), 
	                        1, 
	                        CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);//�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);//������³�ʼ��I2C������
	}
	return ret;
}
static HAL_StatusTypeDef epromWriteHword(uint16_t WriteAddr, uint16_t *wdat){//�ڵ�ָ����ַ��ʼд��16λ��
//�ú�������д��16bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
	HAL_StatusTypeDef ret;
	if((WriteAddr + 1) > (CONFIG_EPROM_SIZE - 1)){//д��ַ��������
		ret = HAL_ERROR;
		return ret;
	}
	ret = HAL_I2C_Mem_Write(&hi2c1, 
	                        CONFIG_EPROM_WRITE_ADDR, 
	                        WriteAddr, 
	                        I2C_MEMADD_SIZE_16BIT, 
	                        (uint8_t*)(wdat), 
	                        2, 
	                        CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);//�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);//������³�ʼ��I2C������	
	}
	return ret;
}
static HAL_StatusTypeDef epromWriteDword(uint16_t WriteAddr, uint32_t *wdat){//�ڵ�ָ����ַ��ʼд��32λ��
//�ú�������д��32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
	HAL_StatusTypeDef ret;
	if((WriteAddr + 3) >= (CONFIG_EPROM_SIZE - 1)){//д��ַ��������
		ret = HAL_ERROR;
		return ret;
	}
	ret = HAL_I2C_Mem_Write(&hi2c1, 
	                        CONFIG_EPROM_WRITE_ADDR, 
	                        WriteAddr, 
	                        I2C_MEMADD_SIZE_16BIT, 
	                        (uint8_t*)(wdat), 
	                        4, 
	                        CONFIG_EPROM_TIMEOUT);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);        //�ͷ�IO��ΪGPIO����λ���״̬��־
		ret = HAL_I2C_Init(&hi2c1);          //������³�ʼ��I2C������
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
	if((ReadAddr + NumToRead) > CONFIG_EPROM_SIZE){//����ַ��������
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
	if((WriteAddr + NumToWrite) > CONFIG_EPROM_SIZE){//����ַ��������
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
		}
	}
#if CONFIG_EPROM_WRITE_DELAY > 0
	HAL_Delay(CONFIG_EPROM_WRITE_DELAY);
#endif
	return ret;
}
/*****************************************************************************/
static void listEpromTable(void){//���EPROM�ֲ���
	printf("MR EPROM:0x%04X---0x%04X(size:%d)\n", (uint32_t)CONFIG_EPROM_MR_START, (uint32_t)CONFIG_EPROM_MR_END, (uint16_t)CONFIG_MRRAM_SIZE);
	printf("DM EPROM:0x%04X---0x%04X(size:%d)\n", (uint32_t)CONFIG_EPROM_DM_START, (uint32_t)CONFIG_EPROM_DM_END, (uint16_t)CONFIG_DMRAM_SIZE);
	printf("FD EPROM:0x%04X---0x%04X(size:%d)\n", (uint32_t)CONFIG_EPROM_FD_START, (uint32_t)CONFIG_EPROM_FD_END, (uint16_t)CONFIG_FDRAM_SIZE);
	
	printf("MR CRC EPROM:0x%04X---0x%04X\n", (uint32_t)CONFIG_EPROM_MR_CRC, (uint32_t)(CONFIG_EPROM_MR_CRC + 3));
	printf("DM CRC EPROM:0x%04X---0x%04X\n", (uint32_t)CONFIG_EPROM_DM_CRC, (uint32_t)(CONFIG_EPROM_DM_CRC + 3));
	printf("FD CRC EPROM:0x%04X---0x%04X\n", (uint32_t)CONFIG_EPROM_FD_CRC, (uint32_t)(CONFIG_EPROM_FD_CRC + 3));	
	printf("MCU CRC EPROM:0x%04X---0x%04X\n", (uint32_t)CONFIG_EPROM_MCU_FW_CRC, (uint32_t)(CONFIG_EPROM_MCU_FW_CRC + 3));
	printf("LCD CRC EPROM:0x%04X---0x%04X\n", (uint32_t)CONFIG_EPROM_LCD_FW_CRC, (uint32_t)(CONFIG_EPROM_LCD_FW_CRC + 3));
	
	printf("CONFIG EPROM:0x%04X---0x%04X(size:%d)\n", (uint32_t)CONFIG_EPROM_CONFIG_START, (uint32_t)CONFIG_EPROM_CONFIG_END, (uint16_t)(CONFIG_EPROM_CONFIG_END - CONFIG_EPROM_CONFIG_START + 1));
	printf("LOGINFO EPROM:0x%04X---0x%04X(size:%d)\n", (uint32_t)CONFIG_EPROM_LOGINFO_START,(uint32_t)CONFIG_EPROM_LOGINFO_END, (uint16_t)(CONFIG_EPROM_LOGINFO_END - CONFIG_EPROM_LOGINFO_START + 1));
}
static void clearEprom(clarmEpromCmd_t cmd){//���EPROM����
	uint8_t var = 0;
	uint32_t i;	
	switch(cmd){
		case CLEAR_EPROM_ALL:{
			for(i = 0;i < CONFIG_EPROM_SIZE;i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		case CLEAR_EPROM_NVRAM:{
			for(i = CONFIG_EPROM_MR_START; i <= CONFIG_EPROM_MR_END;i ++){
				epromWriteByte(i, &var);
			}
			for(i = CONFIG_EPROM_DM_START; i <= CONFIG_EPROM_DM_END;i ++){
				epromWriteByte(i, &var);
			}
			
			for(i = CONFIG_EPROM_MR_CRC; i <= (CONFIG_EPROM_MR_CRC + 3);i ++){
				epromWriteByte(i, &var);
			}
			
			for(i = CONFIG_EPROM_DM_CRC; i <= (CONFIG_EPROM_DM_CRC + 3);i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		case CLEAR_EPROM_FDRAM:{
			for(i = CONFIG_EPROM_FD_START; i <= CONFIG_EPROM_FD_END;i ++){
				epromWriteByte(i, &var);
			}			
			for(i = CONFIG_EPROM_FD_CRC; i <= (CONFIG_EPROM_FD_CRC + 3);i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		case CLEAR_EPROM_MCU_FIRMWARE_CRC:{
			for(i = CONFIG_EPROM_MCU_FW_CRC;i <= (CONFIG_EPROM_MCU_FW_CRC + 3);i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		case CLEAR_EPROM_LCD_FIRMWARE_CRC:{
			for(i = CONFIG_EPROM_LCD_FW_CRC;i <= (CONFIG_EPROM_LCD_FW_CRC + 3);i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		case CLEAR_EPROM_DEVICE_CONFIG:{
			for(i = CONFIG_EPROM_CONFIG_START;i <= CONFIG_EPROM_CONFIG_END;i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		case CLEAR_EPROM_LOG_INFO:{
			for(i = CONFIG_EPROM_LOGINFO_START;i <= CONFIG_EPROM_LOGINFO_END;i ++){
				epromWriteByte(i, &var);
			}
			break;
		}
		default:break;
	}
}
static uint8_t checkBlank(uint32_t adr, uint32_t size){//MCU Flash ���
	uint8_t val;
	uint32_t i;
	for(i = 0;i < size;i ++){
		val = *(__IO uint8_t*)(adr + i);
		if(val != 0xFF){
			return false;
		}
	}
	return true;
}

static FRESULT crcLcdFile(char* scanPath){//ɨ���ļ�����ȫ���ļ�������CRCֵ
	DIR memofsrcdir;
	DIR *srcdir;
	FILINFO menoffinfo;
	FILINFO *finfo;
	char fileName[256];
	char *fn;
	finfo=&menoffinfo; 
	srcdir=&memofsrcdir;  //ԴĿ¼
	retUsbH = f_opendir(srcdir, (const TCHAR*)scanPath);
	while(retUsbH == FR_OK){
		retUsbH = f_readdir(srcdir, finfo);
		if(retUsbH != FR_OK || finfo->fname[0] == 0){//��Ч�ļ�
			break;
		}
		if(finfo->fname[0] == '.'){
			continue;
		}
		fn = finfo->fname;
		memset(fileName, 0x0, sizeof(fileName));
		sprintf(fileName, "%s%c%s",scanPath , '/', fn);
		if(finfo->fattrib & AM_DIR){//�ļ���
			retUsbH = crcLcdFile(fileName);
			if(retUsbH != FR_OK){
				return retUsbH;
			}
		}
		else{
			printf("Bootloader:crc Lcd file:%s\n", fileName);
			crcUdisk += getNewLcdAppCrc(fileName);//����U����MCU APP�̼�CRC32
		}		
	}
	return retUsbH;
}	


static FRESULT updateLcdFile(char* scanPath){//ɨ���ļ�����ȫ���ļ����ϴ�
	DIR memofsrcdir;
	DIR *srcdir;
	FILINFO menoffinfo;
	FILINFO *finfo;
	char fileName[256];
	char *fn;
	finfo=&menoffinfo; 
	srcdir=&memofsrcdir;  //ԴĿ¼
	retUsbH = f_opendir(srcdir, (const TCHAR*)scanPath);
	while(retUsbH == FR_OK){
		retUsbH = f_readdir(srcdir, finfo);
		if(retUsbH != FR_OK || finfo->fname[0] == 0){//��Ч�ļ�
			break;
		}
		if(finfo->fname[0] == '.'){
			continue;
		}
		fn = finfo->fname;
		memset(fileName, 0x0, sizeof(fileName));
		sprintf(fileName, "%s%c%s",scanPath , '/', fn);
		if(finfo->fattrib & AM_DIR){//�ļ���
			retUsbH = crcLcdFile(fileName);
			if(retUsbH != FR_OK){
				return retUsbH;
			}
		}
		else{
			printf("Bootloader:crc Lcd file:%s\n", fileName);
			crcUdisk += updateLcdApp(fileName);//����U����MCU APP�̼�CRC32
		}		
	}
	return retUsbH;
}

