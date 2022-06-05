#include "bootLoader.h"
/*****************************************************************************/
//SECTOR0->16K:BOOTLOADER
//SECTOR1->16K:BOOTLOADER
//SECTOR2->16K:BOOTLOADER
//SECTOR3->16K:BOOTLOADER
/*****************************************************************************/
#define BT_STATE_IDLE								0//����
#define BT_STATE_LOAD_FWINFO						1//EPROM����̼���Ϣ
#define BT_STATE_USBHOST_INIT						2//FATFS ��ʼ��
#define BT_STATE_WAIT_UDISK							3//�ȴ�USB DISK����
#define BT_STATE_READ_CFG							4//��ȡ�����ļ�
#define BT_STATE_UPDATE_MCU_BOT						5//����BOOTLOAD
#define BT_STATE_UPDATE_MCU_APP						6//���µ�Ƭ��Ӧ�ù̼�
#define BT_STATE_UPDATE_LCD_APP						7//������ĻӦ�ù̼�
#define BT_STATE_UPDATE_BOTH_APP					8//���µ�Ƭ�������̼�
#define BT_STATE_UPDATE_EPROM						9//����UDISK->EPROM
#define BT_STATE_DUMP_EPROM							10//����ȫ��EPROM��UDISK
#define BT_STATE_CLEAT_ALL							11//���FLASH��EPROMȫ��
#define BT_STATE_RESET								90//����
#define BT_STATE_RUN_APP							99//��ת��APPӦ�ó���
/*****************************************************************************/
#define BT_FAIL_READ_CFG							'0'//��U�̶�ȡCFGʧ��
#define BT_FAIL_READ_LMCU_APP						'1'//��U�̶�ȡMCU APPʧ��
#define BT_FAIL_READ_LLCD_APP						'2'//��U�̶�ȡLCD APPʧ��
#define BT_FAIL_READ_LEROM_BIN						'3'//��U�̶�ȡEPROM BINʧ��
#define BT_FAIL_WRITE_SEROM_BIN						'4'//��U��д��EPROM BINʧ��
#define BT_FAIL_ERASE_MCU_APP						'5'//����FLASHʧ��
#define BT_FAIL_READ_EPROM							'6'//��ȡEPROMʧ��
#define BT_FAIL_WRITE_EPROM							'7'//д��EPROMʧ��
#define BT_FAIL_LMCU_APP_CHECK						'8'//ld_mcu.bin CRC������
#define BT_FAIL_LMCU_BOT_CHECK						'9'//ld_bot.bin CRC������
#define BT_FAIL_LLCD_APP_CHECK						'A'//ld_lcd.bin CRC������
#define BT_FAIL_CHECKSUM_MCU_APP_FLASH				'B'//У�� mcu app ����
#define BT_FAIL_WRITE_MCU_APP_FLASH					'C'//дmcu app flash����
#define BT_FAIL_LCD_NOT_RESPOND						'D'//LCD����ͨ�ų�ʱ�����
#define BT_FAIL_LCD_DOWNLOAD						'E'//LCD����ʧ��
#define BT_FAIL_VECTOR_TABLE_INVALID				'F'//APP ���������
#define BT_FAIL_CHECK_BLANK							'G'//FLASH��մ���
#define BT_DONE_CLEAR_ALL							'H'//FLASH��EPROM������
#define BT_DONE_UPDATE_EPROM						'I'//����EPROM���
#define BT_DONE_DUMP_EPROM							'J'//����EPROM���
/*****************************************************************************/
#define GDDC_LCD_NORMAL_BAUDRATE					(115200UL)//LCD����������������
#define GDDC_LCD_UPDATE_BAUDRATE					(115200UL)//LCD���������²�����
/*****************************************************************************/
#define GDDC_UART_HANDLE							huart4
#define GDDC_UART_IRQ								UART4_IRQn
#define GDDC_RX_BUF_SIZE							128
#define GDDC_TX_BUF_SIZE							(2048 + 4)
#define GDDC_HEX 									0
#define GDDC_DEC 									1 
#define GDDC_CHR 									2
#define GDDC_RX_TIMEOUT								0xFFFF
#define GDDC_TX_TIMEOUT								0xFFFF
#define GDDC_RETRY_TIMES							10//�������Դ���
#define GDDC_UPDATE_BAUDRATE						115200//���ı䲨����
/*****************************************************************************/
#define MORSECODE_SPACE_TIME						3000
#define MORSECODE_LONG_TIME							900
#define MORSECODE_SHORT_TIME						300
/*****************************************************************************/
uint32_t TmpReadSize = 0x00;
uint32_t RamAddress = 0x00;
static __IO uint32_t LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};//�ļ���д����
/*****************************************************************************/
uint8_t gddcRxBuf[GDDC_RX_BUF_SIZE];//��Ļ���ڽ��ջ�����
uint8_t gddcTxBuf[GDDC_TX_BUF_SIZE];//��Ļ���ڷ��ͻ�����
/*****************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
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
/*****************************************************************************/
static void bootLoadFailHandler(uint8_t ftype);//�������ϳ���
static uint32_t updateMcuApp(void);
static uint32_t updateLcdApp(void);
static void DBGU_Printk(uint8_t *buffer);
static void DBGU_Printk_num(uint8_t *buffer, uint16_t datanum);
static void dp_display_text(uint8_t *text);
static void dp_display_text_num(uint8_t *text,uint16_t datanum);	
static void dp_display_value(uint32_t value,int descriptive);
static void dp_display_array(uint8_t *value,int bytes, int descriptive);	
static void beepDiag(uint8_t diag);
static void clearFlash(void);
static uint32_t getOriginAppCrc(void);
static uint32_t getNewMcuAppCrc(void);
static uint32_t getNewLcdAppCrc(void);
static void updateEprom(void);
static void dumpEprom(void);
/******************************************************************************/
void bootLoadInit(void){//���������ʼ��
	SET_SPEAKER_OFF;//�رշ�����
	SET_AIM_OFF;//�ر�ָʾ����
	SET_FAN_ON;//�򿪼�������ȴ����
	SET_TEC_OFF;//�ر�����
	//�ر����м���
	SET_LASER_CH0_ON;
	SET_LASER_CH1_ON;
	SET_LASER_CH2_ON;
	SET_LASER_CH3_ON;
	//�ر�����LED
	SET_RLED_OFF;
	SET_GLED_OFF;
	SET_YLED_OFF;
	SET_TICK_LED_OFF;
	SET_ERR_LED_OFF;
	//R-G-Y��ˮ
	//R
	SET_RLED_ON;
	SET_GLED_OFF;
	SET_YLED_OFF;
	HAL_Delay(500);
	//G
	SET_RLED_OFF;
	SET_GLED_ON;
	SET_YLED_OFF;
	HAL_Delay(500);
	//Y
	SET_RLED_OFF;
	SET_GLED_OFF;
	SET_YLED_ON;
	HAL_Delay(500);
	//G
	SET_RLED_OFF;
	SET_GLED_ON;
	SET_YLED_OFF;
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
	printf("Bootloader:OUTPUT->AIM_PWM	     	= ON!\n");
	printf("Bootloader:OUTPUT->LAS_FAN  	   	= ON!\n");
	printf("Bootloader:OUTPUT->LAS_TEC     		= OFF!\n");
	HAL_Delay(10);
	
}
void bootLoadProcess(void){//bootload ִ�г���
	HAL_StatusTypeDef ret;
	uint8_t fileBuff[64];
	uint32_t crcFlash, crcUdisk;
	uint32_t crcEpromMcu, crcEpromLcd;//EPROM�д����CRC��¼ֵ
	uint32_t brByte;//ʵ�ʶ�ȡ���ֽ���
	//uint32_t bwByte;//ʵ��д����ֽ���
	//ע��һ��FATFS�ļ�ϵͳ
	switch(bootLoadState){
		case BT_STATE_IDLE:{//�����ȴ�U��ʶ��                             
			SET_RLED_ON;
			SET_GLED_OFF;
			SET_YLED_OFF;	
			printf("Bootloader:Start...............\n");
			listEpromTable();
			readStm32UniqueID();
			ret = epromReadDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);//��EPROM�����豸����
			if(ret == HAL_OK){
				printf("Bootloader:Read eprom MCU FW CRC32:0x%08X\n", crcEpromMcu);
			}
			else{
				printf("Bootloader:Read eprom MCU FW CRC32 fail!\n");
			}
			ret = epromReadDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);//��EPROM�����豸����
			if(ret == HAL_OK){
				printf("Bootloader:Read eprom LCD FW CRC32:0x%08X\n", crcEpromLcd);
			}
			else{
				printf("Bootloader:Read eprom LCD FW CRC32 fail!\n");
			}			
			printf("Bootloader:UniqueID->0x%08X%08X%08X\n", UniqueId[0], UniqueId[1], UniqueId[2]);
			printf("Bootloader:Mcu flash size->%d Kbytes\n", cpuGetFlashSize());
			printf("Bootloader:Ver->0x%08X Build->%s:%s\n", BOOTLOADER_VER, __DATE__, __TIME__);
			printf("Bootloader:Bootload Start:0x%08X,End:0x%08X,Size:0x%08X\n", BOOTLOADER_FLASH_START_ADDRESS, BOOTLOADER_FLASH_END_ADDRESS ,BOOTLOADER_FLASH_SIZE);
			printf("Bootloader:Applicent Start:0x%08X,End:0x%08X,Size:0x%08X\n", APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_END_ADDRESS, APPLICATION_FLASH_SIZE);
			if(	(GET_INTERLOCK_NC == GPIO_PIN_SET) &&//��ȫ����δ����
				(GET_FSWITCH_NC == GPIO_PIN_RESET) &&//��̤����
				(GET_FSWITCH_NO == GPIO_PIN_RESET)){//��̤����
				bootLoadState = BT_STATE_LOAD_FWINFO;//����USB����APP����
			}
			else{//��ȫ��������
				bootLoadState = BT_STATE_RUN_APP;//��������APP����
			}
			break;
		}
		case BT_STATE_LOAD_FWINFO:{
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
				epromWriteDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);
				clearEprom(CLEAR_EPROM_NVRAM);//���NVRAM���索����
				printf("Bootloader:Update new crc32 sucess.\n");
			}
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_LCD_APP:{//����LCDӦ�ó���			
			crcUdisk = getNewLcdAppCrc();//����U����MCU APP�̼�CRC32
			if(crcUdisk == crcEpromLcd){//У������ͬ��������
				printf("Bootloader:Check lcd app crc same,skip!\n");
				bootLoadState = BT_STATE_RUN_APP;
				break;
			}
			crcUdisk = updateLcdApp();
			crcEpromLcd = crcUdisk;//����EPROM��LCD APP CRCֵ
			epromWriteDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);
			printf("Bootloader:Update lcd app new crc32 sucess,wait 60s lcd upgrade done\n");
			//�ȴ�60�� LCD FLASHд����ɺ�����
			HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
			HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
			HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
			HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_BOTH_APP:{//����ȫ��Ӧ�ó���
			//MCU ����
			crcFlash = getOriginAppCrc();//����FLASH��APP�̼�CRC32
			crcUdisk = getNewMcuAppCrc();//����U����MCU APP�̼�CRC32
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
				epromWriteDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);
				clearEprom(CLEAR_EPROM_NVRAM);
				printf("Bootloader:Update mcu app new crc32 sucess.\n");
			}
			//LCD ����
			crcUdisk = getNewLcdAppCrc();//����U����MCU APP�̼�CRC32
			if(crcUdisk == crcEpromLcd){//У������ͬ��������
				printf("Bootloader:Check lcd app crc same,skip!\n");
			}
			else{
				updateLcdApp();
				crcEpromLcd = crcUdisk;
				epromWriteDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);
				printf("Bootloader:Update lcd app new crc32 sucess,wait 60s lcd upgrade done\n");
				//�ȴ�60�� LCD FLASHд����ɺ�����
				HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
				HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
				HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
				HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
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
				SET_RLED_OFF;
				SET_GLED_OFF;
				SET_YLED_OFF;
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
static void beepDiag(uint8_t diag){//������������� Ħ��˹����
	//�ر�USB VBUS
	switch(diag){
		case '0':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		case '1':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		};
		case '2':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		};
		case '3':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		case '4':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		case '5':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case '6':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case '7':{	
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case '8':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case '9':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'A':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		case 'B':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;			
		}
		case 'C':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'D':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'E':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'F':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'G':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'H':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;			
		}
		case 'I':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'J':{
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		case 'K':{
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		case 'L':{//���� ����
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_RLED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_RLED_OFF;
			break;
		}
		case 'M':{//�� ��
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_RLED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_RLED_OFF;
			break;
		}
		default:break;
	}
	HAL_Delay(3000);
}
static void bootLoadFailHandler(uint8_t ftype){//�����������
	MX_DriverVbusFS(FALSE);//�ر�USB VBUS
	printf("Bootloader:SYS_ERR_LED->On!\n");
	SET_RLED_OFF;
	SET_GLED_OFF;
	SET_YLED_OFF;
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
	f_lseek(&LcdFile, 0);//��ȡָ���ƶ�����ͷ
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
		SET_RLED_OFF;
		/* Program flash memory */
		for(programcounter = 0; programcounter < TmpReadSize; programcounter += 4){
			/* Write word into flash memory */
			if(FLASH_If_Write((LastPGAddress + programcounter), *(uint32_t *) (RamAddress + programcounter)) != 0x00){
				bootLoadFailHandler(BT_FAIL_WRITE_MCU_APP_FLASH);//д��FLASH����
			}
		}
		/* Update last programmed address value */
		LastPGAddress += TmpReadSize;
		SET_RLED_ON;
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
	f_lseek(&LcdFile, 0);//��ȡָ���ƶ�����ͷ
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


