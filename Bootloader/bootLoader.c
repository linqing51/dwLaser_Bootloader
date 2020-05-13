#include "bootLoader.h"
/*****************************************************************************/
//SECTOR0->16K:BOOTLOADER
//SECTOR1->16K:BOOTLOADER
//SECTOR2->16K:BOOTLOADER
//SECTOR3->16K:BOOTLOADER
//SECTOR4->64K+:APP
/*****************************************************************************/
#if DEBUG_NOBEEP == 1
#if BOARD_STM32F407_DEV == 1
#define SET_RED_LED()						
#define SET_GREEN_LED()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET))
#define SET_BLUE_LED()						
#define SET_BEEP()							(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET))

#define RESET_RED_LED()						
#define RESET_GREEN_LED()					(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET))
#define RESET_BLUE_LED()					
#define RESET_BEEP()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET))

#define FLIP_RED_LED()						
#define FLIP_GREEN_LED()					(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10))
#define FLIP_BLUE_LED()						
#define FLIP_BEEP()							(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9))
#endif

#if BOARD_STM32F413H_DISCO == 1
#define SET_RED_LED()						(HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET))
#define SET_GREEN_LED()						(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET))
#define SET_BLUE_LED()						
#define SET_BEEP()							(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET))

#define RESET_RED_LED()						(HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET))
#define RESET_GREEN_LED()					(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET))
#define RESET_BLUE_LED()					
#define RESET_BEEP()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET))

#define FLIP_RED_LED()						
#define FLIP_GREEN_LED()					(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10))
#define FLIP_BLUE_LED()						
#define FLIP_BEEP()							(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9))
#endif

#if BOARD_STM32F413RH_DWLASER == 1
#endif

#else
#define SET_RED_LED()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET))
#define SET_GREEN_LED()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET))
#define SET_BLUE_LED()						
#define SET_BEEP()							(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET))

#define RESET_RED_LED()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET))
#define RESET_GREEN_LED()					(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET))
#define RESET_BLUE_LED()					
#define RESET_BEEP()						(HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET))

#define FLIP_RED_LED()						(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9))
#define FLIP_GREEN_LED()					(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10))
#define FLIP_BLUE_LED()						
#define FLIP_BEEP()							(HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_8))
#endif
/*****************************************************************************/
#define BOOTLOADER_VER						0x00010001//�汾
#define DEVID_H								'0'//�豸ID
#define DEVID_L								'A'//�豸ID
#define BUFFER_SIZE        					((uint16_t)512*32)//512��������
#define CONFIG_JUMP_DELAY					3000//���U��ʱ��
#define FATFS_ROOT							"0:"
#define CFG_FIRMWARE_FILENAME				"/las.cfg"
#define LBOT_FIRMWARE_FILENAME				"/ld_bot.bin"//����BOT�̼���ַ
#define LMCU_FIRMWARE_FILENAME				"/ld_mcu.bin"//����MCU�̼���ַ
#define LLCD_FIRMWARE_FILENAME				"/ld_lcd.pkg"//����LCD�̼���ַ
#define SBOT_FIRMWARE_FILENAME				"/sv_bot.bin"//�ض�BOT�̼���ַ
#define SMCU_FIRMWARE_FILENAME				"/sv_mcu.bin"//�ض�MCU�̼���ַ
#define SLCD_FIRMWARE_FILENAME				"/sv_lcd.pkg"//�ض�LCD�̼���ַ
#define BOOTLOADER_ERASE_ENABLE				0//Bootloader����ʹ��
/*****************************************************************************/
#define BT_STATE_IDLE						0//����
#define BT_STATE_USBHOST_INIT				1//FATFS ��ʼ��
#define BT_STATE_WAIT_UDISK					2//�ȴ�USB DISK����
#define BT_STATE_READ_CFG					3//��ȡ�����ļ�
#define BT_STATE_UPDATE_MCU_BOT				4//����BOOTLOAD
#define BT_STATE_UPDATE_MCU_APP				5//���µ�Ƭ��Ӧ�ù̼�
#define BT_STATE_UPDATE_LCD_APP				6//������ĻӦ�ù̼�
#define BT_STATE_UPDATE_BOTH_APP			7//���µ�Ƭ�������̼�
#define BT_STATE_UPDATE_ALL					8//���µ�Ƭ������ Ӧ�� �������̼�
#define BT_STATE_CLEAR_FLASH				9//���FLASH�ռ�
#define BT_STATE_RESET						90//����
#define BT_STATE_RUN_APP					99//��ת��APPӦ�ó���
/*****************************************************************************/
#define BT_FAIL_READ_CFG					'0'//��U�̶�ȡCFGʧ��
#define BT_FAIL_READ_LMCU_APP				'1'//��U�̶�ȡMCU APPʧ��
#define BT_FAIL_READ_LMCU_BOT				'2'//��U�̶�ȡMCU BOOTLOADʧ��
#define BT_FAIL_READ_LLCD_APP				'3'//��U�̶�ȡLCD APPʧ��
#define BT_FAIL_WRITE_CFG					'4'//��U��д��CFGʧ��
#define BT_FAIL_WRITE_SMCU_APP				'5'//��U��д��sv_mcu.binʧ��
#define BT_FAIL_WRITE_SMCU_BOT				'6'//��U��д��sv_bot.binʧ��
#define BT_FAIL_WRITE_SLCD_APP				'7'//��U��д��sv_lcd.binʧ��
#define BT_FAIL_ERASE_MCU_APP				'8'//����MCU APP FLASH����ʧ��
#define BT_FAIL_ERASE_MCU_BOT				'9'//����MCU BOOTLOAD FLASHʧ��
#define BT_FAIL_LMCU_APP_CHECK				'A'//ld_mcu.bin CRC������
#define BT_FAIL_LMCU_BOT_CHECK				'B'//ld_bot.bin CRC������
#define BT_FAIL_LLCD_APP_CHECK				'C'//ld_lcd.bin CRC������
#define BT_FAIL_CHECKSUM_MCU_BOT_FLASH		'D'//У�� bootload ����
#define BT_FAIL_CHECKSUM_MCU_APP_FLASH		'E'//У�� mcu app ����
#define BT_FAIL_WRITE_MCU_APP_FLASH			'F'//дmcu app flash����
#define BT_FAIL_WRITE_MCU_BOT_FLASH			'G'//дmcu bot flash����
#define BT_FAIL_LCD_NOT_RESPOND				'H'//LCD����ͨ�ų�ʱ�����
#define BT_FAIL_LCD_DOWNLOAD				'I'//LCD����ʧ��
#define BT_FAIL_VECTOR_TABLE_INVALID		'J'//APP ���������
#define BT_FAIL_CHECK_BLANK					'K'//FLASH��մ���
#define BT_FAIL_CLEAR_DONE					'L'//FLASH��EPROM������
/*****************************************************************************/
#define GDDC_LCD_NORMAL_BAUDRATE			(115200UL)//LCD����������������
#define GDDC_LCD_UPDATE_BAUDRATE			(115200UL)//LCD���������²�����
/*****************************************************************************/
#if BOARD_STM32F413H_DISCO == 1
#define GDDC_UART_HANDLE					huart1
#define GDDC_UART_IRQ						USART1_IRQn
#endif
#if BOARD_STM32F407_DEV == 1
#define GDDC_UART_HANDLE					huart1
#define GDDC_UART_IRQ						USART1_IRQn
#endif

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
#define EPROM_ADR							0xA0
#define EPROM_SIZE							256
/*****************************************************************************/
static uint32_t TmpReadSize = 0x00;
static uint32_t RamAddress = 0x00;
static __IO uint32_t LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
static uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};
/*****************************************************************************/
static uint8_t gddcRxBuf[GDDC_RX_BUF_SIZE];//��Ļ���ڽ��ջ�����
static uint8_t gddcTxBuf[GDDC_TX_BUF_SIZE];//��Ļ���ڷ��ͻ�����
/*****************************************************************************/
extern I2C_HandleTypeDef hi2c1;
/*****************************************************************************/
FRESULT retUsbH;
FATFS	USBH_fatfs;
FIL CfgFile;//FATFS File Object ���������Ϣ
FIL McuFile;//FATFS File Object ��Ƭ���̼�
FIL LcdFile;//FATFS File Object ��Ļ�̼�
FIL BotFile;//FATFS File Object 
DIR	FileDir;//FATFS �ļ�Ŀ¼
FILINFO FileInfo;//FATFS �ļ���Ϣ
static uint8_t bootLoadState;
uint8_t usbReady;//USB DISK����
int32_t releaseTime0, releaseTime1, overTime, releaseCounter;
uint32_t JumpAddress;
pFunction Jump_To_Application;
/*****************************************************************************/
static void bootLoadFailHandler(uint8_t ftype);//�������ϳ���
static uint16_t updateMcuBot(void);
static uint16_t updateMcuApp(void);
static uint16_t updateLcdApp(void);
static uint16_t checksumMcuBot(void);
static uint16_t checksumMcuApp(void);
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
static void clearFlashAndEprom(void);
void clearEprom(void);
/*****************************************************************************/
void epromTest(void);

void resetInit(void){//��λ���ʼ��
	HAL_DeInit();
	//��λRCCʱ��
	SystemClock_Reset();
	UsbGpioReset();
	__enable_irq();
}
void bootLoadInit(void){//���������ʼ��
	overTime = HAL_GetTick() + CONFIG_JUMP_DELAY;
	releaseTime0 = 0;
	releaseTime1 = 0;
	usbReady = FALSE;
	bootLoadState = BT_STATE_IDLE; 
	RESET_RED_LED();
	RESET_GREEN_LED();
	RESET_BLUE_LED();
	RESET_BEEP();
	SET_BEEP();
	HAL_Delay(200);
	RESET_BEEP();
}
void bootLoadProcess(void){//bootload ִ�г���
	uint8_t fileBuff[64];
	uint16_t crcFlash, crcUdisk;
	uint32_t brByte;//ʵ�ʶ�ȡ���ֽ���
	uint32_t bwByte;//ʵ��д����ֽ���
	//ע��һ��FATFS�ļ�ϵͳ
	switch(bootLoadState){
		case BT_STATE_IDLE:{//�����ȴ�U��ʶ��
			printf("\r\n");
			printf("\r\n");
			printf("\r\n");                                
			printf("Bootloader:Start...............\n");
			printf("Bootloader:Ver:0x%08X Build:%s:%s\r\n", BOOTLOADER_VER, __DATE__, __TIME__);
			bootLoadState = BT_STATE_USBHOST_INIT;
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
			retUsbH = f_open(&CfgFile, CFG_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ | FA_WRITE);//��ȡ�����Ϣ�ļ�
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
				if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '1'){//U01 ��������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Upgrade mcu bootloader!\n");
						bootLoadState = BT_STATE_UPDATE_MCU_BOT;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}		
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '2'){//U02 ���� Ӧ��
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Upgrade mcu application!\n");
						bootLoadState = BT_STATE_UPDATE_MCU_APP;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '3'){//U03 ���� ������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Upgrade lcd application!\n");
						bootLoadState = BT_STATE_UPDATE_LCD_APP;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '4'){//U04 ���� Ӧ�� ������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Upgrade mcu application & lcd application!\n");
						bootLoadState = BT_STATE_UPDATE_BOTH_APP;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '5'){//U05 �������� Ӧ�� ������
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Upgrade bootloader,mcu application,lcd application!\n");
						bootLoadState = BT_STATE_UPDATE_ALL;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == 'F'){//U05 Clear ʣ��FLASH�����EEPROM
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//�豸ƥ��
						printf("Bootloader:Clear mcu app flash & eprom!\n");
						bootLoadState = BT_STATE_CLEAR_FLASH;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else{//�������������ת��ִ��APP
					bootLoadState = BT_STATE_RUN_APP;
				}
			}
			break;
		}
		case BT_STATE_UPDATE_MCU_BOT:{
#if BOOTLOADER_ERASE_ENABLE == 1
			crcUdisk = updateMcuBot();
			crcFlash = checksumMcuBot();
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_BOT_FLASH);
			}
			else{
				printf("Bootloader:Checksum bootloader sucess.\n");
			}
#endif
#if DEBUG_BOOTLOADER != 1
			fileBuff[0] = 'U';
			fileBuff[1] = '1';
			fileBuff[2] = '1';
#else
			fileBuff[0] = 'U';
			fileBuff[1] = '0';
			fileBuff[2] = '1';		
#endif
			bwByte = 0;
			f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
			if(f_write(&CfgFile, fileBuff, 3, (void *)&bwByte) != FR_OK){
				bootLoadFailHandler(BT_FAIL_WRITE_CFG);
			}
			f_close(&CfgFile);
			bootLoadState = BT_STATE_RESET;
			break;
		}
		case BT_STATE_UPDATE_MCU_APP:{//����ld_mcu.bin
			crcUdisk = updateMcuApp();//д��MCU FLASH
			crcFlash = checksumMcuApp();//У��MCU FLASH
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
			}
			else{
				printf("Bootloader:Checksum mcu app sucess.\n");
			}
			clearEprom();
#if DEBUG_BOOTLOADER != 1
			fileBuff[0] = 'U';
			fileBuff[1] = '1';
			fileBuff[2] = '2';
#else
			fileBuff[0] = 'U';
			fileBuff[1] = '0';
			fileBuff[2] = '2';
#endif
			bwByte = 0;
			f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
			if(f_write(&CfgFile, fileBuff, 3, (void *)&bwByte) != FR_OK){
				bootLoadFailHandler(BT_FAIL_WRITE_CFG);
			}
			f_close(&CfgFile);
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		}
		case BT_STATE_UPDATE_LCD_APP:{			
			crcUdisk = updateLcdApp();
#if DEBUG_BOOTLOADER != 1
			fileBuff[0] = 'U';
			fileBuff[1] = '1';
			fileBuff[2] = '3';
#else
			fileBuff[0] = 'U';
			fileBuff[1] = '0';
			fileBuff[2] = '3';
#endif
			bwByte = 0;
			f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
			if(f_write(&CfgFile, fileBuff, 3, (void *)&bwByte) != FR_OK){
				bootLoadFailHandler(BT_FAIL_WRITE_CFG);
			}
			f_close(&CfgFile);
			bootLoadState = BT_STATE_RESET;//����APP
			break;
		
		}
		case BT_STATE_UPDATE_BOTH_APP:{ 	
			crcUdisk = updateMcuApp();
			crcFlash = checksumMcuApp();
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_LMCU_APP_CHECK);
			}
			clearEprom();
			updateLcdApp();
#if DEBUG_BOOTLOADER != 1
			fileBuff[0] = 'U';
			fileBuff[1] = '1';
			fileBuff[2] = '4';
#else
			fileBuff[0] = 'U';
			fileBuff[1] = '0';
			fileBuff[2] = '4';
#endif
			bwByte = 0;
			f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
			if(f_write(&CfgFile, fileBuff, 3, (void *)&bwByte) != FR_OK){
				bootLoadFailHandler(BT_FAIL_WRITE_CFG);
			}
			f_close(&CfgFile);
			bootLoadState = BT_STATE_RESET;
			break;
		}
		case BT_STATE_UPDATE_ALL:{
			crcUdisk = updateMcuBot();
			crcFlash = checksumMcuBot();
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_BOT_FLASH);
			}
			crcUdisk = updateMcuApp();
			crcFlash = checksumMcuApp();
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_LMCU_APP_CHECK);
			}
			clearEprom();
			updateLcdApp();
#if DEBUG_BOOTLOADER != 1
			fileBuff[0] = 'U';
			fileBuff[1] = '1';
			fileBuff[2] = '5';
#else
			fileBuff[0] = 'U';
			fileBuff[1] = '0';
			fileBuff[2] = '5';
			
#endif
			bwByte = 0;
			f_lseek(&CfgFile, 0);//��ȡָ���ƶ�����ͷ
			if(f_write(&CfgFile, fileBuff, 3, (void *)&bwByte) != FR_OK){
				bootLoadFailHandler(BT_FAIL_WRITE_CFG);
			}
			f_close(&CfgFile);
			bootLoadState = BT_STATE_RESET;
			break;
		}
		case BT_STATE_CLEAR_FLASH:{
			clearFlashAndEprom();
			break;
		}
		case BT_STATE_RESET:{
			printf("Bootloader:System Reset\n");
			__set_FAULTMASK(1);
			NVIC_SystemReset();/* Software reset */
			break;
		}
		case BT_STATE_RUN_APP:{
			/* Check Vector Table: Test if user code is programmed starting from* address "APPLICATION_ADDRESS" */
			if((((*(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS) & 0xFF000000) == 0x20000000) || (((*(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS) & 0xFF000000) == 0x10000000)){
				/* Jump to user application */
				printf("Bootloader:Jump application address.\n");
				printf("Bootloader:First word:0x%XH\n",(*(uint32_t*)APPLICATION_FLASH_START_ADDRESS));
				JumpAddress = *(__IO uint32_t *) (APPLICATION_FLASH_START_ADDRESS + 4);
				Jump_To_Application = (pFunction) JumpAddress;
				/* Initialize user application's Stack Pointer */
				__set_MSP(*(__IO uint32_t *) APPLICATION_FLASH_START_ADDRESS);
				SysTick->CTRL = 0;//�ؼ�����
				HAL_DeInit();                                    
				HAL_NVIC_DisableIRQ(SysTick_IRQn); 
				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);				
				HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);           //��ѡ
				HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
				__disable_irq();
				HAL_FLASH_Lock();//����FLASH
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
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		case '1':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		};
		case '2':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		};
		case '3':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP(); HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		case '4':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		case '5':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case '6':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case '7':{	
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case '8':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case '9':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'A':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		case 'B':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;			
		}
		case 'C':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'D':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'E':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'F':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'G':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'H':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;			
		}
		case 'I':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'J':{
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		case 'K':{
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		case 'L':{//���� ����
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_BEEP();HAL_Delay(MORSECODE_SHORT_TIME);RESET_BEEP();
			break;
		}
		case 'M':{//�� ��
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_BEEP();HAL_Delay(MORSECODE_LONG_TIME);RESET_BEEP();
			break;
		}
		default:break;
	}
	HAL_Delay(3000);
}
static void bootLoadFailHandler(uint8_t ftype){//�����������
	MX_DriverVbusFS(FALSE);//�ر�USB VBUS
	//�������
	SET_RED_LED();
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
		case BT_FAIL_READ_LMCU_BOT:{//��U�̶�ȡMCU BOOTLOADʧ��
			printf("Bootloader:FailHandler,Read %s fail!.\n", LBOT_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_LMCU_BOT);
			};
		}
		case BT_FAIL_READ_LLCD_APP:{//��U�̶�ȡLCD APPʧ��
			printf("Bootloader:FailHandler,Read %s fail!.\n", LLCD_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_LLCD_APP);
			};
		}
		case BT_FAIL_WRITE_CFG:{//��U��д��CFGʧ��
			printf("Bootloader:FailHandler,Write %s fail!.\n", CFG_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_WRITE_CFG);
			};
		}
		case BT_FAIL_WRITE_SMCU_APP:{//��U��д��MCU APPʧ��
			printf("Bootloader:FailHandler,Write %s fail\n", SMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_WRITE_SMCU_APP);
			};
		}
		case BT_FAIL_WRITE_SMCU_BOT:{//��U��д��MCU BOOTLOADʧ��
			printf("Bootloader:FailHandler,Write %s fail\n", SBOT_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_WRITE_SMCU_BOT);
			}
		}
		case BT_FAIL_WRITE_SLCD_APP:{//��U��д��LCD APPʧ��
			printf("Bootloader:FailHandler,Write %s fail\n", SLCD_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_WRITE_SLCD_APP);
			};
		}
		case BT_FAIL_ERASE_MCU_APP:{//����MCU APP FLASH����ʧ��
			printf("Bootloader:FailHandler,Erase mcu application fail\n");
			while(1){
				beepDiag(BT_FAIL_ERASE_MCU_APP);
			};
		}
		case BT_FAIL_ERASE_MCU_BOT:{//����MCU BOOTLOAD FLASHʧ��
			printf("Bootloader:FailHandler,Erase mcu bootloader fail!\n");
			while(1){
				beepDiag(BT_FAIL_ERASE_MCU_BOT);
			};
		}
		case BT_FAIL_LMCU_APP_CHECK:{//lmcu.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_LMCU_APP_CHECK);
			};
		}
		case BT_FAIL_LMCU_BOT_CHECK:{//lbot.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LBOT_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_LMCU_BOT_CHECK);
			};
		}
		case BT_FAIL_LLCD_APP_CHECK:{//llcd.bin ������
			printf("Bootloader:FailHandler,%s size is invalid!\n", LLCD_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_LLCD_APP_CHECK);
			};
		}
		case BT_FAIL_CHECKSUM_MCU_BOT_FLASH:{//У�� lbot.bin ����
			printf("Bootloader:FailHandler,Verify %s fail!.\n", LBOT_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_CHECKSUM_MCU_BOT_FLASH);
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
		case BT_FAIL_CLEAR_DONE:{//
			printf("Bootloader:FailHandler,Flash and Eprom easer done!.\n");
			while(1){
				beepDiag(BT_FAIL_CLEAR_DONE);
			}
		}
		default:{
			break;
		}
	}
}
static void clearFlashAndEprom(void){
	RESET_GREEN_LED();
	RESET_RED_LED();
	SET_BLUE_LED();
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
	if (FLASH_If_EraseApplication() != 0x00){//����APP FLASH����ʧ��
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_SIZE);//FLASH ���
	SET_GREEN_LED();
	RESET_RED_LED();
	RESET_BLUE_LED();
	clearEprom();
	bootLoadFailHandler(BT_FAIL_CLEAR_DONE);
}
static uint16_t updateMcuBot(void){//����MCU BOOTLOADER
	retUsbH = f_open(&BotFile, LBOT_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LMCU_BOT);			
	}
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", LMCU_FIRMWARE_FILENAME, retUsbH);
	if(f_size(&McuFile) > BOOTLOADER_FLASH_SIZE){//MCU�̼�����FLSAH����
		bootLoadFailHandler(BT_FAIL_LMCU_BOT_CHECK);
	}
	RESET_GREEN_LED();
	RESET_RED_LED();
	SET_BLUE_LED();
	if (FLASH_If_EraseBootloader() != 0x00){//����APP FLASH����ʧ��
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(BOOTLOADER_FLASH_START_ADDRESS, BOOTLOADER_FLASH_SIZE);//FLASH ���
	printf("Bootloader:Erase mcu bootload sucess.\n");
	SET_GREEN_LED();
	RESET_RED_LED();
	RESET_BLUE_LED();
	printf("Bootloader:Write bootload finish.\n");
	f_close(&BotFile);
	return 0;
}
static uint16_t updateMcuApp(void){//����MCU APP
	uint16_t crc16;
	uint32_t i;
	uint32_t programcounter = 0x00;
	uint8_t readflag = TRUE;
	uint16_t bytesread;//ʵ���ļ���ȡ�ֽ���
	retUsbH = f_open(&McuFile, LMCU_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//��ȡʧ��
		bootLoadFailHandler(BT_FAIL_READ_LMCU_APP);			
	}
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", LMCU_FIRMWARE_FILENAME, retUsbH);
	if(f_size(&McuFile) > APPLICATION_FLASH_SIZE){//MCU�̼�����FLSAH����
		bootLoadFailHandler(BT_FAIL_LMCU_APP_CHECK);
	}
	RESET_GREEN_LED();
	RESET_RED_LED();
	SET_BLUE_LED();
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
	if (FLASH_If_EraseApplication() != 0x00){//����APP FLASH����ʧ��
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_SIZE);//FLASH ���
	printf("Bootloader:Erase mcu application sucess.\n");
	SET_GREEN_LED();
	RESET_RED_LED();
	RESET_BLUE_LED();
	printf("Bootloader:Erase mcu app sucess.\n");
	RamAddress = (uint32_t)&RAM_Buf;//��ȡRAM��������ַ
	/* Erase address init */
	LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
	/* While file still contain data */
	crc16 = 0;
	crc16Clear();
	while((readflag == TRUE)){
		/* Read maximum 512 Kbyte from the selected file */
		f_read(&McuFile, RAM_Buf, BUFFER_SIZE, (void*)&bytesread);
		crc16 = crc16Calculate(RAM_Buf, bytesread);
		/* Temp variable */
		TmpReadSize = bytesread;
		/* The read data < "BUFFER_SIZE" Kbyte */
		if(TmpReadSize < BUFFER_SIZE){
			readflag = FALSE;
		}
		/* Program flash memory */
		FLIP_GREEN_LED();//�̵�
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
		crc16 = crc16CalculateAdd(0xFF);
	}
	HAL_FLASH_Lock();
	printf("Bootloader:Write mcu app finish.\n");
	f_close(&McuFile);
	SET_GREEN_LED();
	return crc16;
}

static uint16_t updateLcdApp(void){//����LCD APP
	UART_HandleTypeDef *puart;
	HAL_StatusTypeDef uRet;
	uint16_t crc16;
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
	crc16 = 0;
	crc16Clear();
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
		crc16 = crc16Calculate(&gddcTxBuf[2], actualByte);
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
	return crc16;
}

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
static void checkBlank(uint32_t adr, uint32_t size){//Flash ���
	uint8_t val;
	uint32_t i;
	for(i = 0;i < size;i ++){
		val = *(__IO uint8_t*)(adr + i);
		if(val != 0xFF){
			bootLoadFailHandler(BT_FAIL_CHECK_BLANK);
		}
	}
}
static uint16_t checksumMcuBot(void){//����MCU BOT CRC16
	uint8_t val;
	uint32_t i;
	uint16_t crc16;
	crc16 = 0;
	crc16Clear();
	for(i = BOOTLOADER_FLASH_START_ADDRESS;i < BOOTLOADER_FLASH_SIZE;i ++){
		val = *(__IO uint8_t*)(i);
		crc16 = crc16CalculateAdd(val);//CRC16���������ֽ�
	}
	return crc16;	
}
static uint16_t checksumMcuApp(void){//����MCU APP CRC32
	uint8_t val;
	uint32_t i;
	uint16_t crc16;
	crc16Clear();
	for(i = APPLICATION_FLASH_START_ADDRESS;i < APPLICATION_FLASH_END_ADDRESS;i ++){
		val = *(__IO uint8_t*)(i);
		crc16 = crc16CalculateAdd(val);//CRC16���������ֽ�
	}
	return crc16;	
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
	HAL_Delay(65);
	//�Ȱ�PA12���������ߣ�����D+ģ��USB�İβ嶯��   
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(65);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
	__HAL_RCC_GPIOA_CLK_DISABLE();
	
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	HAL_Delay(1000);//�ȴ�100mS
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
	/** Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_DISABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK){
		Error_Handler();
	}
}


HAL_StatusTypeDef EEPROM_WriteData(uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	uint16_t i;
	HAL_StatusTypeDef sta;
	for(i = 0;i < Size;i ++){
		MemAddress += i;
		sta = HAL_I2C_Mem_Write(&hi2c1, EPROM_ADR, MemAddress, I2C_MEMADD_SIZE_8BIT, &pData[i], 1, 200);
		if(sta != HAL_OK){
			return sta;
		}
		HAL_Delay(1);
	}
	return sta;
}

HAL_StatusTypeDef EEPROM_ReadData(uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	uint16_t i;
	HAL_StatusTypeDef sta;
	for(i = 0;i < Size;i ++){
		MemAddress += i;
		sta = HAL_I2C_Mem_Read(&hi2c1, EPROM_ADR, MemAddress,I2C_MEMADD_SIZE_8BIT, &pData[i], 1, 200);
		if(sta != HAL_OK){
			return sta;
		}
		HAL_Delay(10);
	}
	return sta;
}
void clearEprom(void){//EPROM���
	uint8_t var = 0;
	uint32_t i;
	for(i = 0;i < EPROM_SIZE;i ++){
		EEPROM_WriteData(0, &var, 1);
	}
	printf("Bootloader->:Erase eprom sucess!\n");
}
void epromTest(void){
	uint8_t wbuf[16] = {'0', '1', '2', '3', '4', '5', '6', '7', 
	                        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	uint8_t rbuf[16];
	EEPROM_WriteData(0x10, wbuf, 16);
	EEPROM_ReadData(0x10, rbuf, 16);
	while(1);
}


