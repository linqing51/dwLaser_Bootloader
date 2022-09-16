#include "bootLoader.h"
/*****************************************************************************/
//SECTOR0->16K:BOOTLOADER
//SECTOR1->16K:BOOTLOADER
//SECTOR2->16K:BOOTLOADER
//SECTOR3->16K:BOOTLOADER
/*****************************************************************************/
#define BT_STATE_IDLE															0//����
#define BT_STATE_USBHOST_INIT											1//FATFS ��ʼ��
#define BT_STATE_WAIT_UDISK												2//�ȴ�USB DISK����
#define BT_STATE_READ_CFG													3//��ȡ�����ļ�
#define BT_STATE_UPDATE_MCU_BOT										4//����BOOTLOAD
#define BT_STATE_UPDATE_MCU_APP										5//���µ�Ƭ��Ӧ�ù̼�
#define BT_STATE_UPDATE_EPROM											6//����UDISK->EPROM
#define BT_STATE_DUMP_EPROM												7//����ȫ��EPROM��UDISK
#define BT_STATE_CLEAT_ALL												8//���FLASH��EPROMȫ��
#define BT_STATE_RESET														90//����
#define BT_STATE_RUN_APP													99//��ת��APPӦ�ó���
/*****************************************************************************/
#define BT_FAIL_READ_CFG													'0'//��U�̶�ȡCFGʧ��
#define BT_FAIL_READ_LMCU_APP											'1'//��U�̶�ȡMCU APPʧ��
#define BT_FAIL_READ_LEROM_BIN										'2'//��U�̶�ȡEPROM BINʧ��
#define BT_FAIL_WRITE_SEROM_BIN										'3'//��U��д��EPROM BINʧ��
#define BT_FAIL_ERASE_MCU_APP											'4'//����FLASHʧ��
#define BT_FAIL_READ_EPROM												'5'//��ȡEPROMʧ��
#define BT_FAIL_WRITE_EPROM												'6'//д��EPROMʧ��
#define BT_FAIL_LMCU_APP_CHECK										'7'//ld_mcu.bin CRC������
#define BT_FAIL_LMCU_BOT_CHECK										'8'//ld_bot.bin CRC������
#define BT_FAIL_CHECKSUM_MCU_APP_FLASH						'9'//У�� mcu app ����
#define BT_FAIL_WRITE_MCU_APP_FLASH								'A'//дmcu app flash����
#define BT_FAIL_VECTOR_TABLE_INVALID							'B'//APP ���������
#define BT_FAIL_CHECK_BLANK												'C'//FLASH��մ���
#define BT_DONE_CLEAR_ALL													'D'//FLASH��EPROM������
#define BT_DONE_UPDATE_EPROM											'E'//����EPROM���
#define BT_DONE_DUMP_EPROM												'F'//����EPROM���
/*****************************************************************************/
#define MORSECODE_SPACE_TIME											1000
#define MORSECODE_LONG_TIME												750
#define MORSECODE_SHORT_TIME											150
/*****************************************************************************/
const char BootloadVer[8] __attribute__((at(0X800F000)))= {0x00, 0x00, 0x00, 0x00, 0x80, 0x5A, 0x00, 0x01};
/*****************************************************************************/
uint32_t TmpReadSize = 0x00;
uint32_t RamAddress = 0x00;
static __IO uint32_t LastPGAddress = APPLICATION_FLASH_START_ADDRESS;
uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};//�ļ���д����
/*****************************************************************************/
extern I2C_HandleTypeDef hi2c1;
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
uint32_t UniqueId[3];
/*****************************************************************************/
static void bootLoadFailHandler(uint8_t ftype);//�������ϳ���
static uint32_t updateMcuApp(void);
static void clearFlash(void);
static uint32_t getOriginAppCrc(void);
static uint32_t getNewMcuAppCrc(void);
static void beepDiag(uint8_t diag);
static void readStm32ChipID(void);
static uint16_t cpuGetFlashSize(void);
void resetInit(void);
static void UsbGpioReset(void);
static void SystemClock_Reset(void);
static void softDelayMs(uint16_t ms);
static uint8_t checkBlank(uint32_t adr, uint32_t size);
/******************************************************************************/
void bootLoadInit(void){//���������ʼ��
	SET_SPEAK_ENA(GPIO_PIN_RESET);//�رշ�����
	SET_LASER_PWM(GPIO_PIN_RESET);//�ر�����
	SET_LASER1_AIM(GPIO_PIN_RESET);//�ر�ָʾ����
	SET_LASER2_AIM(GPIO_PIN_RESET);
	SET_TEC_PWM(GPIO_PIN_RESET);//�رռ���
	SET_LINK_LED(GPIO_PIN_RESET);
	SET_ALARM_LED(GPIO_PIN_RESET);
	SET_LASER1_LED(GPIO_PIN_RESET);
	SET_LASER2_LED(GPIO_PIN_RESET);
	//ָʾ����ˮ
	//R
	SET_ALARM_LED(GPIO_PIN_SET);
	SET_LINK_LED(GPIO_PIN_SET);
	SET_LASER1_LED(GPIO_PIN_SET);
	SET_LASER2_LED(GPIO_PIN_SET);
	HAL_Delay(500);
	SET_ALARM_LED(GPIO_PIN_RESET);
	SET_LINK_LED(GPIO_PIN_RESET);
	SET_LASER1_LED(GPIO_PIN_RESET);
	SET_LASER2_LED(GPIO_PIN_RESET);
	overTime = HAL_GetTick() + CONFIG_JUMP_DELAY;
	releaseTime0 = 0;
	releaseTime1 = 0;
	usbReady = FALSE;
	bootLoadState = BT_STATE_IDLE; 
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");   
	//��ʾ����IO״̬
	if(GET_ESTOP() == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->ESTOP_NC      		= Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->ESTOP_NC      		= Close!\n");
	}
	//
	if(GET_EXT_ENABLE() == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->LASER0_START_NC   = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->LASER0_START_NC   = Close!\n");
	}
	//
	if(GET_LASER_TRG() == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->LASER1_START_NC   = Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->LASER1_START_NC   = Close!\n");
	}
	//
	if(GET_INTERLOCK() == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->INTERLOCK_NO    	= Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->INTERLOCK_NO    	= Close!\n");
	}
	//��ʾ���IO״̬
	if(GET_LASER_PWM() == GPIO_PIN_SET){//LASER_PWM
		printf("Bootloader:OUTPUT->LASER_PWM     		= High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LASER_PWM     		= Low!\n");
	}
	//
	if(GET_LASER1_AIM() == GPIO_PIN_SET){//
		printf("Bootloader:OUTPUT->LASER0_AIM       = ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LASER0_AIM       = OFF!\n");
	
	}
	//
	if(GET_LASER2_AIM() == GPIO_PIN_SET){//
		printf("Bootloader:OUTPUT->LASER1_AIM       = ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LASER1_AIM       = OFF!\n");
	}
	//
	if(GET_TEC_PWM() == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->TEC_PWM       		= ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->TEC_PWM       		= OFF!\n");
	}
	HAL_Delay(10);
}
void bootLoadProcess(void){//bootload ִ�г���
	uint8_t fileBuff[64];
	uint32_t crcFlash, crcUdisk;
	uint32_t brByte;//ʵ�ʶ�ȡ���ֽ���
	//uint32_t bwByte;//ʵ��д����ֽ���
	//ע��һ��FATFS�ļ�ϵͳ
	switch(bootLoadState){
		case BT_STATE_IDLE:{//�����ȴ�U��ʶ��                             
			SET_ALARM_LED(GPIO_PIN_RESET);
			SET_LASER1_LED(GPIO_PIN_RESET);
			SET_LASER2_LED(GPIO_PIN_RESET);
			printf("Bootloader:Start...............\n");
			readStm32ChipID();		
			printf("Bootloader:UniqueID->0x%08X%08X%08X\n", UniqueId[0], UniqueId[1], UniqueId[2]);
			printf("Bootloader:Mcu flash size->%d Kbytes\n", cpuGetFlashSize());
			printf("Bootloader:Build->%s:%s\n", __DATE__, __TIME__);
			printf("Bootloader:Bootload Start:0x%08X,End:0x%08X,Size:0x%08X\n", BOOTLOADER_FLASH_START_ADDRESS, BOOTLOADER_FLASH_END_ADDRESS ,BOOTLOADER_FLASH_SIZE);
			printf("Bootloader:Applicent Start:0x%08X,End:0x%08X,Size:0x%08X\n", APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_END_ADDRESS, APPLICATION_FLASH_SIZE);
			//��ȫ�����Ͽ�������������
			if(GET_INTERLOCK() == GPIO_PIN_SET){//���°�ȫ���������¼�ͣ ����APP��������
				bootLoadState = BT_STATE_USBHOST_INIT;//����USB����APP����
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
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '2'){//U04 ���� EPROM
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
			if(crcUdisk == crcFlash){//У������ͬ��������
				printf("Bootloader:Check mcu app crc same,skip!\n");
				bootLoadState = BT_STATE_RUN_APP;
				break;
			}
			crcUdisk = updateMcuApp();//д��MCU FLASH
			crcFlash = getOriginAppCrc();//У��MCU FLASH
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
			}
			else{
				printf("Bootloader:Checksum mcu app sucess.\n");
				bootLoadState = BT_STATE_RESET;//����APP
			}
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
				SET_ALARM_LED(GPIO_PIN_RESET);
				SET_LASER1_LED(GPIO_PIN_RESET);
				SET_LASER2_LED(GPIO_PIN_RESET);
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
	SET_ALARM_LED(GPIO_PIN_RESET);
	SET_LASER1_LED(GPIO_PIN_RESET);
	SET_LASER2_LED(GPIO_PIN_RESET);
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
		case BT_FAIL_CHECKSUM_MCU_APP_FLASH:{//У�� lmcu.bin ����
			printf("Bootloader:FailHandler,Verify %s fail!.\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
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
		FLIP_LINK_LED();
		/* Program flash memory */
		for(programcounter = 0; programcounter < TmpReadSize; programcounter += 4){
			/* Write word into flash memory */
			if(FLASH_If_Write((LastPGAddress + programcounter), *(uint32_t *) (RamAddress + programcounter)) != 0x00){
				bootLoadFailHandler(BT_FAIL_WRITE_MCU_APP_FLASH);//д��FLASH����
			}
		}
		/* Update last programmed address value */
		LastPGAddress += TmpReadSize;
		FLIP_LINK_LED();
	}
	for(i = LastPGAddress;i < APPLICATION_FLASH_END_ADDRESS;i ++){//����ʣ��CRC
		crc32 = crc32CalculateAdd(0xFF);
	}
	HAL_FLASH_Lock();
	printf("Bootloader:Write mcu app finish.\n");
	f_close(&McuFile);
	return crc32;
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

static void beepDiag(uint8_t diag){//������������� Ħ��˹����
	//�ر�USB VBUS
	switch(diag){
		case '0':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '1':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		};
		case '2':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		};
		case '3':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '4':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '5':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '6':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '7':{	
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '8':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case '9':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'A':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'B':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;			
		}
		case 'C':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'D':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'E':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'F':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'G':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'H':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;			
		}
		case 'I':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'J':{
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'K':{
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'L':{//���� ����
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		case 'M':{//�� ��
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED(GPIO_PIN_SET);;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED(GPIO_PIN_RESET);;
			break;
		}
		default:break;
	}
	HAL_Delay(3000);
}

static void readStm32ChipID(void){//��ȡ������Ψһ���к�        
	UniqueId[0] = *(volatile uint32_t*)(0x1FFF7A10);
	UniqueId[1] = *(volatile uint32_t*)(0x1FFF7A14);
	UniqueId[2] = *(volatile uint32_t*)(0x1FFF7A18);
}
static uint16_t cpuGetFlashSize(void){//��ȡ��������������
   return *(volatile uint16_t*)(0x1FFF7A22);
}
void resetInit(void){//��λ���ʼ��
	HAL_DeInit();
	//��λRCCʱ��
	SystemClock_Reset();
	UsbGpioReset();
	__enable_irq();
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

static void softDelayMs(uint16_t ms){//�����ʱ
	uint32_t i;
	for(i = 0;i < 1000;i ++){
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
}

static uint8_t checkBlank(uint32_t adr, uint32_t size){//MCU Flash ���
	uint8_t val;
	uint32_t i;
	for(i = 0;i < size;i ++){
		val = *(__IO uint8_t*)(adr + i);
		if(val != 0xFF){
			return 0;
		}
	}
	return 1;
}



