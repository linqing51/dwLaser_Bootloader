#include "bootLoader.h"
/*****************************************************************************/
//SECTOR0->16K:BOOTLOADER
//SECTOR1->16K:BOOTLOADER
//SECTOR2->16K:BOOTLOADER
//SECTOR3->16K:BOOTLOADER
/*****************************************************************************/
#define BT_STATE_IDLE															0//空闲
#define BT_STATE_LOAD_FWINFO											1//EPROM载入固件信息
#define BT_STATE_USBHOST_INIT											2//FATFS 初始化
#define BT_STATE_WAIT_UDISK												3//等待USB DISK就绪
#define BT_STATE_READ_CFG													4//读取配置文件
#define BT_STATE_UPDATE_MCU_BOT										5//更新BOOTLOAD
#define BT_STATE_UPDATE_MCU_APP										6//更新单片机应用固件
#define BT_STATE_UPDATE_EPROM											7//更新UDISK->EPROM
#define BT_STATE_DUMP_EPROM												8//储存全部EPROM到UDISK
#define BT_STATE_CLEAT_ALL												9//清除FLASH和EPROM全部
#define BT_STATE_RESET														90//重启
#define BT_STATE_RUN_APP													99//跳转到APP应用程序
/*****************************************************************************/
#define BT_FAIL_READ_CFG													'0'//从U盘读取CFG失败
#define BT_FAIL_READ_LMCU_APP											'1'//从U盘读取MCU APP失败
#define BT_FAIL_READ_LEROM_BIN										'2'//从U盘读取EPROM BIN失败
#define BT_FAIL_WRITE_SEROM_BIN										'3'//向U盘写入EPROM BIN失败
#define BT_FAIL_ERASE_MCU_APP											'4'//擦除FLASH失败
#define BT_FAIL_READ_EPROM												'5'//读取EPROM失败
#define BT_FAIL_WRITE_EPROM												'6'//写入EPROM失败
#define BT_FAIL_LMCU_APP_CHECK										'7'//ld_mcu.bin CRC检查错误
#define BT_FAIL_LMCU_BOT_CHECK										'8'//ld_bot.bin CRC检查错误
#define BT_FAIL_CHECKSUM_MCU_APP_FLASH						'9'//校验 mcu app 错误
#define BT_FAIL_WRITE_MCU_APP_FLASH								'A'//写mcu app flash错误
#define BT_FAIL_VECTOR_TABLE_INVALID							'B'//APP 向量表错误
#define BT_FAIL_CHECK_BLANK												'C'//FLASH查空错误
#define BT_DONE_CLEAR_ALL													'D'//FLASH和EPROM清除完成
#define BT_DONE_UPDATE_EPROM											'E'//更新EPROM完成
#define BT_DONE_DUMP_EPROM												'F'//下载EPROM完成
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
uint8_t RAM_Buf[BUFFER_SIZE] = {0x00};//文件读写缓冲
/*****************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern USBH_HandleTypeDef hUsbHostFS;
/*****************************************************************************/
FRESULT retUsbH;
FATFS	USBH_fatfs;
/*****************************************************************************/
FIL LogFile;//FATFS File Object 记录信息
FIL CfgFile;//FATFS File Object 下载完成信息
FIL McuFile;//FATFS File Object 单片机固件
FIL LcdFile;//FATFS File Object 屏幕固件
FIL BotFile;//FATFS File Object BOOTLOAD固件
FIL SepromFile;//FATFS File Object EPROM->UDISK
FIL LepromFile;//FATFS File Object UDISK->EPROM
/*****************************************************************************/
DIR	FileDir;//FATFS 文件目录
FILINFO FileInfo;//FATFS 文件信息
static uint8_t bootLoadState;
uint8_t usbReady;//USB DISK就绪
int32_t releaseTime0, releaseTime1, overTime, releaseCounter;
uint32_t JumpAddress;
pFunction Jump_To_Application;
/*****************************************************************************/
static void bootLoadFailHandler(uint8_t ftype);//引导故障程序
static uint32_t updateMcuApp(void);
static void clearFlash(void);
static uint32_t getOriginAppCrc(void);
static uint32_t getNewMcuAppCrc(void);
static void updateEprom(void);
static void dumpEprom(void);
static void beepDiag(uint8_t diag);
/******************************************************************************/
void bootLoadInit(void){//引导程序初始化
	SET_SPEAKER_OFF;//关闭蜂鸣器
	SET_TEC_PWM_ON;//关闭制冷
	SET_LASER0_AIM_OFF;//关闭指示激光
	SET_LASER1_AIM_OFF;
	SET_FAN5V_ON;//打开激光器冷却风扇
	//关闭激光
	SET_LASER_PWM_OFF;
	//关闭所有LED
	SET_ALARM_LED_OFF;
	SET_LASER0_LED_OFF;
	SET_LASER1_LED_OFF;
	SET_TICK_LED_OFF;
	SET_ERR_LED_OFF;
	//指示灯流水
	//
	SET_ALARM_LED_ON;
	SET_LASER0_LED_OFF;
	SET_LASER1_LED_OFF;
	HAL_Delay(500);
	//G
	SET_ALARM_LED_OFF;
	SET_LASER0_LED_ON;
	SET_LASER1_LED_OFF;
	HAL_Delay(500);
	//Y
	SET_ALARM_LED_OFF;
	SET_LASER0_LED_OFF;
	SET_LASER1_LED_ON;
	HAL_Delay(500);
	overTime = HAL_GetTick() + CONFIG_JUMP_DELAY;
	releaseTime0 = 0;
	releaseTime1 = 0;
	usbReady = FALSE;
	bootLoadState = BT_STATE_IDLE; 
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");   
	//显示输入IO状态
	if(GET_ESTOP_NC == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->ESTOP_NC      		= Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->ESTOP_NC      		= Close!\n");
	}
	//
	if(GET_LASER0_START == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->LASER0_START_NC   = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->LASER0_START_NC   = Close!\n");
	}
	//
	if(GET_LASER1_START == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->LASER1_START_NC   = Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->LASER1_START_NC   = Close!\n");
	}
	//
	if(GET_INTERLOCK_NC == GPIO_PIN_SET){//TTL=H
		printf("Bootloader:INPUT->INTERLOCK_NO    	= Open!\n");
	}
	else{//TTL=L
		printf("Bootloader:INPUT->INTERLOCK_NO    	= Close!\n");
	}
	//显示输出IO状态
	if(GET_LASER_PWM == GPIO_PIN_SET){//LASER_PWM
		printf("Bootloader:OUTPUT->LASER_PWM     		= High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LASER_PWM     		= Low!\n");
	}
	//
	if(GET_LASER0_AIM == GPIO_PIN_SET){//
		printf("Bootloader:OUTPUT->LASER0_AIM       = ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LASER0_AIM       = OFF!\n");
	
	}
	//
	if(GET_LASER1_AIM == GPIO_PIN_SET){//
		printf("Bootloader:OUTPUT->LASER1_AIM       = ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LASER1_AIM       = OFF!\n");
	}
	//
	if(GET_TEC_PWM == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->TEC_PWM       		= ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->TEC_PWM       		= OFF!\n");
	}
	//
	if(GET_FAN5V == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->FAN5V       			= ON!\n");
	}
	else{
		printf("Bootloader:OUTPUT->FAN5V						= ON!\n");
	}	
	HAL_Delay(10);
}
void bootLoadProcess(void){//bootload 执行程序
	HAL_StatusTypeDef ret;
	uint8_t fileBuff[64];
	uint32_t crcFlash, crcUdisk;
	uint32_t crcEpromMcu, crcEpromLcd;//EPROM中储存的CRC记录值
	uint32_t brByte;//实际读取的字节数
	//uint32_t bwByte;//实际写入的字节数
	//注册一个FATFS文件系统
	switch(bootLoadState){
		case BT_STATE_IDLE:{//开机等待U盘识别                             
			SET_ALARM_LED_ON;
			SET_LASER0_LED_OFF;
			SET_LASER1_LED_OFF;	
			printf("Bootloader:Start...............\n");
			listEpromTable();
			readStm32UniqueID();
			ret = epromReadDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);//从EPROM载入设备配置
			if(ret == HAL_OK){
				printf("Bootloader:Read eprom MCU FW CRC32:0x%08X\n", crcEpromMcu);
			}
			else{
				printf("Bootloader:Read eprom MCU FW CRC32 fail!\n");
			}
			ret = epromReadDword(CONFIG_EPROM_LCD_FW_CRC, &crcEpromLcd);//从EPROM载入设备配置
			if(ret == HAL_OK){
				printf("Bootloader:Read eprom LCD FW CRC32:0x%08X\n", crcEpromLcd);
			}
			else{
				printf("Bootloader:Read eprom LCD FW CRC32 fail!\n");
			}			
			printf("Bootloader:UniqueID->0x%08X%08X%08X\n", UniqueId[0], UniqueId[1], UniqueId[2]);
			printf("Bootloader:Mcu flash size->%d Kbytes\n", cpuGetFlashSize());
			printf("Bootloader:Build->%s:%s\n", __DATE__, __TIME__);
			printf("Bootloader:Bootload Start:0x%08X,End:0x%08X,Size:0x%08X\n", BOOTLOADER_FLASH_START_ADDRESS, BOOTLOADER_FLASH_END_ADDRESS ,BOOTLOADER_FLASH_SIZE);
			printf("Bootloader:Applicent Start:0x%08X,End:0x%08X,Size:0x%08X\n", APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_END_ADDRESS, APPLICATION_FLASH_SIZE);
			if((GET_INTERLOCK_NC == GPIO_PIN_SET) && (GET_ESTOP_NC == GPIO_PIN_RESET)){//拔下安全联锁、按下急停 进入APP更新流程
				bootLoadState = BT_STATE_LOAD_FWINFO;//进入USB更新APP流程
			}
			else{
				bootLoadState = BT_STATE_RUN_APP;//进入运行APP流程
			}
			break;
		}
		case BT_STATE_LOAD_FWINFO:{
			if(ret == HAL_OK){
				bootLoadState = BT_STATE_USBHOST_INIT;
				printf("Bootloader:Load eprom done!\n");
			}
			else{
				bootLoadState = BT_STATE_RUN_APP;//进入运行APP流程
			}
			break;
		}
		case BT_STATE_USBHOST_INIT:{//在USB HOST上挂载FATFS
			retUsbH = f_mount(&USBH_fatfs, FATFS_ROOT, 0);
			if(retUsbH != FR_OK){//挂载U盘失败
				printf("Bootloader:Mount Fatfs errror:%d!\n", retUsbH);
				bootLoadState = BT_STATE_RUN_APP;//跳转到APP
			}
			else{//挂载U盘成功
				printf("Bootloader:Mount Fatfs sucess!\n");
				bootLoadState = BT_STATE_WAIT_UDISK;
			}
			break;
		}
		case BT_STATE_WAIT_UDISK:{//等待USB DISK就绪
			//显示倒计时
			releaseTime0 = (overTime - (int32_t)HAL_GetTick()) / 1000;
			if(releaseTime0 != releaseTime1){
				printf("Bootloader:Wait usb disk init:%d Second!\n", releaseTime0);
				releaseTime1 = releaseTime0;
			} 
			if(releaseTime0 <= 0){
				bootLoadState = BT_STATE_READ_CFG;
			}
			else if(releaseTime0 <=2 && usbReady == TRUE){//U盘就绪
				bootLoadState = BT_STATE_READ_CFG;
			}
			break;
		}
		case BT_STATE_READ_CFG:{
			//检查U盘是否存在Flash Down文件
			retUsbH = f_open(&CfgFile, CFG_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);//读取完成信息文件
			if(retUsbH != FR_OK){//读取失败跳过固件更新直接运行程序
				printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", CFG_FIRMWARE_FILENAME, retUsbH);
				bootLoadState = BT_STATE_RUN_APP;//跳转到更新MCU APP固件
			}
			else{//读取成功检查文件内容
				printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", CFG_FIRMWARE_FILENAME, retUsbH);
				f_lseek(&CfgFile, 0);//读取指针移动到开头
				retUsbH = f_read(&CfgFile, fileBuff, 5, &brByte);
				if((retUsbH != FR_OK) || (brByte < 5)){//读取文件开头4个字节
					bootLoadFailHandler(BT_FAIL_READ_CFG);//读取CFG错误
				}
				if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '1'){//U01 更新 MCU应用
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//设备匹配
						printf("Bootloader:Start upgrade mcu application!\n");
						bootLoadState = BT_STATE_UPDATE_MCU_APP;
					}
					else{
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '2'){//U04 更新 EPROM
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//设备匹配
						printf("Bootloader:Start update eprom!\n");
						bootLoadState = BT_STATE_UPDATE_EPROM;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'D' && fileBuff[1] == '0' && fileBuff[2] == '1'){//D1 读取EPROM
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//设备匹配
						printf("Bootloader:Start dump eprom to udisk!\n");
						bootLoadState = BT_STATE_DUMP_EPROM;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else if(fileBuff[0] == 'C' && fileBuff[1] == '0' && fileBuff[2] == '1'){//U01 清除所有
					if(fileBuff[3] == DEVID_L && fileBuff[4] == DEVID_H){//设备匹配
						printf("Bootloader:Start clear flash and eprom!\n");
						bootLoadState = BT_STATE_CLEAT_ALL;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = BT_STATE_RUN_APP;
					}
				}
				else{//其它无需更新跳转到执行APP
					bootLoadState = BT_STATE_RUN_APP;
				}
				f_close(&CfgFile);
			}
			break;
		}
		case BT_STATE_UPDATE_MCU_APP:{//更新ld_mcu.bin
			crcFlash = getOriginAppCrc();//计算FLASH中APP固件CRC32
			crcUdisk = getNewMcuAppCrc();//计算U盘中MCU APP固件CRC32
			if((crcUdisk == crcEpromMcu) && (crcFlash == crcEpromMcu)){//校验码相同跳过更新
				printf("Bootloader:Check mcu app crc same,skip!\n");
				bootLoadState = BT_STATE_RESET;
				break;
			}
			crcUdisk = updateMcuApp();//写入MCU FLASH
			crcFlash = getOriginAppCrc();//校验MCU FLASH
			if(crcUdisk != crcFlash){
				bootLoadFailHandler(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
			}
			else{
				printf("Bootloader:Checksum mcu app sucess.\n");
				epromWriteDword(CONFIG_EPROM_MCU_FW_CRC, &crcEpromMcu);
				clearEprom(CLEAR_EPROM_NVRAM);//清除NVRAM掉电储存区
				printf("Bootloader:Update new crc32 sucess.\n");
			}
			bootLoadState = BT_STATE_RESET;//更新APP
			break;
		}
		case BT_STATE_UPDATE_EPROM:{//更新EPROM
			updateEprom();
			break;
		}
		case BT_STATE_DUMP_EPROM:{
			dumpEprom();
			break;
		}
		case BT_STATE_CLEAT_ALL:{//清除FLASH和EPROM全部
			clearFlash();
			clearEprom(CLEAR_EPROM_ALL);
			bootLoadFailHandler(BT_DONE_CLEAR_ALL);
			break;
		}
		case BT_STATE_RESET:{//复位
			printf("Bootloader:System Reset\n");
			__set_FAULTMASK(1);
			NVIC_SystemReset();/* Software reset */
			break;
		}
		case BT_STATE_RUN_APP:{//运行应用程序
			HAL_FLASH_Lock();//锁定FLASH
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
				SET_ALARM_LED_OFF;
				SET_LASER0_LED_OFF;
				SET_LASER1_LED_OFF;
				__disable_irq();
				SysTick->CTRL = 0;//关键代码
				//关闭中断                                    				
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

static void bootLoadFailHandler(uint8_t ftype){//引导错误程序
	MX_DriverVbusFS(FALSE);//关闭USB VBUS
	printf("Bootloader:SYS_ERR_LED->On!\n");
	SET_ALARM_LED_OFF;
	SET_LASER0_LED_OFF;
	SET_LASER1_LED_OFF;
	switch(ftype){
		case BT_FAIL_READ_CFG:{//从U盘读取CFG失败
			printf("Bootloader:FailHandler,Read %s fail!.\n", CFG_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_CFG);
			};
		}
		case BT_FAIL_READ_LMCU_APP:{//从U盘读取MCU APP失败
			printf("Bootloader:FailHandler,Read %s fail!.\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_READ_LMCU_APP);
			};
		}
		case BT_FAIL_ERASE_MCU_APP:{//擦除MCU APP FLASH区域失败
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
		case BT_FAIL_LMCU_APP_CHECK:{//lmcu.bin 检查错误
			printf("Bootloader:FailHandler,%s size is invalid!\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_LMCU_APP_CHECK);
			};
		}
		case BT_FAIL_CHECKSUM_MCU_APP_FLASH:{//校验 lmcu.bin 错误
			printf("Bootloader:FailHandler,Verify %s fail!.\n", LMCU_FIRMWARE_FILENAME);
			while(1){
				beepDiag(BT_FAIL_CHECKSUM_MCU_APP_FLASH);
			};
		}
		case BT_FAIL_VECTOR_TABLE_INVALID:{//APP无效向量表
			printf("Bootloader:FailHandler,App vector table invalid.\n");
			while(1){
				beepDiag(BT_FAIL_VECTOR_TABLE_INVALID);
			};
		}
		case BT_FAIL_CHECK_BLANK:{//FLASH 查空错误
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
static void clearFlash(void){//清除MCU FLASH
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
	if (FLASH_If_EraseApplication() != 0x00){//擦除APP FLASH区域失败
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_SIZE);//FLASH 查空
}
static uint32_t getNewMcuAppCrc(void){//获取待更新MCU APP CRC32
	uint32_t crc32;
	uint32_t i;
	uint8_t readflag = TRUE;
	uint16_t bytesread;//实际文件读取字节数
	retUsbH = f_open(&McuFile, LMCU_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//读取失败
		bootLoadFailHandler(BT_FAIL_READ_LMCU_APP);			
	}
	if(f_size(&McuFile) > APPLICATION_FLASH_SIZE){//MCU固件大于FLSAH容量
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
	for(i = LastPGAddress;i < APPLICATION_FLASH_END_ADDRESS;i ++){//补完剩余CRC
		crc32 = crc32CalculateAdd(0xFF);
	}
	f_close(&McuFile);
	return crc32;
}

static uint32_t updateMcuApp(void){//更新MCU APP
	uint32_t crc32;
	uint32_t i;
	uint32_t programcounter = 0x00;
	uint8_t readflag = TRUE;
	uint32_t bytesread;//实际文件读取字节数
	retUsbH = f_open(&McuFile, LMCU_FIRMWARE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(retUsbH != FR_OK){//读取失败
		bootLoadFailHandler(BT_FAIL_READ_LMCU_APP);			
	}
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", LMCU_FIRMWARE_FILENAME, retUsbH);
	if(f_size(&McuFile) > APPLICATION_FLASH_SIZE){//MCU固件大于FLSAH容量
		bootLoadFailHandler(BT_FAIL_LMCU_APP_CHECK);
	}
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGSERR|FLASH_FLAG_WRPERR);
	if (FLASH_If_EraseApplication() != 0x00){//擦除APP FLASH区域失败
		bootLoadFailHandler(BT_FAIL_ERASE_MCU_APP);
	}
	checkBlank(APPLICATION_FLASH_START_ADDRESS, APPLICATION_FLASH_SIZE);//FLASH 查空
	printf("Bootloader:Erase mcu application sucess.\n");
	RamAddress = (uint32_t)&RAM_Buf;//获取RAM缓存区地址
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
		SET_ALARM_LED_OFF;;
		/* Program flash memory */
		for(programcounter = 0; programcounter < TmpReadSize; programcounter += 4){
			/* Write word into flash memory */
			if(FLASH_If_Write((LastPGAddress + programcounter), *(uint32_t *) (RamAddress + programcounter)) != 0x00){
				bootLoadFailHandler(BT_FAIL_WRITE_MCU_APP_FLASH);//写入FLASH错误
			}
		}
		/* Update last programmed address value */
		LastPGAddress += TmpReadSize;
		SET_ALARM_LED_ON;
	}
	for(i = LastPGAddress;i < APPLICATION_FLASH_END_ADDRESS;i ++){//补完剩余CRC
		crc32 = crc32CalculateAdd(0xFF);
	}
	HAL_FLASH_Lock();
	printf("Bootloader:Write mcu app finish.\n");
	f_close(&McuFile);
	return crc32;
}

static void updateEprom(void){//UDISK->EPROM
	HAL_StatusTypeDef ret;
	uint32_t brByte;
	retUsbH = f_open(&LepromFile, LOAD_EPROM_FILENAME, FA_OPEN_EXISTING | FA_READ);//读取完成信息文件
	if(retUsbH != FR_OK){//读取失败跳过固件更新直接运行程序
		printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", LOAD_EPROM_FILENAME, retUsbH);
				bootLoadFailHandler(BT_FAIL_READ_LEROM_BIN);
	}
	else{//读取成功检查文件内容
		printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", LOAD_EPROM_FILENAME, retUsbH);
		f_lseek(&LepromFile, 0);//读取指针移动到开头
		retUsbH = f_read(&LepromFile, RAM_Buf, CONFIG_EPROM_SIZE, &brByte);
		if((retUsbH != FR_OK) || (brByte !=  CONFIG_EPROM_SIZE)){
			bootLoadFailHandler(BT_FAIL_READ_LEROM_BIN);
		}
		f_close(&LepromFile);
		ret = epromWrite(0, RAM_Buf, CONFIG_EPROM_SIZE);//写入EPROM
		if(ret != HAL_OK){
			bootLoadFailHandler(BT_FAIL_WRITE_EPROM);
		}
		bootLoadFailHandler(BT_DONE_UPDATE_EPROM);
	}
}
static void dumpEprom(void){//下载EPROM信息到U盘
	uint32_t wrByte;
	epromRead(0x0, RAM_Buf, CONFIG_EPROM_SIZE);
	retUsbH = f_open(&SepromFile, LOAD_EPROM_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
	if(retUsbH != FR_OK){//打开失败
		bootLoadFailHandler(BT_FAIL_WRITE_SEROM_BIN);
	}
	retUsbH = f_write(&SepromFile, RAM_Buf, CONFIG_EPROM_SIZE, &wrByte);
	if(retUsbH != FR_OK){//写入失败
		bootLoadFailHandler(BT_FAIL_WRITE_SEROM_BIN);
	}
	bootLoadFailHandler(BT_DONE_DUMP_EPROM);
}
static uint32_t getOriginAppCrc(void){//计算MCU APP CRC32
	uint8_t val;
	uint32_t i;
	uint32_t crc32;
	crc32Clear();
	for(i = APPLICATION_FLASH_START_ADDRESS;i < APPLICATION_FLASH_END_ADDRESS;i ++){
		val = *(__IO uint8_t*)(i);
		crc32 = crc32CalculateAdd(val);//CRC32计算连续字节
	}
	return crc32;	
}

static void beepDiag(uint8_t diag){//蜂鸣器诊断声音 摩尔斯电码
	//关闭USB VBUS
	switch(diag){
		case '0':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '1':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		};
		case '2':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		};
		case '3':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '4':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '5':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '6':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '7':{	
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '8':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case '9':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'A':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'B':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;			
		}
		case 'C':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'D':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'E':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'F':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'G':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'H':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;			
		}
		case 'I':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'J':{
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'K':{
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'L':{//．━ ．．
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//.
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_SHORT_TIME);SET_ALARM_LED_OFF;
			break;
		}
		case 'M':{//━ ━
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;HAL_Delay(MORSECODE_SPACE_TIME);
			//-
			SET_ALARM_LED_ON;HAL_Delay(MORSECODE_LONG_TIME);SET_ALARM_LED_OFF;
			break;
		}
		default:break;
	}
	HAL_Delay(3000);
}
