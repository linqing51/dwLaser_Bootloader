
/* 类型声明 ------------------------------------------------------------------*/
/* 宏 ------------------------------------------------------------------------*/

#define UART_UPDATE //UART_UPDATE串口初始化和使能


/* Pointer to file system object *///以下三个变量必须定义为全局变量，否则编译出错，应该是栈太小的原因
FATFS fs;				//文件系统对象
FRESULT res;            //返回值
FIL fsrc;               //file objects//文件对象的指针

int8_t USART_RX_STA=0;
uint8_t USART_RX_BUF[10];    //串口数据

uint8_t sendbuffer[2048+4];  //数据包



void USART1_IRQHandler(void)
{
    uint16_t USART1_Res;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART1_Res=USART_ReceiveData(USART1);
//        dp_display_text((int8_t *)&USART1_Res);
        USART_RX_BUF[USART_RX_STA&0X3FFF]=USART1_Res ;
        USART_RX_STA++;

    
    } 
}

int main(void)
{
    int8_t file_init_path[] = "private";                    //初始文件夹private



    res= updateLcdApp(file_init_path);
    if(res!=FR_OK){
       
        return -1;
    }
    
  	
}



