
/* �������� ------------------------------------------------------------------*/
/* �� ------------------------------------------------------------------------*/

#define UART_UPDATE //UART_UPDATE���ڳ�ʼ����ʹ��


/* Pointer to file system object *///���������������붨��Ϊȫ�ֱ���������������Ӧ����ջ̫С��ԭ��
FATFS fs;				//�ļ�ϵͳ����
FRESULT res;            //����ֵ
FIL fsrc;               //file objects//�ļ������ָ��

int8_t USART_RX_STA=0;
uint8_t USART_RX_BUF[10];    //��������

uint8_t sendbuffer[2048+4];  //���ݰ�



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
    int8_t file_init_path[] = "private";                    //��ʼ�ļ���private



    res= updateLcdApp(file_init_path);
    if(res!=FR_OK){
       
        return -1;
    }
    
  	
}



