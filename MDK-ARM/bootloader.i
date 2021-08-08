#line 1 "..\\Bootloader\\bootLoader.c"
#line 1 "..\\Bootloader\\bootLoader.h"


 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


















  

 







 
#line 1 "../Core/Inc/stm32f4xx_hal_conf.h"



















 

 







 
 

 


 



 
 
 
 
 

 
 
 
 
 
 
 
 
 

 
 
 
 
 
 
 
 
 
 

 
 
 
 
 
 

 
 
 
 
 
 
 
 
#line 89 "../Core/Inc/stm32f4xx_hal_conf.h"

 




 












 






 







 












 





 

 


 
#line 154 "../Core/Inc/stm32f4xx_hal_conf.h"

#line 193 "../Core/Inc/stm32f4xx_hal_conf.h"

 



 
 

 

 

 
#line 212 "../Core/Inc/stm32f4xx_hal_conf.h"

 





 

 

 

 





 




#line 246 "../Core/Inc/stm32f4xx_hal_conf.h"





 





 




 



 


 

#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

















 

 







 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"


















 

 







 
#line 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



























 



 



 
    






   


 
  


 






 
#line 95 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
   


 
#line 107 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 
#line 119 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 



 

#line 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"























 



 



 
    









 



 








 
  


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,      
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  DFSDM1_FLT0_IRQn            = 61,      
  DFSDM1_FLT1_IRQn            = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  CAN3_TX_IRQn                = 74,      
  CAN3_RX0_IRQn               = 75,      
  CAN3_RX1_IRQn               = 76,      
  CAN3_SCE_IRQn               = 77,      
  RNG_IRQn                    = 80,      
  FPU_IRQn                    = 81,      
  UART7_IRQn                  = 82,      
  UART8_IRQn                  = 83,      
  SPI4_IRQn                   = 84,      
  SPI5_IRQn                   = 85,      
  SAI1_IRQn                   = 87,      
  UART9_IRQn                  = 88,      
  UART10_IRQn                 = 89,      
  QUADSPI_IRQn                = 92,      
  FMPI2C1_EV_IRQn             = 95,      
  FMPI2C1_ER_IRQn             = 96,      
  LPTIM1_IRQn                 = 97,      
  DFSDM2_FLT0_IRQn            = 98,      
  DFSDM2_FLT1_IRQn            = 99,      
  DFSDM2_FLT2_IRQn            = 100,     
  DFSDM2_FLT3_IRQn            = 101      
} IRQn_Type;



 

#line 1 "../Drivers/CMSIS/Include/core_cm4.h"
 




 
















 










#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 35 "../Drivers/CMSIS/Include/core_cm4.h"

















 




 



 

#line 1 "../Drivers/CMSIS/Include/cmsis_version.h"
 




 
















 










 
#line 64 "../Drivers/CMSIS/Include/core_cm4.h"

 









 
#line 87 "../Drivers/CMSIS/Include/core_cm4.h"

#line 161 "../Drivers/CMSIS/Include/core_cm4.h"

#line 1 "../Drivers/CMSIS/Include/cmsis_compiler.h"
 




 
















 




#line 29 "../Drivers/CMSIS/Include/cmsis_compiler.h"



 
#line 1 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 




 
















 









 













   
   


 
#line 103 "../Drivers/CMSIS/Include/cmsis_armcc.h"

 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}









 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1U);
}









 
static __inline uint32_t __get_FPSCR(void)
{


  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{


  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);



}


 


 



 




 






 







 






 








 










 










 






                  





 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int16_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 532 "../Drivers/CMSIS/Include/cmsis_armcc.h"







 











 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 


#line 780 "../Drivers/CMSIS/Include/cmsis_armcc.h"

   


 



 



#line 851 "../Drivers/CMSIS/Include/cmsis_armcc.h"











 


#line 35 "../Drivers/CMSIS/Include/cmsis_compiler.h"




 
#line 263 "../Drivers/CMSIS/Include/cmsis_compiler.h"




#line 163 "../Drivers/CMSIS/Include/core_cm4.h"

















 
#line 207 "../Drivers/CMSIS/Include/core_cm4.h"

 






 
#line 223 "../Drivers/CMSIS/Include/core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:1;                
    uint32_t ICI_IT_1:6;                  
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t ICI_IT_2:2;                  
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 

































 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 


















 





















 


















 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile const  uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 






 





















 






 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;



 









 









 



 









 






























 








 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 







 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1562 "../Drivers/CMSIS/Include/core_cm4.h"

#line 1571 "../Drivers/CMSIS/Include/core_cm4.h"









 










 


 



 





 

#line 1625 "../Drivers/CMSIS/Include/core_cm4.h"

#line 1635 "../Drivers/CMSIS/Include/core_cm4.h"




 
#line 1646 "../Drivers/CMSIS/Include/core_cm4.h"










 
static __inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)  );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}







 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)]               >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}










 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  vectors[(int32_t)IRQn + 16] = vector;
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return vectors[(int32_t)IRQn + 16];
}





 
__declspec(noreturn) static __inline void __NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 

 



#line 1 "../Drivers/CMSIS/Include/mpu_armv7.h"





 
















 
 





 



#line 62 "../Drivers/CMSIS/Include/mpu_armv7.h"

#line 69 "../Drivers/CMSIS/Include/mpu_armv7.h"





 












   














 




  











                          









  










  












  




 




 




 




 





 
typedef struct {
  uint32_t RBAR; 
  uint32_t RASR; 
} ARM_MPU_Region_t;
    


 
static __inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

}


 
static __inline void ARM_MPU_Disable(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL  &= ~(1UL );
}



 
static __inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}




    
static __inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





    
static __inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}





 
static __inline void orderedCpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i) 
  {
    dst[i] = src[i];
  }
}




 
static __inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt) 
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    orderedCpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  orderedCpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}

#line 1961 "../Drivers/CMSIS/Include/core_cm4.h"




 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if      ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;            
  }
  else
  {
    return 0U;            
  }
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                               










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 179 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
#line 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h"

































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 180 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
#line 181 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;     
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 
typedef struct
{
  volatile uint32_t FLTCR1;       
  volatile uint32_t FLTCR2;       
  volatile uint32_t FLTISR;       
  volatile uint32_t FLTICR;       
  volatile uint32_t FLTJCHGR;     
  volatile uint32_t FLTFCR;       
  volatile uint32_t FLTJDATAR;    
  volatile uint32_t FLTRDATAR;    
  volatile uint32_t FLTAWHTR;     
  volatile uint32_t FLTAWLTR;     
  volatile uint32_t FLTAWSR;      
  volatile uint32_t FLTAWCFR;     
  volatile uint32_t FLTEXMAX;     
  volatile uint32_t FLTEXMIN;     
  volatile uint32_t FLTCNVTIMR;   
} DFSDM_Filter_TypeDef;



 
typedef struct
{
  volatile uint32_t CHCFGR1;      
  volatile uint32_t CHCFGR2;      
  volatile uint32_t CHAWSCDR;    
 
  volatile uint32_t CHWDATAR;     
  volatile uint32_t CHDATINR;     
} DFSDM_Channel_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;





 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef;



 

typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;


 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint32_t BSRR;      
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED;      
  volatile uint32_t CFGR2;         
  volatile uint32_t CMPCR;         
  uint32_t      RESERVED1[2];  
  volatile uint32_t CFGR;          
  volatile uint32_t MCHDLYCR;      
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;        
  volatile uint32_t OAR2;        
  volatile uint32_t DR;          
  volatile uint32_t SR1;         
  volatile uint32_t SR2;         
  volatile uint32_t CCR;         
  volatile uint32_t TRISE;       
  volatile uint32_t FLTR;        
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t OAR1;         
  volatile uint32_t OAR2;         
  volatile uint32_t TIMINGR;      
  volatile uint32_t TIMEOUTR;     
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t PECR;         
  volatile uint32_t RXDR;         
  volatile uint32_t TXDR;         
} FMPI2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
  uint32_t      RESERVED7;      
  volatile uint32_t DCKCFGR;        
  volatile uint32_t CKGATENR;       
  volatile uint32_t DCKCFGR2;       
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;



 
  
typedef struct
{
  volatile uint32_t GCR;       
} SAI_TypeDef;

typedef struct
{
  volatile uint32_t CR1;       
  volatile uint32_t CR2;       
  volatile uint32_t FRCR;      
  volatile uint32_t SLOTR;     
  volatile uint32_t IMR;       
  volatile uint32_t SR;        
  volatile uint32_t CLRFR;     
  volatile uint32_t DR;        
} SAI_Block_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;                  
  volatile uint32_t CLKCR;                  
  volatile uint32_t ARG;                    
  volatile uint32_t CMD;                    
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;                 
  volatile uint32_t DLEN;                   
  volatile uint32_t DCTRL;                  
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;                    
  volatile uint32_t MASK;                   
  uint32_t      RESERVED0[2];           
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];          
  volatile uint32_t FIFO;                   
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t DCR;       
  volatile uint32_t SR;        
  volatile uint32_t FCR;       
  volatile uint32_t DLR;       
  volatile uint32_t CCR;       
  volatile uint32_t AR;        
  volatile uint32_t ABR;       
  volatile uint32_t DR;        
  volatile uint32_t PSMKR;     
  volatile uint32_t PSMAR;     
  volatile uint32_t PIR;       
  volatile uint32_t LPTR;      
} QUADSPI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  uint32_t  Reserved5[3];              
  volatile uint32_t GHWCFG3;               
  uint32_t  Reserved6;                 
  volatile uint32_t GLPMCFG;               
  uint32_t  Reserved;                  
  volatile uint32_t GDFIFOCFG;             
  uint32_t  Reserved43[40];            
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;



 
typedef struct 
{
  volatile uint32_t DCFG;             
  volatile uint32_t DCTL;             
  volatile uint32_t DSTS;             
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;          
  volatile uint32_t DOEPMSK;          
  volatile uint32_t DAINT;            
  volatile uint32_t DAINTMSK;         
  uint32_t  Reserved20;           
  uint32_t Reserved9;             
  volatile uint32_t DVBUSDIS;         
  volatile uint32_t DVBUSPULSE;       
  volatile uint32_t DTHRCTL;          
  volatile uint32_t DIEPEMPMSK;       
  volatile uint32_t DEACHINT;         
  volatile uint32_t DEACHMSK;         
  uint32_t Reserved40;            
  volatile uint32_t DINEP1MSK;        
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;       
} USB_OTG_DeviceTypeDef;



 
typedef struct 
{
  volatile uint32_t DIEPCTL;            
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;            
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;           
  volatile uint32_t DIEPDMA;            
  volatile uint32_t DTXFSTS;            
  uint32_t Reserved18;              
} USB_OTG_INEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;            
  volatile uint32_t HCSPLT;            
  volatile uint32_t HCINT;             
  volatile uint32_t HCINTMSK;          
  volatile uint32_t HCTSIZ;            
  volatile uint32_t HCDMA;             
  uint32_t Reserved[2];            
} USB_OTG_HostChannelTypeDef;



 
typedef struct
{
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t IER;          
  volatile uint32_t CFGR;         
  volatile uint32_t CR;           
  volatile uint32_t CMP;          
  volatile uint32_t ARR;          
  volatile uint32_t CNT;          
  volatile uint32_t OR;           
} LPTIM_TypeDef;



 



 
#line 927 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



 





 
#line 971 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 981 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 1015 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1046 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



 



 

 


#line 1072 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"






 



   
#line 1124 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 1191 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 
#line 1233 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1287 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
  
 
#line 1337 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1393 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1455 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 




 




 
#line 1526 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1576 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1626 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 
#line 1693 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1713 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 


 
#line 1752 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1760 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



 
 
 
 
 
 
 
#line 1802 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 1830 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1880 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 1893 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 1906 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1920 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1934 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1979 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 1990 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 1997 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 2004 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2033 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
 
#line 2052 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2063 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2077 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2091 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2108 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2119 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2133 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2147 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2164 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

   
#line 2175 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2189 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2203 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2217 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2228 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2242 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2256 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2270 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2281 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2295 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2309 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
#line 2318 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2407 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2496 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2585 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2674 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 2773 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2871 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 2969 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3067 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3165 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3263 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3361 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3459 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3557 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3655 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3753 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3851 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 3949 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4047 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4145 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4243 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4341 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4439 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4537 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4635 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4733 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4831 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 4929 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5027 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5125 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5223 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5321 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5419 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 





 





 




 
 
 
 
 


 

 
#line 5461 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 5468 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 5482 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 5498 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 5505 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 5519 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 5526 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5534 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 




 




 
#line 5572 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5580 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5588 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 
#line 5606 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 

 

 
#line 5657 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5681 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 5694 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

 
#line 5745 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5774 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5803 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5817 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 5836 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5844 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5855 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5863 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5871 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5879 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 5888 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5896 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 5904 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 
 
 
 
 
 
#line 5994 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 6020 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

  
#line 6039 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

  
#line 6101 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

  
#line 6163 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

  
#line 6225 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

  
#line 6287 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 





 
 
 
 
 
 
#line 6382 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6411 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6485 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6511 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6585 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6659 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6733 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6807 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 6825 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 6847 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6873 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6905 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 6913 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 6958 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
                                             
 
#line 6975 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 6988 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"













#line 7040 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7048 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"













#line 7094 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7102 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"













#line 7148 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7156 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"













#line 7202 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7211 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7219 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7231 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7239 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7247 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7255 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7270 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7278 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7290 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7298 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7306 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7314 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7329 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7337 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7349 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7357 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7365 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7373 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7388 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7396 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7408 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7416 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7424 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7432 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7447 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7455 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7467 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7475 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7490 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7498 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7510 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7518 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7533 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7541 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7553 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7561 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 7576 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7584 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7596 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 7604 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
 
 
 
 
 
#line 7697 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 7748 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7766 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7848 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7898 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 7980 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8030 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8080 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8098 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8148 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 8165 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8263 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8345 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 8397 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 8454 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8496 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8554 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8596 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
 
 
 
 
 
#line 8646 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8657 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 8673 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



#line 8708 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 
#line 8720 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 8769 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8795 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8806 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 8819 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 8883 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



 
#line 8922 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8933 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8944 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8961 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 8978 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9031 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9060 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 






 
 
 
 
 
 




 
#line 9095 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 9108 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
 
 
 
 
 
#line 9132 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9139 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9175 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 9205 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
 
 
 
 
 
#line 9274 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9293 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9322 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9336 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 9406 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 




 




 




 




 




 




 




 
 
 
 
 
 
#line 9453 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9462 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9474 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9493 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 


#line 9504 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9515 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9528 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 9542 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9550 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 


#line 9561 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 










 










 
#line 9593 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9603 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9611 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 9625 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 9641 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 







#line 9656 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9663 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 9689 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9711 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 9730 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 
#line 9769 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9777 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 9784 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 9871 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9927 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 9962 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 


 


#line 9974 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 


 


#line 9987 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10076 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10135 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10179 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 10188 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10196 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10285 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10344 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10358 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 10371 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10403 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 



 
#line 10420 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10431 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 10444 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 10461 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 10473 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 10482 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 10489 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 



#line 10514 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 10523 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10549 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10567 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
 
 
 
 
 
#line 10581 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10598 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 10646 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10690 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10760 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 


 
#line 10813 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10821 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 10834 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10904 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 10974 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 
#line 10992 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11035 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11065 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 11093 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11141 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 


 
#line 11156 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11168 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 

    
 
 
 
 
 
 












 












#line 11309 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11316 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 11338 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11346 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11354 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11367 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11377 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"











 
#line 11400 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11411 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11421 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 


 
#line 11433 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 11447 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 
#line 11474 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11497 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11504 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11527 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 





 
 
 
 
 
 






 
#line 11559 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 11572 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 










#line 11601 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 




 




 




 




 
#line 11655 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11663 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11676 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 11749 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11784 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11852 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 
 
 
 
 



 
#line 11881 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11888 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 11919 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11942 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 11971 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 






























#line 12032 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12043 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 





 




 
#line 12073 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


 
#line 12084 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12096 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12108 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12120 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12134 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12146 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12158 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12170 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12182 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12196 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12208 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12220 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12232 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12244 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12258 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12270 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12282 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12294 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"



 
#line 12306 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12314 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 12321 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12329 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 12387 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 12409 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

















 
#line 12436 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12443 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12468 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12476 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12483 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





#line 12495 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 12508 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12555 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12593 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12619 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 






#line 12633 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12640 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"











#line 12657 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12664 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 







#line 12684 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 12698 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 






#line 12712 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12719 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"











#line 12736 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12743 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 







#line 12763 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 12777 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12824 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 




 




 




 




 




 
#line 12877 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 12902 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12912 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 12921 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 






#line 12944 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 12972 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 12995 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13018 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 






















#line 13048 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 13055 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 13080 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13091 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 
#line 13122 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 





 
 
 
 
 
 
#line 13165 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 13178 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13225 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13248 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"











 
#line 13296 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13309 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 
 
 
 
 
 
#line 13330 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 13338 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





 
#line 13354 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 13362 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"






 







 





 
 
 
 
 
 
#line 13394 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13408 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 13476 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13493 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
 
 
 
 
#line 13554 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

#line 13565 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

#line 13576 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 13587 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"





















 
#line 13618 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13641 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13655 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 13677 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 13690 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




#line 13707 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13729 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

#line 13793 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13810 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


#line 13826 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13852 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 13868 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 13880 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 13923 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 14008 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14094 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14102 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 14121 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14129 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 
#line 14153 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




#line 14175 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14186 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14194 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14210 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14226 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 14239 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14271 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14279 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 14331 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14360 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14389 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14398 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14406 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 14447 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14455 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14469 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14478 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14504 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




#line 14523 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"













#line 14555 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

#line 14568 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14579 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14591 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14626 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14667 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14702 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

#line 14714 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 14729 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 




 




 
#line 14752 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 

#line 14793 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14828 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 

#line 14836 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







 
#line 14853 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
 
#line 14866 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"







#line 14880 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14888 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14896 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"


  



 



 

 




 




 
#line 14925 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

#line 14938 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 


 



 
#line 14962 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 14972 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 


 









 


 


 


 


 



 








 
#line 15024 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 


 
#line 15042 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15056 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15066 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15074 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15082 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



 
#line 15094 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15104 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15112 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15120 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15128 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15138 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15148 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 



 
#line 15159 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 15222 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15234 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15242 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15256 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 




 
#line 15269 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15279 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15287 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15297 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 
#line 15307 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 



 
#line 15320 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 
#line 15327 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"
 



 





 
#line 15348 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 


 




 


 





 
#line 15377 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

 


 


 


 


 



 



 

 







 



#line 15420 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f413xx.h"

















 



 



 









 
#line 172 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"








 



  
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;



 




 



















 

#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


















  

 
#line 297 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

 
#line 235 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"









 



 
  



 
#line 31 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


















 

 







 
 
 



 








 



 
#line 89 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 97 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 





 



 
#line 135 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 202 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 



 



 






 



 

#line 238 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"













 



 
#line 270 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





#line 314 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 381 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 478 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 495 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 520 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 
#line 539 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 


















#line 589 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





#line 600 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 607 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










 



 
#line 631 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 640 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 647 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 759 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 819 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 
#line 842 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 



 










 



 


































 



 


#line 980 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 

 
#line 1002 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 












 



 




























#line 1058 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 















 




 
#line 1099 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 









#line 1129 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 



#line 1167 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1177 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1196 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










#line 1223 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 

























 




 








 



 




 



 
#line 1303 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 1320 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1332 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1363 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 











 

#line 1411 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

 



 



 



 
#line 1439 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 





































 



 
#line 1504 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
#line 1519 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 

 



 







#line 1546 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1557 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"
 

 



 

#line 1587 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1595 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 1611 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 

 



 





 



 



 



 
#line 1651 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 



 



 






 




 



 

 



 





 



 
#line 1712 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"









 




 
#line 1740 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1761 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1772 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1781 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1794 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1803 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 







 



 
#line 1839 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1854 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


#line 1887 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
#line 2054 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



#line 2064 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 

#line 2078 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 







 



 

#line 2101 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 2129 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 










 



 














 




 




 




 







 




 
#line 2207 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 
#line 2251 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2265 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 







#line 2538 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2552 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2769 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2783 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2790 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2811 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2959 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 



#line 2984 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3005 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3122 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3131 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3148 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3163 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 3192 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

















#line 3218 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





#line 3245 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3252 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3261 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3294 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3312 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"












#line 3330 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3351 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3359 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 
#line 3382 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3410 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3425 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




#line 3461 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"
 




#line 3491 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3498 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3510 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 

#line 3524 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 
#line 3545 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 











 



 












#line 3618 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3627 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3636 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 








#line 3669 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 



 

#line 3686 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




 



 
#line 3720 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 
#line 3747 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 



 







 

#line 32 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 33 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"

 



   
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

 




























 


#line 103 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"







#line 118 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"


 
#line 140 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"




  









 


#line 173 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"



  



 


#line 190 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

 
 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       
 

  uint32_t PLLN;       

 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 



  uint32_t PLLR;       


 

}RCC_PLLInitTypeDef;

#line 176 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 202 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




 
typedef struct
{
  uint32_t PLLI2SM;    
 

  uint32_t PLLI2SN;    
 

  uint32_t PLLI2SQ;    

 
                           
  uint32_t PLLI2SR;    

 
}RCC_PLLI2SInitTypeDef;



 
typedef struct
{
  uint32_t PeriphClockSelection; 
 

  RCC_PLLI2SInitTypeDef PLLI2S;  
 
  

  uint32_t PLLDivR;              

 

  uint32_t PLLI2SDivR;           

 

                                      
  uint32_t I2sApb1ClockSelection;    
 

  uint32_t I2sApb2ClockSelection;    
 

  uint32_t RTCClockSelection;      
 

  uint32_t SdioClockSelection;    
 

  uint32_t Fmpi2c1ClockSelection;  
 

  uint32_t Clk48ClockSelection;     
 
  
  uint32_t Dfsdm1ClockSelection;    
 

  uint32_t Dfsdm1AudioClockSelection;
 
  

  uint32_t Dfsdm2ClockSelection;    
 

  uint32_t Dfsdm2AudioClockSelection;
 
  
  uint32_t Lptim1ClockSelection;   
 
  
  uint32_t SaiAClockSelection;     
 

  uint32_t SaiBClockSelection;     
 


  uint32_t PLLI2SSelection;      
 

  uint8_t TIMPresSelection;      
 
}RCC_PeriphCLKInitTypeDef;


#line 378 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 421 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


  

 


 



 
 
#line 454 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 464 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 481 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
    
 
#line 495 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 507 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 519 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 


 
#line 537 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 
#line 548 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 



 
#line 562 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 



 
#line 575 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 

#line 600 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
#line 629 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 722 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




 






  



 






  
      


 






 
      



 




 



 




 






 




 



 




 



 




 



 






 



 






 



 





 



 




 



 




 


#line 890 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"








 




 










 




 










 






 




#line 954 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 
     
 


 
 
#line 2007 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 2875 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 3256 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 3526 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 3902 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 4706 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 







 
#line 4769 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 
#line 4791 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 4803 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 
#line 4825 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                        
#line 4834 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                     



                                        



 







 




                                        


          




   







 
#line 4887 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






 







 










 
  






 
#line 4981 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 4998 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                        
#line 5087 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                        
#line 5118 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                        


 







 
#line 5160 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5191 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


   






 
#line 5284 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
#line 5302 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 
#line 5330 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5348 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
  



 
#line 5367 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5379 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 







                                        







 




  







#line 5428 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 
#line 5465 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5495 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 
#line 5519 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5536 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 
#line 5558 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5569 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 




                                        







 








 










 








 
#line 5653 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5684 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 
#line 5713 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                        
#line 5731 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 

 

 



































 
#line 5816 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
                             
 








 



#line 5870 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






















 
#line 5918 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5944 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5967 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 6057 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 






 






 


                                 
#line 6095 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6107 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 6133 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                 
#line 6166 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6331 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
#line 6380 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 





 






 









 








 



 





 






 








 







 

      










 

      







 












 

      







 









 








 


      







 








 









 








 









 

      






 







 







 






 







 






 




#line 6669 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      

















      






 

#line 6720 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6733 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 

 


 



 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);





void HAL_RCCEx_SelectLSEMode(uint8_t Mode);


HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void);







  



 
 
 
 


 




 
   
#line 6792 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






 






 





 
#line 6818 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

 
#line 6832 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      


 

 


      



#line 6853 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




 



 

 


 


 
#line 6880 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      



























      



      


#line 6934 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




 




#line 6962 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 7014 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 7035 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




    




 




                                              























































      













 



 



  



   






 
#line 34 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"



 



 

 


 



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  
 

  uint32_t LSIState;             
 

  RCC_PLLInitTypeDef PLL;         
}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 







 



 





 



 





 



 






 



 




 



 





 



 






 



 




 



 






 





 






 





 






 



 
#line 236 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 







 



 
#line 289 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 




 



 






 



 







 



 
#line 335 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 









 
 





 


 
#line 366 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 

 


 







 
#line 428 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 435 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 452 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 459 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 519 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 527 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 545 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 553 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 620 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 629 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 648 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 657 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
#line 672 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 680 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
#line 696 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 705 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
#line 722 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 732 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
#line 750 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 757 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
#line 776 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 784 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
#line 804 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 813 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 















 









 




 



 








 




 



 





















 
#line 912 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 


















 
#line 955 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 



 
























 













 







 






 




 



 







 










 










 



 



 









 










 







 



 



 















 




















 




 




 











 












 













 













 




 



















 





 



 

 
 

 



 
 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);


 



 
 
void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void HAL_RCC_NMI_IRQHandler(void);

 
void HAL_RCC_CSSCallback(void);



 



 

 
 
 


 




 

 
 



 


 



 
 



 



 
 




 


 


 


 












 



 

 


 



 






















#line 1413 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"































 



 



 



 







 
#line 273 "../Core/Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"



 



  

 


 



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


 

 



  



 
#line 103 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"




 










  







    



 





 




 






 

 


   





 
  


 

 


 






 







 







 







 







 



 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 



  

 
 


 
  


 

 
#line 166 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 281 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 387 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 483 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
#line 573 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 682 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 



  







  






  






  








  







  








 
#line 745 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"


  
#line 755 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



  
#line 767 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 
#line 780 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 
#line 790 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 





   


  





  

   


  



 
#line 908 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
#line 982 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
#line 1102 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1225 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 


  



 

 


 


 

 


 


 

 
 
 


 


 

 


 


 
#line 1277 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

#line 1291 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"







#line 1305 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

#line 1333 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 



   
 
#line 1367 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1394 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1418 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1441 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
#line 1461 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 
 




 
#line 1486 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1518 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1547 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 



 

 



 



  



 

 


 



 



  



  
  






 
#line 215 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  
 
 
 


 



 

 


 
#line 282 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"


 

 


 



 



  



 







 
#line 277 "../Core/Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"



 




 

 



 
typedef enum
{
  HAL_EXTI_COMMON_CB_ID          = 0x00U
} EXTI_CallbackIDTypeDef;



 
typedef struct
{
  uint32_t Line;                     
  void (* PendingCallback)(void);    
} EXTI_HandleTypeDef;



 
typedef struct
{
  uint32_t Line;      
 
  uint32_t Mode;      
 
  uint32_t Trigger;   
 
  uint32_t GPIOSel;   

 
} EXTI_ConfigTypeDef;



 

 


 



 
#line 125 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"



 



 





 



 







 




 
#line 183 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"



 



 

 


 



 

 


 


 








 




 




 




 








 

 


 














#line 310 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_exti.h"




 

 



 




 
 
HAL_StatusTypeDef HAL_EXTI_SetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_GetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_ClearConfigLine(EXTI_HandleTypeDef *hexti);
HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti, EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void));
HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti, uint32_t ExtiLine);


 




 
 
void HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti);
uint32_t HAL_EXTI_GetPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_ClearPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_GenerateSWI(EXTI_HandleTypeDef *hexti);



 



 



 



 







 
#line 281 "../Core/Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"



 



  

 




 
   


 
typedef struct
{
  uint32_t Channel;              
 

  uint32_t Direction;            

 

  uint32_t PeriphInc;            
 

  uint32_t MemInc;               
 

  uint32_t PeriphDataAlignment;  
 

  uint32_t MemDataAlignment;     
 

  uint32_t Mode;                 


 

  uint32_t Priority;             
 

  uint32_t FIFOMode;             


 

  uint32_t FIFOThreshold;        
 

  uint32_t MemBurst;             



 

  uint32_t PeriphBurst;          



 
}DMA_InitTypeDef;




 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
  HAL_DMA_STATE_ERROR             = 0x04U,   
  HAL_DMA_STATE_ABORT             = 0x05U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER           = 0x00U,   
  HAL_DMA_HALF_TRANSFER           = 0x01U    
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID         = 0x00U,   
  HAL_DMA_XFER_HALFCPLT_CB_ID     = 0x01U,   
  HAL_DMA_XFER_M1CPLT_CB_ID       = 0x02U,   
  HAL_DMA_XFER_M1HALFCPLT_CB_ID   = 0x03U,   
  HAL_DMA_XFER_ERROR_CB_ID        = 0x04U,   
  HAL_DMA_XFER_ABORT_CB_ID        = 0x05U,   
  HAL_DMA_XFER_ALL_CB_ID          = 0x06U    
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                         

  DMA_InitTypeDef            Init;                                                               

  HAL_LockTypeDef            Lock;                                                                

  volatile HAL_DMA_StateTypeDef  State;                                                             

  void                       *Parent;                                                            

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);          

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);      

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);        
  
  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);         
  
  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);           

  volatile uint32_t              ErrorCode;                                                         
  
  uint32_t                   StreamBaseAddress;                                                 

  uint32_t                   StreamIndex;                                                       
 
}DMA_HandleTypeDef;



 

 




 




  
#line 194 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 




  
#line 220 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 




  





 
        



  




  




  




 




  





  




 





 




  





 




 






  




 




  




 






  




  






  




  






 




 







 




  
#line 383 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 



 
 
 




 













 






 






 


 





 
#line 448 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





       
#line 468 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
#line 488 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
#line 508 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
#line 528 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h"



 



  

 



 
   


  
typedef enum
{
  MEMORY0      = 0x00U,     
  MEMORY1      = 0x01U      
}HAL_DMA_MemoryTypeDef;



 

 



 




 

 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 


 
         
 



 


 



 



 







 
#line 641 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

 




 




 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma); 
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);


 




 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_CleanCallbacks(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



  




 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


  


  
 



 


  

 



 
#line 730 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

















































  

 



 


 



  



 







 
#line 285 "../Core/Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"



 



  
 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
          
  uint8_t                TypeExtField;          
                  
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
#line 100 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 



 





 




 







 



 




 



 




 



 




 



 




 



 




 



 





 



 
#line 213 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 
   


 
#line 226 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 



 
#line 241 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 




 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

 
 
 
 


 



































#line 347 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

#line 356 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

#line 385 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"






 

 



  



 
  





 

 
#line 289 "../Core/Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"



 



  

 


 













 
typedef struct
{
  uint32_t ClockPrescaler;               

 
  uint32_t Resolution;                   
 
  uint32_t DataAlign;                    

 
  uint32_t ScanConvMode;                 





 
  uint32_t EOCSelection;                 





 
  FunctionalState ContinuousConvMode;    

 
  uint32_t NbrOfConversion;              

 
  FunctionalState DiscontinuousConvMode; 


 
  uint32_t NbrOfDiscConversion;          

 
  uint32_t ExternalTrigConv;             


 
  uint32_t ExternalTrigConvEdge;         

 
  FunctionalState DMAContinuousRequests; 



 
}ADC_InitTypeDef;







  
typedef struct 
{
  uint32_t Channel;                
 
  uint32_t Rank;                   
 
  uint32_t SamplingTime;           







 
  uint32_t Offset;                  
}ADC_ChannelConfTypeDef;



  
typedef struct
{
  uint32_t WatchdogMode;      
 
  uint32_t HighThreshold;     
      
  uint32_t LowThreshold;      
 
  uint32_t Channel;           

       
  FunctionalState ITMode;     

 
  uint32_t WatchdogNumber;     
}ADC_AnalogWDGConfTypeDef;



  
 





 




 





 




 




 





  



typedef struct

{
  ADC_TypeDef                   *Instance;                    

  ADC_InitTypeDef               Init;                         

  volatile uint32_t                 NbrOfCurrentConversionRank;   

  DMA_HandleTypeDef             *DMA_Handle;                  

  HAL_LockTypeDef               Lock;                         

  volatile uint32_t                 State;                        

  volatile uint32_t                 ErrorCode;                    
#line 218 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"
}ADC_HandleTypeDef;

#line 241 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"



 

 


 



 
#line 262 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"


 




  






  



  
#line 297 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"


  



  






  



  






  



 
 
 
#line 345 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"


  



  




  



  
#line 380 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"





  



  
#line 398 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"


  

  

  





  



  




 



  
#line 431 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"


  
    


  






  
    


  
#line 455 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"


  



  





 



  

 


 




 
#line 493 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"





 






 







 







 






 







 







 




 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"



 



  

 


 
   











 
typedef struct 
{
  uint32_t InjectedChannel;                      

 
  uint32_t InjectedRank;                         

 
  uint32_t InjectedSamplingTime;                 







 
  uint32_t InjectedOffset;                       


 
  uint32_t InjectedNbrOfConversion;              



 
  FunctionalState InjectedDiscontinuousConvMode; 





 
  FunctionalState AutoInjectedConv;              






 
  uint32_t ExternalTrigInjecConv;                






 
  uint32_t ExternalTrigInjecConvEdge;            



 
}ADC_InjectionConfTypeDef; 



  
typedef struct
{
  uint32_t Mode;              
 
  uint32_t DMAAccessMode;     
 
  uint32_t TwoSamplingDelay;  
 
}ADC_MultiModeTypeDef;



 

 


 



  
#line 150 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"


  



  






  



  






  



  
#line 196 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"


  



  






 



 
#line 221 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"








  




  

 


 
#line 254 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"


  

 


 



 

 
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef* hadc, uint32_t InjectedRank);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef* hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef* hadc, ADC_MultiModeTypeDef* multimode);



  



 
 
 
 


 



 

 


 
#line 312 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"







#line 359 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc_ex.h"







 







 







 

 


 



 



  



 








 
#line 553 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"

 


 



 
 
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);








 



 
 
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

void HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);


 



 
 
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);


 



 
 
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc);


 



 
 
 
 


 
 
 
 

 
 
 



 

 



 

 





 









 








 









 






 



    
#line 770 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"

#line 779 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_adc.h"





 







 







 







 







 







 






 






 






 






 






 






 




 

 


 



 



  



 








 
#line 293 "../Core/Inc/stm32f4xx_hal_conf.h"






















#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"

















 

 












 
#line 35 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"



 



 

 


 



 
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,   
  HAL_DAC_STATE_READY             = 0x01U,   
  HAL_DAC_STATE_BUSY              = 0x02U,   
  HAL_DAC_STATE_TIMEOUT           = 0x03U,   
  HAL_DAC_STATE_ERROR             = 0x04U    
}HAL_DAC_StateTypeDef;
 


 



typedef struct

{
  DAC_TypeDef                 *Instance;      

  volatile HAL_DAC_StateTypeDef   State;          

  HAL_LockTypeDef             Lock;           

  DMA_HandleTypeDef           *DMA_Handle1;   

  DMA_HandleTypeDef           *DMA_Handle2;   

  volatile uint32_t               ErrorCode;      

#line 95 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"

}DAC_HandleTypeDef;



 
typedef struct
{
  uint32_t DAC_Trigger;       
 

  uint32_t DAC_OutputBuffer;  
 
}DAC_ChannelConfTypeDef;

#line 134 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"


 

 


 



 
#line 153 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"


 



 

#line 169 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"





 



 




 



 




 



 





 



  




 



  




 



 

 


 




 
#line 244 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"





 






 






 






 









 









 









 



 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac_ex.h"

















 

 












 
#line 35 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac_ex.h"



 



 

 
 


 



 
#line 77 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac_ex.h"


 



 

 
 


 



 
 
uint32_t HAL_DACEx_DualGetValue(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef* hdac, uint32_t Alignment, uint32_t Data1, uint32_t Data2);

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef* hdac);


 



 
 
 
 


 



 

 


 
#line 150 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac_ex.h"


 

 


 
void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *hdma); 


 







 



 
  






 
#line 308 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"

 


 



 
 
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);


 



 
 
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef* hdac, uint32_t Channel);


 



 
 
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef* hdac, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);


 



 
 
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef* hdac);
void HAL_DAC_IRQHandler(DAC_HandleTypeDef* hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);








 



 
 
 
 


 



 

 


 
#line 397 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"

#line 407 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dac.h"




 





 





 




 

 


 


 







 



 
  






 
#line 317 "../Core/Inc/stm32f4xx_hal_conf.h"










#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"



 



  

 


 
 


 
typedef enum 
{
  FLASH_PROC_NONE = 0U, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;



 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;    
  
  volatile uint32_t               NbSectorsToErase;    
  
  volatile uint8_t                VoltageForErase;     
  
  volatile uint32_t               Sector;              
  
  volatile uint32_t               Bank;                
  
  volatile uint32_t               Address;             
  
  HAL_LockTypeDef             Lock;                

  volatile uint32_t               ErrorCode;           

}FLASH_ProcessTypeDef;



 

 


   



  
#line 97 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"


 
  


  






 




  
#line 126 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"


 
  



  




   



 







  



  







  



  
  
 


 





  






  





  





  





  





  





  





  






 








 










   









   
















 















 



 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 

  uint32_t Banks;       
 

  uint32_t Sector;      
 

  uint32_t NbSectors;   
 

  uint32_t VoltageRange;
 

} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;   
 

  uint32_t WRPState;     
 

  uint32_t WRPSector;         
 

  uint32_t Banks;        
         

  uint32_t RDPLevel;     
 

  uint32_t BORLevel;     
 

  uint8_t  USERConfig;    

} FLASH_OBProgramInitTypeDef;



 





typedef struct
{
  uint32_t OptionType;     
 

  uint32_t PCROPState;     
 



  uint16_t Sectors;        
 



#line 130 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
}FLASH_AdvOBProgramInitTypeDef;




 

 



 



  




 
  


  






 
  


  




 
  


  






 
  


 






  
  


  




  
  


  




  




  




     



   






 








  




 






  






#line 266 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



 
   
#line 293 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  




     
#line 311 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



  
  



 
#line 327 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 336 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


  
    


 





#line 356 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


  



 
    
#line 391 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

    
#line 412 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
       

  
#line 430 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
#line 441 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
#line 451 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 464 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



  



 
   
#line 502 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
#line 524 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
     
      
  
#line 543 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 555 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
 
 
#line 566 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 580 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 


 
  


 
    
#line 617 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
      
 
#line 639 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
       

 
#line 651 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 662 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 677 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



 
  


 







 



 
#line 708 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



  
  
 

 


 



 
 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);






HAL_StatusTypeDef HAL_FLASHEx_AdvOBProgram (FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
void              HAL_FLASHEx_AdvOBGetConfig(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
HAL_StatusTypeDef HAL_FLASHEx_OB_SelectPCROP(void);
HAL_StatusTypeDef HAL_FLASHEx_OB_DeSelectPCROP(void);










 



 
 
 
 


 
  




 




  





  




  




 






  






 

 


 



 



























#line 849 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







#line 863 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
  
#line 883 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 898 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







#line 913 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
#line 928 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 939 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 949 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"













#line 968 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"





  
























   


























#line 1034 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



 

 


 
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);


  



  



 







 
#line 298 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h"

















  

 



#line 75 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h"




 
#line 299 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"

 


 


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void HAL_FLASH_IRQHandler(void);
  
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
 
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);


 



 
 
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);


 



  
 
 


 



 
 


 



  



  



  



  



  




 

 


 



 






 



 

 


 



 



  



 







 
#line 329 "../Core/Inc/stm32f4xx_hal_conf.h"


























#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"



 



 

 


 




 
typedef struct
{
  uint32_t ClockSpeed;       
 

  uint32_t DutyCycle;        
 

  uint32_t OwnAddress1;      
 

  uint32_t AddressingMode;   
 

  uint32_t DualAddressMode;  
 

  uint32_t OwnAddress2;      
 

  uint32_t GeneralCallMode;  
 

  uint32_t NoStretchMode;    
 

} I2C_InitTypeDef;



 



























 
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,    
  HAL_I2C_STATE_READY             = 0x20U,    
  HAL_I2C_STATE_BUSY              = 0x24U,    
  HAL_I2C_STATE_BUSY_TX           = 0x21U,    
  HAL_I2C_STATE_BUSY_RX           = 0x22U,    
  HAL_I2C_STATE_LISTEN            = 0x28U,    
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   
 
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   
 
  HAL_I2C_STATE_ABORT             = 0x60U,    
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,    
  HAL_I2C_STATE_ERROR             = 0xE0U     

} HAL_I2C_StateTypeDef;



 


















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

} HAL_I2C_ModeTypeDef;



 




 
#line 177 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"


 




 



typedef struct

{
  I2C_TypeDef                *Instance;       

  I2C_InitTypeDef            Init;            

  uint8_t                    *pBuffPtr;       

  uint16_t                   XferSize;        

  volatile uint16_t              XferCount;       

  volatile uint32_t              XferOptions;     

  volatile uint32_t              PreviousState;  
 

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_I2C_StateTypeDef  State;           

  volatile HAL_I2C_ModeTypeDef   Mode;            

  volatile uint32_t              ErrorCode;       

  volatile uint32_t              Devaddress;      

  volatile uint32_t              Memaddress;      

  volatile uint32_t              MemaddSize;      

  volatile uint32_t              EventCount;      


#line 244 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"
} I2C_HandleTypeDef;

#line 274 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"


 



 
 



 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 
#line 359 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"



 




 






 





 



 

#line 402 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"


 



 

 



 




 
#line 429 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"









 











 
























 













 






 
#line 504 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"




 
#line 516 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"




 





 




 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c_ex.h"

















 

 








 
#line 31 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c_ex.h"



 



 

 
 


 



 




 



 

 
 


 



 
 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);


 



 
 
 
 


 



 

 


 





 



 



 










 
#line 535 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

 
#line 558 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"


 



 
 
 
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);


 



 
 
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);


 



 
 
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);



 



 
 
 
 


 





 

 


 

#line 669 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"













 
#line 705 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_i2c.h"








 



 

 


 



 



 



 








 
#line 357 "../Core/Inc/stm32f4xx_hal_conf.h"


















#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;



 

 


 
  


 



 



  
#line 86 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


    
 


 
#line 100 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


 




 




 
    


 




 



 




 



 







 



  
  
 


 





















 







 





 





 





 





 





 





 





 






 






 








 







 





 





 




 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

















  

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 



  

  
 


 
#line 66 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 
#line 80 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 




 
#line 94 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


    





  
  
 


 

#line 127 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"









 
#line 145 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

#line 193 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 

 


 
 


 
void HAL_PWREx_EnableFlashPowerDown(void);
void HAL_PWREx_DisableFlashPowerDown(void); 
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void); 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);




void HAL_PWREx_EnableMainRegulatorLowVoltage(void);
void HAL_PWREx_DisableMainRegulatorLowVoltage(void);
void HAL_PWREx_EnableLowRegulatorLowVoltage(void);
void HAL_PWREx_DisableLowRegulatorLowVoltage(void);



#line 228 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 



 
 
 
 


 



 
 
 
 



 



 


    
 



 



 

 



   
 
 





 



 

 


 



 






#line 310 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

#line 321 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 



 



  



 
  







 
#line 275 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"

 


 
  


 
 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);


 



 
 
 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

 
void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);

 
void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);


 



 

 
 
 


 



 



 



 
 







 



 
 
 



 



 




 



 
 
 




 



 
 


 



 
#line 408 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


 



 



  



 
  







 
#line 377 "../Core/Inc/stm32f4xx_hal_conf.h"


























#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  



 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 
 

  uint32_t OverSampling;              
 
} UART_InitTypeDef;







































 
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    
 
  HAL_UART_STATE_READY             = 0x20U,    
 
  HAL_UART_STATE_BUSY              = 0x24U,    
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,    
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,    
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_UART_STATE_ERROR             = 0xE0U     
 
} HAL_UART_StateTypeDef;



 
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;         

  UART_InitTypeDef              Init;              

  uint8_t                       *pTxBuffPtr;       

  uint16_t                      TxXferSize;        

  volatile uint16_t                 TxXferCount;       

  uint8_t                       *pRxBuffPtr;       

  uint16_t                      RxXferSize;        

  volatile uint16_t                 RxXferCount;       

  DMA_HandleTypeDef             *hdmatx;           

  DMA_HandleTypeDef             *hdmarx;           

  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    gState;           

 

  volatile HAL_UART_StateTypeDef    RxState;          
 

  volatile uint32_t                 ErrorCode;         

#line 188 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"

} UART_HandleTypeDef;

#line 218 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"



 

 


 



 
#line 240 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"


 



 




 



 




 



 





 



 






 



 





 



 




 



 




 



 




 



 




 





 
#line 344 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"


 









 













 



 

 


 






 
#line 400 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"





 



















 























 







 
#line 465 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"






 







 







 







 

















 



















 


















 
















 



















 



















 



















 









 





 





 





 



 

 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

 







 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);



 



 
 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 
 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);


 



 
 
 
 


 


 







 

 


 
#line 800 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"





 








 






 

 


 



 



 



 







 
#line 405 "../Core/Inc/stm32f4xx_hal_conf.h"






















#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hcd.h"

















 

 







 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_usb.h"

















 

 







 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_usb.h"




 



 

 



 


typedef enum
{
  USB_DEVICE_MODE  = 0,
  USB_HOST_MODE    = 1,
  USB_DRD_MODE     = 2
} USB_OTG_ModeTypeDef;



 
typedef enum
{
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
} USB_OTG_URBStateTypeDef;



 
typedef enum
{
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,
  HC_BBLERR,
  HC_DATATGLERR
} USB_OTG_HCStateTypeDef;



 
typedef struct
{
  uint32_t dev_endpoints;           

 

  uint32_t Host_channels;           

 

  uint32_t speed;                   
 

  uint32_t dma_enable;               

  uint32_t ep0_mps;                  

  uint32_t phy_itface;              
 

  uint32_t Sof_enable;               

  uint32_t low_power_enable;         

  uint32_t lpm_enable;               

  uint32_t battery_charging_enable;  

  uint32_t vbus_sensing_enable;      

  uint32_t use_dedicated_ep1;        

  uint32_t use_external_vbus;        

} USB_OTG_CfgTypeDef;

typedef struct
{
  uint8_t   num;                  
 

  uint8_t   is_in;                
 

  uint8_t   is_stall;             
 

  uint8_t   type;                 
 

  uint8_t   data_pid_start;       
 

  uint8_t   even_odd_frame;       
 

  uint16_t  tx_fifo_num;          
 

  uint32_t  maxpacket;            
 

  uint8_t   *xfer_buff;            

  uint32_t  dma_addr;              

  uint32_t  xfer_len;              

  uint32_t  xfer_count;            
} USB_OTG_EPTypeDef;

typedef struct
{
  uint8_t   dev_addr;           
 

  uint8_t   ch_num;             
 

  uint8_t   ep_num;             
 

  uint8_t   ep_is_in;           
 

  uint8_t   speed;              
 

  uint8_t   do_ping;             

  uint8_t   process_ping;        

  uint8_t   ep_type;            
 

  uint16_t  max_packet;         
 

  uint8_t   data_pid;           
 

  uint8_t   *xfer_buff;          

  uint32_t  xfer_len;            

  uint32_t  xfer_count;          

  uint8_t   toggle_in;          
 

  uint8_t   toggle_out;         
 

  uint32_t  dma_addr;            

  uint32_t  ErrCnt;              

  USB_OTG_URBStateTypeDef urb_state;  
 

  USB_OTG_HCStateTypeDef state;       
 
} USB_OTG_HCTypeDef;



 



 




 




 



 





 



 







 



 





 



 




 



 
#line 279 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_usb.h"


 



 





 



 





 



 






 



 






 



 





 



 







 



 







 



 





 



 





 































 

 


 









 

 


 

HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef *USBx, uint32_t hclk, uint8_t speed);
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_OTG_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx, uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num);
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src,
                                  uint8_t ch_ep_num, uint16_t len, uint8_t dma);

void             *USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress(USB_OTG_GlobalTypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateSetup(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t dma, uint8_t *psetup);
uint8_t           USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetMode(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadInterrupts(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);
void              USB_ClearInterrupts(USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_HostInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx, uint8_t freq);
HAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DriveVbus(USB_OTG_GlobalTypeDef *USBx, uint8_t state);
uint32_t          USB_GetHostSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetCurrentFrame(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num,
                              uint8_t epnum, uint8_t dev_address, uint8_t speed,
                              uint8_t ep_type, uint16_t mps);
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx,
                                   USB_OTG_HCTypeDef *hc, uint8_t dma);

uint32_t          USB_HC_ReadInterrupt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx, uint8_t hc_num);
HAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num);
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx);




 



 



 



 









 
#line 30 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hcd.h"




 



 

 


 



 
typedef enum
{
  HAL_HCD_STATE_RESET    = 0x00,
  HAL_HCD_STATE_READY    = 0x01,
  HAL_HCD_STATE_ERROR    = 0x02,
  HAL_HCD_STATE_BUSY     = 0x03,
  HAL_HCD_STATE_TIMEOUT  = 0x04
} HCD_StateTypeDef;

typedef USB_OTG_GlobalTypeDef   HCD_TypeDef;
typedef USB_OTG_CfgTypeDef      HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef       HCD_HCTypeDef;
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef;
typedef USB_OTG_HCStateTypeDef  HCD_HCStateTypeDef;


 



 



typedef struct

{
  HCD_TypeDef               *Instance;   
  HCD_InitTypeDef           Init;        
  HCD_HCTypeDef             hc[16];      
  HAL_LockTypeDef           Lock;        
  volatile HCD_StateTypeDef     State;       
  volatile  uint32_t            ErrorCode;   
  void                      *pData;      
#line 94 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hcd.h"
} HCD_HandleTypeDef;


 



 

 


 



 






 



 




 




 






 



 

 



 














 

 


 



 
HAL_StatusTypeDef HAL_HCD_Init(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_DeInit(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                  uint8_t epnum, uint8_t dev_address,
                                  uint8_t speed, uint8_t ep_type, uint16_t mps);

HAL_StatusTypeDef HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);
void              HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void              HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);

#line 229 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_hcd.h"


 

 


 
HAL_StatusTypeDef HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                           uint8_t direction, uint8_t ep_type,
                                           uint8_t token, uint8_t *pbuff,
                                           uint16_t length, uint8_t do_ping);

 
void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void HAL_HCD_WKUP_IRQHandler(HCD_HandleTypeDef *hhcd);
void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
                                         HCD_URBStateTypeDef urb_state);


 

 


 
HAL_StatusTypeDef HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_Start(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_Stop(HCD_HandleTypeDef *hhcd);


 


 

 


 
HCD_StateTypeDef        HAL_HCD_GetState(HCD_HandleTypeDef *hhcd);
HCD_URBStateTypeDef     HAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
HCD_HCStateTypeDef      HAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t                HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);



 



 

 


 


 
 


 



 

 


 



 








 
#line 429 "../Core/Inc/stm32f4xx_hal_conf.h"


































 
#line 479 "../Core/Inc/stm32f4xx_hal_conf.h"







 
#line 31 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"



 



  

 
 



 



 
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


 



 
   
 


 


 
#line 94 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

#line 117 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


 



 





 




#line 141 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

#line 156 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"




 



 





 



 



 





 



 



 





 

 



 
extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;


 

 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);


 



 
 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);







 



 
 
 


 


 
 


 


 
 
 


 



  
  






 
#line 5 "..\\Bootloader\\bootLoader.h"
 
#line 7 "..\\Bootloader\\bootLoader.h"
 
#line 1 "../USB_HOST/Target/usbh_platform.h"
 
















 
 

 







 
#line 1 "../USB_HOST/App/usb_host.h"
 

















 
 

 







 
#line 32 "../USB_HOST/App/usb_host.h"
#line 33 "../USB_HOST/App/usb_host.h"

 

 



 




 




 



 

 
typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_START,
  APPLICATION_READY,
  APPLICATION_DISCONNECT
}ApplicationTypeDef;




 

 

 
void MX_USB_HOST_Init(void);

void MX_USB_HOST_Process(void);



 



 



 







 
#line 31 "../USB_HOST/Target/usbh_platform.h"

 

 

void MX_DriverVbusFS(uint8_t state);







 
#line 9 "..\\Bootloader\\bootLoader.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"

















 
 







 
#line 1 "../USB_HOST/Target/usbh_conf.h"
 

















 
 

 





 

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 31 "../USB_HOST/Target/usbh_conf.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 32 "../USB_HOST/Target/usbh_conf.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 33 "../USB_HOST/Target/usbh_conf.h"
#line 1 "../Core/Inc/main.h"
 

















 
 

 







 
#line 32 "../Core/Inc/main.h"
#line 33 "../Core/Inc/main.h"

 
 

 

 
 

 

 
 

 

 
 

 

 
void Error_Handler(void);

 

 

 
#line 152 "../Core/Inc/main.h"
 

 







 
#line 34 "../USB_HOST/Target/usbh_conf.h"

#line 36 "../USB_HOST/Target/usbh_conf.h"
#line 37 "../USB_HOST/Target/usbh_conf.h"

 

 



 




 




 



 




 

 


 


 


 


 


 


 


 


 


 
 











 




 

 

 


 


 


 


 

#line 136 "../USB_HOST/Target/usbh_conf.h"



#line 147 "../USB_HOST/Target/usbh_conf.h"

#line 157 "../USB_HOST/Target/usbh_conf.h"



 




 



 




 

 



 



 



 







 
#line 29 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"



 



 




 

































#line 84 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"








#line 99 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"

 




 





 





 
 
#line 130 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"

 
#line 142 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"








 
 
#line 162 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"



























 











typedef union
{
  uint16_t w;
  struct BW
  {
    uint8_t msb;
    uint8_t lsb;
  }
  bw;
}
uint16_t_uint8_t;


typedef union _USB_Setup
{
  uint32_t d8[2];

  struct _SetupPkt_Struc
  {
    uint8_t           bmRequestType;
    uint8_t           bRequest;
    uint16_t_uint8_t  wValue;
    uint16_t_uint8_t  wIndex;
    uint16_t_uint8_t  wLength;
  } b;
}
USB_Setup_TypeDef;

typedef  struct  _DescHeader
{
  uint8_t  bLength;
  uint8_t  bDescriptorType;
}
USBH_DescHeader_t;

typedef struct _DeviceDescriptor
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdUSB;         
  uint8_t   bDeviceClass;
  uint8_t   bDeviceSubClass;
  uint8_t   bDeviceProtocol;
  

 
  uint8_t   bMaxPacketSize;
  uint16_t  idVendor;       
  uint16_t  idProduct;      
  uint16_t  bcdDevice;      
  uint8_t   iManufacturer;   
  uint8_t   iProduct;        
  uint8_t   iSerialNumber;   
  uint8_t   bNumConfigurations;  
}
USBH_DevDescTypeDef;

typedef struct _EndpointDescriptor
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bEndpointAddress;    
  uint8_t   bmAttributes;        
  uint16_t  wMaxPacketSize;     
  uint8_t   bInterval;           
}
USBH_EpDescTypeDef;

typedef struct _InterfaceDescriptor
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;     
  uint8_t bNumEndpoints;         
  uint8_t bInterfaceClass;       
  uint8_t bInterfaceSubClass;    
  uint8_t bInterfaceProtocol;    
  uint8_t iInterface;            
  USBH_EpDescTypeDef               Ep_Desc[2U];
}
USBH_InterfaceDescTypeDef;


typedef struct _ConfigurationDescriptor
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;         
  uint8_t   bNumInterfaces;        
  uint8_t   bConfigurationValue;   
  uint8_t   iConfiguration;        
  uint8_t   bmAttributes;          
  uint8_t   bMaxPower;             
  USBH_InterfaceDescTypeDef        Itf_Desc[2U];
}
USBH_CfgDescTypeDef;


 
typedef enum
{
  USBH_OK = 0,
  USBH_BUSY,
  USBH_FAIL,
  USBH_NOT_SUPPORTED,
  USBH_UNRECOVERED_ERROR,
  USBH_ERROR_SPEED_UNKNOWN,
} USBH_StatusTypeDef;




 

typedef enum
{
  USBH_SPEED_HIGH  = 0U,
  USBH_SPEED_FULL  = 1U,
  USBH_SPEED_LOW   = 2U,

} USBH_SpeedTypeDef;

 
typedef enum
{
  HOST_IDLE = 0U,
  HOST_DEV_WAIT_FOR_ATTACHMENT,
  HOST_DEV_ATTACHED,
  HOST_DEV_DISCONNECTED,
  HOST_DETECT_DEVICE_SPEED,
  HOST_ENUMERATION,
  HOST_CLASS_REQUEST,
  HOST_INPUT,
  HOST_SET_CONFIGURATION,
  HOST_SET_WAKEUP_FEATURE,
  HOST_CHECK_CLASS,
  HOST_CLASS,
  HOST_SUSPENDED,
  HOST_ABORT_STATE,
} HOST_StateTypeDef;

 
typedef enum
{
  ENUM_IDLE = 0U,
  ENUM_GET_FULL_DEV_DESC,
  ENUM_SET_ADDR,
  ENUM_GET_CFG_DESC,
  ENUM_GET_FULL_CFG_DESC,
  ENUM_GET_MFC_STRING_DESC,
  ENUM_GET_PRODUCT_STRING_DESC,
  ENUM_GET_SERIALNUM_STRING_DESC,
} ENUM_StateTypeDef;

 
typedef enum
{
  CTRL_IDLE = 0U,
  CTRL_SETUP,
  CTRL_SETUP_WAIT,
  CTRL_DATA_IN,
  CTRL_DATA_IN_WAIT,
  CTRL_DATA_OUT,
  CTRL_DATA_OUT_WAIT,
  CTRL_STATUS_IN,
  CTRL_STATUS_IN_WAIT,
  CTRL_STATUS_OUT,
  CTRL_STATUS_OUT_WAIT,
  CTRL_ERROR,
  CTRL_STALLED,
  CTRL_COMPLETE
} CTRL_StateTypeDef;


 
typedef enum
{
  CMD_IDLE = 0U,
  CMD_SEND,
  CMD_WAIT
} CMD_StateTypeDef;

typedef enum
{
  USBH_URB_IDLE = 0U,
  USBH_URB_DONE,
  USBH_URB_NOTREADY,
  USBH_URB_NYET,
  USBH_URB_ERROR,
  USBH_URB_STALL
} USBH_URBStateTypeDef;

typedef enum
{
  USBH_PORT_EVENT = 1U,
  USBH_URB_EVENT,
  USBH_CONTROL_EVENT,
  USBH_CLASS_EVENT,
  USBH_STATE_CHANGED_EVENT,
}
USBH_OSEventTypeDef;

 
typedef struct
{
  uint8_t               pipe_in;
  uint8_t               pipe_out;
  uint8_t               pipe_size;
  uint8_t               *buff;
  uint16_t              length;
  uint16_t              timer;
  USB_Setup_TypeDef     setup;
  CTRL_StateTypeDef     state;
  uint8_t               errorcount;

} USBH_CtrlTypeDef;

 
typedef struct
{
  uint8_t                           CfgDesc_Raw[256U];
  uint8_t                           Data[512U];
  uint8_t                           address;
  uint8_t                           speed;
  uint8_t                           EnumCnt;
  uint8_t                           RstCnt;
  volatile uint8_t                      is_connected;
  volatile uint8_t                      is_disconnected;
  volatile uint8_t                      is_ReEnumerated;
  uint8_t                           PortEnabled;
  uint8_t                           current_interface;
  USBH_DevDescTypeDef               DevDesc;
  USBH_CfgDescTypeDef               CfgDesc;
} USBH_DeviceTypeDef;

struct _USBH_HandleTypeDef;

 
typedef struct
{
  const char          *Name;
  uint8_t              ClassCode;
  USBH_StatusTypeDef(*Init)(struct _USBH_HandleTypeDef *phost);
  USBH_StatusTypeDef(*DeInit)(struct _USBH_HandleTypeDef *phost);
  USBH_StatusTypeDef(*Requests)(struct _USBH_HandleTypeDef *phost);
  USBH_StatusTypeDef(*BgndProcess)(struct _USBH_HandleTypeDef *phost);
  USBH_StatusTypeDef(*SOFProcess)(struct _USBH_HandleTypeDef *phost);
  void                *pData;
} USBH_ClassTypeDef;

 
typedef struct _USBH_HandleTypeDef
{
  volatile HOST_StateTypeDef     gState;        
  ENUM_StateTypeDef     EnumState;     
  CMD_StateTypeDef      RequestState;
  USBH_CtrlTypeDef      Control;
  USBH_DeviceTypeDef    device;
  USBH_ClassTypeDef    *pClass[1U];
  USBH_ClassTypeDef    *pActiveClass;
  uint32_t              ClassNumber;
  uint32_t              Pipes[16];
  volatile uint32_t         Timer;
  uint32_t              Timeout;
  uint8_t               id;
  void                 *pData;
  void (* pUser)(struct _USBH_HandleTypeDef *pHandle, uint8_t id);

#line 480 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"

} USBH_HandleTypeDef;


#line 492 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_def.h"







 

#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_ioreq.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_ioreq.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"

















 
 
#line 177 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"


 



 



 

 



#line 31 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_ioreq.h"



 



 




 




 












 




 


 




 


 



 


 



 
USBH_StatusTypeDef USBH_CtlSendSetup(USBH_HandleTypeDef *phost,
                                     uint8_t *buff,
                                     uint8_t pipe_num);

USBH_StatusTypeDef USBH_CtlSendData(USBH_HandleTypeDef *phost,
                                    uint8_t *buff,
                                    uint16_t length,
                                    uint8_t pipe_num,
                                    uint8_t do_ping);

USBH_StatusTypeDef USBH_CtlReceiveData(USBH_HandleTypeDef *phost,
                                       uint8_t *buff,
                                       uint16_t length,
                                       uint8_t pipe_num);

USBH_StatusTypeDef USBH_BulkReceiveData(USBH_HandleTypeDef *phost,
                                        uint8_t *buff,
                                        uint16_t length,
                                        uint8_t pipe_num);

USBH_StatusTypeDef USBH_BulkSendData(USBH_HandleTypeDef *phost,
                                     uint8_t *buff,
                                     uint16_t length,
                                     uint8_t pipe_num,
                                     uint8_t do_ping);

USBH_StatusTypeDef USBH_InterruptReceiveData(USBH_HandleTypeDef *phost,
                                             uint8_t             *buff,
                                             uint8_t             length,
                                             uint8_t             pipe_num);

USBH_StatusTypeDef USBH_InterruptSendData(USBH_HandleTypeDef *phost,
                                          uint8_t *buff,
                                          uint8_t length,
                                          uint8_t pipe_num);


USBH_StatusTypeDef USBH_IsocReceiveData(USBH_HandleTypeDef *phost,
                                        uint8_t *buff,
                                        uint32_t length,
                                        uint8_t pipe_num);


USBH_StatusTypeDef USBH_IsocSendData(USBH_HandleTypeDef *phost,
                                     uint8_t *buff,
                                     uint32_t length,
                                     uint8_t pipe_num);


 









 



 



 

 


#line 31 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_pipes.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_pipes.h"



 



 




 



 


 



 


 




 


 



 


 



 

USBH_StatusTypeDef USBH_OpenPipe(USBH_HandleTypeDef *phost,
                                 uint8_t pipe_num,
                                 uint8_t epnum,
                                 uint8_t dev_address,
                                 uint8_t speed,
                                 uint8_t ep_type,
                                 uint16_t mps);

USBH_StatusTypeDef USBH_ClosePipe(USBH_HandleTypeDef *phost,
                                  uint8_t pipe_num);

uint8_t USBH_AllocPipe(USBH_HandleTypeDef *phost,
                       uint8_t ep_addr);

USBH_StatusTypeDef USBH_FreePipe(USBH_HandleTypeDef *phost,
                                 uint8_t idx);






 











 



 



 

 


#line 32 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_ctlreq.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_ctlreq.h"



 



 




 




 
 











 




 


 




 


 



 
extern uint8_t USBH_CfgDesc[512];


 



 
USBH_StatusTypeDef USBH_CtlReq(USBH_HandleTypeDef *phost, uint8_t *buff,
                               uint16_t length);

USBH_StatusTypeDef USBH_GetDescriptor(USBH_HandleTypeDef *phost,
                                      uint8_t  req_type, uint16_t value_idx,
                                      uint8_t *buff, uint16_t length);

USBH_StatusTypeDef USBH_Get_DevDesc(USBH_HandleTypeDef *phost, uint8_t length);

USBH_StatusTypeDef USBH_Get_StringDesc(USBH_HandleTypeDef *phost,
                                       uint8_t string_index, uint8_t *buff,
                                       uint16_t length);

USBH_StatusTypeDef USBH_SetCfg(USBH_HandleTypeDef *phost, uint16_t cfg_idx);

USBH_StatusTypeDef USBH_Get_CfgDesc(USBH_HandleTypeDef *phost, uint16_t length);

USBH_StatusTypeDef USBH_SetAddress(USBH_HandleTypeDef *phost,
                                   uint8_t DeviceAddress);

USBH_StatusTypeDef USBH_SetInterface(USBH_HandleTypeDef *phost, uint8_t ep_num,
                                     uint8_t altSetting);

USBH_StatusTypeDef USBH_SetFeature(USBH_HandleTypeDef *phost, uint8_t wValue);

USBH_StatusTypeDef USBH_ClrFeature(USBH_HandleTypeDef *phost, uint8_t ep_num);

USBH_DescHeader_t *USBH_GetNextDesc(uint8_t *pbuf, uint16_t *ptr);


 









 



 



 

 


#line 33 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"



 



 




 




 



 
#line 61 "../Middlewares/ST/STM32_USB_Host_Library/Core/Inc/usbh_core.h"




 





 



 



 



 



 


USBH_StatusTypeDef  USBH_Init(USBH_HandleTypeDef *phost, void (*pUsrFunc)(USBH_HandleTypeDef *phost, uint8_t id), uint8_t id);
USBH_StatusTypeDef  USBH_DeInit(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_RegisterClass(USBH_HandleTypeDef *phost, USBH_ClassTypeDef *pclass);
USBH_StatusTypeDef  USBH_SelectInterface(USBH_HandleTypeDef *phost, uint8_t interface);
uint8_t             USBH_FindInterface(USBH_HandleTypeDef *phost,
                                       uint8_t Class,
                                       uint8_t SubClass,
                                       uint8_t Protocol);
uint8_t             USBH_GetActiveClass(USBH_HandleTypeDef *phost);

uint8_t             USBH_FindInterfaceIndex(USBH_HandleTypeDef *phost,
                                            uint8_t interface_number,
                                            uint8_t alt_settings);

uint8_t              USBH_IsPortEnabled(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef  USBH_Start(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_Stop(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_Process(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_ReEnumerate(USBH_HandleTypeDef *phost);

 
USBH_StatusTypeDef   USBH_LL_Init(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_DeInit(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_Start(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_Stop(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef   USBH_LL_Connect(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_Disconnect(USBH_HandleTypeDef *phost);
USBH_SpeedTypeDef    USBH_LL_GetSpeed(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_ResetPort(USBH_HandleTypeDef *phost);
uint32_t             USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost,
                                             uint8_t pipe);

USBH_StatusTypeDef   USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost,
                                        uint8_t state);

USBH_StatusTypeDef   USBH_LL_OpenPipe(USBH_HandleTypeDef *phost,
                                      uint8_t pipe,
                                      uint8_t epnum,
                                      uint8_t dev_address,
                                      uint8_t speed,
                                      uint8_t ep_type,
                                      uint16_t mps);

USBH_StatusTypeDef   USBH_LL_ClosePipe(USBH_HandleTypeDef *phost,
                                       uint8_t pipe);

USBH_StatusTypeDef   USBH_LL_SubmitURB(USBH_HandleTypeDef *phost,
                                       uint8_t pipe,
                                       uint8_t direction,
                                       uint8_t ep_type,
                                       uint8_t token,
                                       uint8_t *pbuff,
                                       uint16_t length,
                                       uint8_t do_ping);

USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef *phost,
                                         uint8_t pipe);





USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef *phost,
                                     uint8_t pipe, uint8_t toggle);

uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe);

void                 USBH_LL_PortDisabled(USBH_HandleTypeDef *phost);
void                 USBH_LL_PortEnabled(USBH_HandleTypeDef *phost);

 
void USBH_LL_SetTimer(USBH_HandleTypeDef *phost, uint32_t time);
void USBH_LL_IncTimer(USBH_HandleTypeDef *phost);

void USBH_Delay(uint32_t Delay);



 








 



 



 

 



#line 10 "..\\Bootloader\\bootLoader.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_bot.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_bot.h"



 



 



 




 




 

typedef enum
{
  BOT_OK          = 0,
  BOT_FAIL        = 1,
  BOT_PHASE_ERROR = 2,
  BOT_BUSY        = 3
}
BOT_StatusTypeDef;

typedef enum
{
  BOT_CMD_IDLE  = 0,
  BOT_CMD_SEND,
  BOT_CMD_WAIT,
}
BOT_CMDStateTypeDef;

 
typedef enum
{

  BOT_CSW_CMD_PASSED   =        0x00,
  BOT_CSW_CMD_FAILED   =        0x01,
  BOT_CSW_PHASE_ERROR  =        0x02,
}
BOT_CSWStatusTypeDef;

typedef enum
{
  BOT_SEND_CBW  = 1,
  BOT_SEND_CBW_WAIT,
  BOT_DATA_IN,
  BOT_DATA_IN_WAIT,
  BOT_DATA_OUT,
  BOT_DATA_OUT_WAIT,
  BOT_RECEIVE_CSW,
  BOT_RECEIVE_CSW_WAIT,
  BOT_ERROR_IN,
  BOT_ERROR_OUT,
  BOT_UNRECOVERED_ERROR
}
BOT_StateTypeDef;

typedef union
{
  struct __CBW
  {
    uint32_t Signature;
    uint32_t Tag;
    uint32_t DataTransferLength;
    uint8_t  Flags;
    uint8_t  LUN;
    uint8_t  CBLength;
    uint8_t  CB[16];
  } field;
  uint8_t data[31];
}
BOT_CBWTypeDef;

typedef union
{
  struct __CSW
  {
    uint32_t Signature;
    uint32_t Tag;
    uint32_t DataResidue;
    uint8_t  Status;
  } field;
  uint8_t data[13];
}
BOT_CSWTypeDef;

typedef struct
{
  uint32_t                   data[16];
  BOT_StateTypeDef           state;
  BOT_StateTypeDef           prev_state;
  BOT_CMDStateTypeDef        cmd_state;
  BOT_CBWTypeDef             cbw;
  uint8_t                    Reserved1;
  BOT_CSWTypeDef             csw;
  uint8_t                    Reserved2[3];
  uint8_t                    *pbuf;
}
BOT_HandleTypeDef;



 





 




























 



 


 



 



 



 
USBH_StatusTypeDef USBH_MSC_BOT_REQ_Reset(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef USBH_MSC_BOT_REQ_GetMaxLUN(USBH_HandleTypeDef *phost, uint8_t *Maxlun);

USBH_StatusTypeDef USBH_MSC_BOT_Init(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef USBH_MSC_BOT_Process(USBH_HandleTypeDef *phost, uint8_t lun);
USBH_StatusTypeDef USBH_MSC_BOT_Error(USBH_HandleTypeDef *phost, uint8_t lun);





 










 



 



 



 
 

#line 31 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc.h"
#line 1 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_scsi.h"

















 

 







 
#line 30 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_scsi.h"




 



 



 




 


 
typedef struct
{
  uint32_t block_nbr;
  uint16_t block_size;
} SCSI_CapacityTypeDef;


 
typedef struct
{
  uint8_t key;
  uint8_t asc;
  uint8_t ascq;
} SCSI_SenseTypeDef;

 
typedef struct
{
  uint8_t PeripheralQualifier;
  uint8_t DeviceType;
  uint8_t RemovableMedia;
  uint8_t vendor_id[9];
  uint8_t product_id[17];
  uint8_t revision_id[5];
} SCSI_StdInquiryDataTypeDef;



 
#line 86 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_scsi.h"











 
#line 112 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_scsi.h"


 




 
#line 128 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc_scsi.h"


 




 






 



 


 



 



 



 
USBH_StatusTypeDef USBH_MSC_SCSI_TestUnitReady(USBH_HandleTypeDef *phost,
                                               uint8_t lun);

USBH_StatusTypeDef USBH_MSC_SCSI_ReadCapacity(USBH_HandleTypeDef *phost,
                                              uint8_t lun,
                                              SCSI_CapacityTypeDef *capacity);

USBH_StatusTypeDef USBH_MSC_SCSI_Inquiry(USBH_HandleTypeDef *phost,
                                         uint8_t lun,
                                         SCSI_StdInquiryDataTypeDef *inquiry);

USBH_StatusTypeDef USBH_MSC_SCSI_RequestSense(USBH_HandleTypeDef *phost,
                                              uint8_t lun,
                                              SCSI_SenseTypeDef *sense_data);

USBH_StatusTypeDef USBH_MSC_SCSI_Write(USBH_HandleTypeDef *phost,
                                       uint8_t lun,
                                       uint32_t address,
                                       uint8_t *pbuf,
                                       uint32_t length);

USBH_StatusTypeDef USBH_MSC_SCSI_Read(USBH_HandleTypeDef *phost,
                                      uint8_t lun,
                                      uint32_t address,
                                      uint8_t *pbuf,
                                      uint32_t length);




 










 



 



 



 

 

#line 32 "../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc/usbh_msc.h"



 



 



 




 




 

typedef enum
{
  MSC_INIT = 0,
  MSC_IDLE,
  MSC_TEST_UNIT_READY,
  MSC_READ_CAPACITY10,
  MSC_READ_INQUIRY,
  MSC_REQUEST_SENSE,
  MSC_READ,
  MSC_WRITE,
  MSC_UNRECOVERED_ERROR,
  MSC_PERIODIC_CHECK,
}
MSC_StateTypeDef;

typedef enum
{
  MSC_OK,
  MSC_NOT_READY,
  MSC_ERROR,

}
MSC_ErrorTypeDef;

typedef enum
{
  MSC_REQ_IDLE = 0,
  MSC_REQ_RESET,
  MSC_REQ_GET_MAX_LUN,
  MSC_REQ_ERROR,
}
MSC_ReqStateTypeDef;






 
typedef struct
{
  MSC_StateTypeDef            state;
  MSC_ErrorTypeDef            error;
  USBH_StatusTypeDef          prev_ready_state;
  SCSI_CapacityTypeDef        capacity;
  SCSI_SenseTypeDef           sense;
  SCSI_StdInquiryDataTypeDef  inquiry;
  uint8_t                     state_changed;

}
MSC_LUNTypeDef;

 
typedef struct _MSC_Process
{
  uint8_t              max_lun;
  uint8_t              Reserved[3];
  uint8_t              InPipe;
  uint8_t              OutPipe;
  uint8_t              OutEp;
  uint8_t              InEp;
  uint16_t             OutEpSize;
  uint16_t             InEpSize;
  MSC_StateTypeDef     state;
  MSC_ErrorTypeDef     error;
  MSC_ReqStateTypeDef  req_state;
  MSC_ReqStateTypeDef  prev_req_state;
  BOT_HandleTypeDef    hbot;
  MSC_LUNTypeDef       unit[2U];
  uint16_t             current_lun;
  uint16_t             rw_lun;
  uint32_t             timer;
}
MSC_HandleTypeDef;




 





 





 


 




 



 


 



 
extern USBH_ClassTypeDef  USBH_msc;




 



 
uint8_t USBH_MSC_IsReady(USBH_HandleTypeDef *phost);
uint8_t USBH_MSC_GetMaxLUN(USBH_HandleTypeDef *phost);
uint8_t USBH_MSC_UnitIsReady(USBH_HandleTypeDef *phost, uint8_t lun);

USBH_StatusTypeDef USBH_MSC_GetLUNInfo(USBH_HandleTypeDef *phost, uint8_t lun,
                                       MSC_LUNTypeDef *info);

USBH_StatusTypeDef USBH_MSC_Read(USBH_HandleTypeDef *phost, uint8_t lun,
                                 uint32_t address, uint8_t *pbuf, uint32_t length);

USBH_StatusTypeDef USBH_MSC_Write(USBH_HandleTypeDef *phost, uint8_t lun,
                                  uint32_t address, uint8_t *pbuf, uint32_t length);


 










 



 



 



 
 



#line 11 "..\\Bootloader\\bootLoader.h"
#line 1 "../Middlewares/Third_Party/FatFs/src/ff.h"

















 









#line 1 "../Middlewares/Third_Party/FatFs/src/integer.h"
 
 
 




#line 16 "../Middlewares/Third_Party/FatFs/src/integer.h"

 
typedef int				INT;
typedef unsigned int	UINT;

 
typedef unsigned char	BYTE;

 
typedef short			SHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;

 
typedef long			LONG;
typedef unsigned long	DWORD;

 
typedef unsigned long long QWORD;



#line 29 "../Middlewares/Third_Party/FatFs/src/ff.h"
#line 1 "../FATFS/Target/ffconf.h"
 















 
 






 
#line 27 "../FATFS/Target/ffconf.h"
#line 28 "../FATFS/Target/ffconf.h"
#line 29 "../FATFS/Target/ffconf.h"
#line 30 "../FATFS/Target/ffconf.h"
 




 





 








 







 



 


 


 


 



 



 


 



 



























 
















 




 










 







 



 


 

 






 
 







 







 




 










 



 





 




 












 










 



















 

 
#line 268 "../FATFS/Target/ffconf.h"




#line 30 "../Middlewares/Third_Party/FatFs/src/ff.h"







 

#line 46 "../Middlewares/Third_Party/FatFs/src/ff.h"



 

#line 62 "../Middlewares/Third_Party/FatFs/src/ff.h"
typedef char TCHAR;







 

#line 78 "../Middlewares/Third_Party/FatFs/src/ff.h"
typedef DWORD FSIZE_t;




 

typedef struct {
	BYTE	fs_type;		 
	BYTE	drv;			 
	BYTE	n_fats;			 
	BYTE	wflag;			 
	BYTE	fsi_flag;		 
	WORD	id;				 
	WORD	n_rootdir;		 
	WORD	csize;			 
#line 107 "../Middlewares/Third_Party/FatFs/src/ff.h"
	DWORD	last_clst;		 
	DWORD	free_clst;		 
#line 118 "../Middlewares/Third_Party/FatFs/src/ff.h"
	DWORD	n_fatent;		 
	DWORD	fsize;			 
	DWORD	volbase;		 
	DWORD	fatbase;		 
	DWORD	dirbase;		 
	DWORD	database;		 
	DWORD	winsect;		 
	BYTE	win[512];	 
} FATFS;



 

typedef struct {
	FATFS*	fs;			 
	WORD	id;			 
	BYTE	attr;		 
	BYTE	stat;		 
	DWORD	sclust;		 
	FSIZE_t	objsize;	 
#line 147 "../Middlewares/Third_Party/FatFs/src/ff.h"
	UINT	lockid;		 

} _FDID;



 

typedef struct {
	_FDID	obj;			 
	BYTE	flag;			 
	BYTE	err;			 
	FSIZE_t	fptr;			 
	DWORD	clust;			 
	DWORD	sect;			 

	DWORD	dir_sect;		 
	BYTE*	dir_ptr;		 


	DWORD*	cltbl;			 


	BYTE	buf[512];	 

} FIL;



 

typedef struct {
	_FDID	obj;			 
	DWORD	dptr;			 
	DWORD	clust;			 
	DWORD	sect;			 
	BYTE*	dir;			 
	BYTE	fn[12];			 
#line 191 "../Middlewares/Third_Party/FatFs/src/ff.h"
} DIR;



 

typedef struct {
	FSIZE_t	fsize;			 
	WORD	fdate;			 
	WORD	ftime;			 
	BYTE	fattrib;		 




	TCHAR	fname[13];		 

} FILINFO;



 

typedef enum {
	FR_OK = 0,				 
	FR_DISK_ERR,			 
	FR_INT_ERR,				 
	FR_NOT_READY,			 
	FR_NO_FILE,				 
	FR_NO_PATH,				 
	FR_INVALID_NAME,		 
	FR_DENIED,				 
	FR_EXIST,				 
	FR_INVALID_OBJECT,		 
	FR_WRITE_PROTECTED,		 
	FR_INVALID_DRIVE,		 
	FR_NOT_ENABLED,			 
	FR_NO_FILESYSTEM,		 
	FR_MKFS_ABORTED,		 
	FR_TIMEOUT,				 
	FR_LOCKED,				 
	FR_NOT_ENOUGH_CORE,		 
	FR_TOO_MANY_OPEN_FILES,	 
	FR_INVALID_PARAMETER	 
} FRESULT;



 
 

FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);				 
FRESULT f_close (FIL* fp);											 
FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);			 
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);	 
FRESULT f_lseek (FIL* fp, FSIZE_t ofs);								 
FRESULT f_truncate (FIL* fp);										 
FRESULT f_sync (FIL* fp);											 
FRESULT f_opendir (DIR* dp, const TCHAR* path);						 
FRESULT f_closedir (DIR* dp);										 
FRESULT f_readdir (DIR* dp, FILINFO* fno);							 
FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern);	 
FRESULT f_findnext (DIR* dp, FILINFO* fno);							 
FRESULT f_mkdir (const TCHAR* path);								 
FRESULT f_unlink (const TCHAR* path);								 
FRESULT f_rename (const TCHAR* path_old, const TCHAR* path_new);	 
FRESULT f_stat (const TCHAR* path, FILINFO* fno);					 
FRESULT f_chmod (const TCHAR* path, BYTE attr, BYTE mask);			 
FRESULT f_utime (const TCHAR* path, const FILINFO* fno);			 
FRESULT f_chdir (const TCHAR* path);								 
FRESULT f_chdrive (const TCHAR* path);								 
FRESULT f_getcwd (TCHAR* buff, UINT len);							 
FRESULT f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs);	 
FRESULT f_getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn);	 
FRESULT f_setlabel (const TCHAR* label);							 
FRESULT f_forward (FIL* fp, UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf);	 
FRESULT f_expand (FIL* fp, FSIZE_t szf, BYTE opt);					 
FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt);			 
FRESULT f_mkfs (const TCHAR* path, BYTE opt, DWORD au, void* work, UINT len);	 
FRESULT f_fdisk (BYTE pdrv, const DWORD* szt, void* work);			 
int f_putc (TCHAR c, FIL* fp);										 
int f_puts (const TCHAR* str, FIL* cp);								 
int f_printf (FIL* fp, const TCHAR* str, ...);						 
TCHAR* f_gets (TCHAR* buff, int len, FIL* fp);						 

#line 283 "../Middlewares/Third_Party/FatFs/src/ff.h"








 
 

 




 
#line 308 "../Middlewares/Third_Party/FatFs/src/ff.h"

 
#line 316 "../Middlewares/Third_Party/FatFs/src/ff.h"




 
 


 
#line 332 "../Middlewares/Third_Party/FatFs/src/ff.h"

 


 






 





 











#line 12 "..\\Bootloader\\bootLoader.h"
#line 1 "../Middlewares/Third_Party/FatFs/src/ff_gen_drv.h"
















 

 







 
#line 1 "../Middlewares/Third_Party/FatFs/src/diskio.h"


 











#line 16 "../Middlewares/Third_Party/FatFs/src/diskio.h"


 
typedef BYTE	DSTATUS;

 
typedef enum {
	RES_OK = 0,		 
	RES_ERROR,		 
	RES_WRPRT,		 
	RES_NOTRDY,		 
	RES_PARERR		 
} DRESULT;


 
 


DSTATUS disk_initialize (BYTE pdrv);
DSTATUS disk_status (BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);
DWORD get_fattime (void);

 






 

 






 





 






 








#line 29 "../Middlewares/Third_Party/FatFs/src/ff_gen_drv.h"
#line 30 "../Middlewares/Third_Party/FatFs/src/ff_gen_drv.h"
#line 31 "../Middlewares/Third_Party/FatFs/src/ff_gen_drv.h"


 



 
typedef struct
{
  DSTATUS (*disk_initialize) (BYTE);                      
  DSTATUS (*disk_status)     (BYTE);                      
  DRESULT (*disk_read)       (BYTE, BYTE*, DWORD, UINT);        

  DRESULT (*disk_write)      (BYTE, const BYTE*, DWORD, UINT);  


  DRESULT (*disk_ioctl)      (BYTE, BYTE, void*);               


}Diskio_drvTypeDef;



 
typedef struct
{
  uint8_t                 is_initialized[1];
  const Diskio_drvTypeDef *drv[1];
  uint8_t                 lun[1];
  volatile uint8_t        nbr;

}Disk_drvTypeDef;

 
 
 
uint8_t FATFS_LinkDriver(const Diskio_drvTypeDef *drv, char *path);
uint8_t FATFS_UnLinkDriver(char *path);
uint8_t FATFS_LinkDriverEx(const Diskio_drvTypeDef *drv, char *path, BYTE lun);
uint8_t FATFS_UnLinkDriverEx(char *path, BYTE lun);
uint8_t FATFS_GetAttachedDriversNbr(void);







 

#line 13 "..\\Bootloader\\bootLoader.h"
#line 1 "..\\Bootloader\\flash_if.h"











































 

 







 
#line 56 "..\\Bootloader\\flash_if.h"

 
typedef  void (*pFunction)(void);

#line 66 "..\\Bootloader\\flash_if.h"

 
#line 84 "..\\Bootloader\\flash_if.h"

 
 
void FLASH_If_FlashUnlock(void);
FlagStatus FLASH_If_ReadOutProtectionStatus(void);
uint32_t FLASH_If_Write(uint32_t Address, uint32_t Data);
uint32_t FLASH_If_EraseBootloader(void);
uint32_t FLASH_If_EraseApplication(void);






 
#line 14 "..\\Bootloader\\bootLoader.h"
#line 1 "../Core/Inc/usart.h"

















 
 







 
#line 29 "../Core/Inc/usart.h"

 

 

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

 

 

void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);

 

 







 
#line 15 "..\\Bootloader\\bootLoader.h"
#line 1 "..\\..\\dwLaser_Application\\MCU_Application\\deviceInfo\\deviceConfig.h"


 
#line 5 "..\\..\\dwLaser_Application\\MCU_Application\\deviceInfo\\deviceConfig.h"
#line 1 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"





 
















 











































































































































 




 








 









 



 






































































 



 



 



 


 






 



 



 


#line 314 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"




#line 1 "../Drivers/CMSIS/Include/core_cm4.h"
 




 
















 







#line 170 "../Drivers/CMSIS/Include/core_cm4.h"

#line 323 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"
#line 343 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

#line 346 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"




 





 












 






   









 






#line 61 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 75 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   




 















 
#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











 





extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double  , double  );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 230 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   
  typedef float float_t;
  typedef double double_t;
#line 251 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"



extern const int math_errhandling;
#line 261 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }


    inline float _sqrtf(float __x) { return __sqrtf(__x); }



    



 

extern __declspec(__nothrow) __attribute__((const)) double ceil(double  );
    
    
extern __declspec(__nothrow) __attribute__((const)) double fabs(double  );
    
    

extern __declspec(__nothrow) __attribute__((const)) double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __attribute__((const)) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __attribute__((const)) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 479 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __attribute__((const)) double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __attribute__((const)) float _fabsf(float);  
inline __declspec(__nothrow) __attribute__((const)) float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __attribute__((const)) float ceilf(float  );
extern __declspec(__nothrow) __attribute__((const)) float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 803 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );

inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __declspec(__nothrow) __attribute__((const)) double fmax(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __attribute__((const)) double fmin(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );

inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }


extern __declspec(__nothrow) long long llrint(double  );
extern __declspec(__nothrow) long long llrintf(float  );

inline __declspec(__nothrow) long long llrintl(long double __x)     { return llrint((double)__x); }


extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );

inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }


extern __declspec(__nothrow) long long llround(double  );
extern __declspec(__nothrow) long long llroundf(float  );

inline __declspec(__nothrow) long long llroundl(long double __x)     { return llround((double)__x); }


extern __declspec(__nothrow) __attribute__((const)) double nan(const char *  );
extern __declspec(__nothrow) __attribute__((const)) float nanf(const char *  );

inline __declspec(__nothrow) __attribute__((const)) long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 856 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) __attribute__((const)) double nearbyint(double  );
extern __declspec(__nothrow) __attribute__((const)) float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int *  );
extern  float remquof(float  , float  , int *  );

inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }


extern __declspec(__nothrow) __attribute__((const)) double round(double  );
extern __declspec(__nothrow) __attribute__((const)) float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __attribute__((const)) double trunc(double  );
extern __declspec(__nothrow) __attribute__((const)) float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 896 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 1087 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











#line 1317 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





 
#line 347 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"






  

 

#line 363 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

  

 

#line 374 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

  

 
   
   


  

 
#line 394 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

  

 

  typedef enum
  {
    ARM_MATH_SUCCESS = 0,                 
    ARM_MATH_ARGUMENT_ERROR = -1,         
    ARM_MATH_LENGTH_ERROR = -2,           
    ARM_MATH_SIZE_MISMATCH = -3,          
    ARM_MATH_NANINF = -4,                 
    ARM_MATH_SINGULAR = -5,               
    ARM_MATH_TEST_FAILURE = -6            
  } arm_status;

  

 
  typedef int8_t q7_t;

  

 
  typedef int16_t q15_t;

  

 
  typedef int32_t q31_t;

  

 
  typedef int64_t q63_t;

  

 
  typedef float float32_t;

  

 
  typedef double float64_t;

  

 





#line 481 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"






#line 497 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

   

 


#line 515 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"


  

 
  __attribute__((always_inline)) static __inline q31_t clip_q63_to_q31(
  q63_t x)
  {
    return ((q31_t) (x >> 32) != ((q31_t) x >> 31)) ?
      ((0x7FFFFFFF ^ ((q31_t) (x >> 63)))) : (q31_t) x;
  }

  

 
  __attribute__((always_inline)) static __inline q15_t clip_q63_to_q15(
  q63_t x)
  {
    return ((q31_t) (x >> 32) != ((q31_t) x >> 31)) ?
      ((0x7FFF ^ ((q15_t) (x >> 63)))) : (q15_t) (x >> 15);
  }

  

 
  __attribute__((always_inline)) static __inline q7_t clip_q31_to_q7(
  q31_t x)
  {
    return ((q31_t) (x >> 24) != ((q31_t) x >> 23)) ?
      ((0x7F ^ ((q7_t) (x >> 31)))) : (q7_t) x;
  }

  

 
  __attribute__((always_inline)) static __inline q15_t clip_q31_to_q15(
  q31_t x)
  {
    return ((q31_t) (x >> 16) != ((q31_t) x >> 15)) ?
      ((0x7FFF ^ ((q15_t) (x >> 31)))) : (q15_t) x;
  }

  

 

  __attribute__((always_inline)) static __inline q63_t mult32x64(
  q63_t x,
  q31_t y)
  {
    return ((((q63_t) (x & 0x00000000FFFFFFFF) * y) >> 32) +
            (((q63_t) (x >> 32) * y)));
  }

  

 

  __attribute__((always_inline)) static __inline uint32_t arm_recip_q31(
  q31_t in,
  q31_t * dst,
  q31_t * pRecipTable)
  {
    q31_t out;
    uint32_t tempVal;
    uint32_t index, i;
    uint32_t signBits;

    if (in > 0)
    {
      signBits = ((uint32_t) (__clz( in) - 1));
    }
    else
    {
      signBits = ((uint32_t) (__clz(-in) - 1));
    }

     
    in = (in << signBits);

     
    index = (uint32_t)(in >> 24);
    index = (index & 0x0000003F);

     
    out = pRecipTable[index];

     
     
    for (i = 0U; i < 2U; i++)
    {
      tempVal = (uint32_t) (((q63_t) in * out) >> 31);
      tempVal = 0x7FFFFFFFu - tempVal;
       
       
      out = clip_q63_to_q31(((q63_t) out * tempVal) >> 30);
    }

     
    *dst = out;

     
    return (signBits + 1U);
  }


  

 
  __attribute__((always_inline)) static __inline uint32_t arm_recip_q15(
  q15_t in,
  q15_t * dst,
  q15_t * pRecipTable)
  {
    q15_t out = 0;
    uint32_t tempVal = 0;
    uint32_t index = 0, i = 0;
    uint32_t signBits = 0;

    if (in > 0)
    {
      signBits = ((uint32_t)(__clz( in) - 17));
    }
    else
    {
      signBits = ((uint32_t)(__clz(-in) - 17));
    }

     
    in = (in << signBits);

     
    index = (uint32_t)(in >>  8);
    index = (index & 0x0000003F);

     
    out = pRecipTable[index];

     
     
    for (i = 0U; i < 2U; i++)
    {
      tempVal = (uint32_t) (((q31_t) in * out) >> 15);
      tempVal = 0x7FFFu - tempVal;
       
      out = (q15_t) (((q31_t) out * tempVal) >> 14);
       
    }

     
    *dst = out;

     
    return (signBits + 1);
  }




 
#line 1005 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"


  

 
  typedef struct
  {
    uint16_t numTaps;         
    q7_t *pState;             
    q7_t *pCoeffs;            
  } arm_fir_instance_q7;

  

 
  typedef struct
  {
    uint16_t numTaps;          
    q15_t *pState;             
    q15_t *pCoeffs;            
  } arm_fir_instance_q15;

  

 
  typedef struct
  {
    uint16_t numTaps;          
    q31_t *pState;             
    q31_t *pCoeffs;            
  } arm_fir_instance_q31;

  

 
  typedef struct
  {
    uint16_t numTaps;      
    float32_t *pState;     
    float32_t *pCoeffs;    
  } arm_fir_instance_f32;


  





 
  void arm_fir_q7(
  const arm_fir_instance_q7 * S,
  q7_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  






 
  void arm_fir_init_q7(
  arm_fir_instance_q7 * S,
  uint16_t numTaps,
  q7_t * pCoeffs,
  q7_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_q15(
  const arm_fir_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_fir_fast_q15(
  const arm_fir_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  








 
  arm_status arm_fir_init_q15(
  arm_fir_instance_q15 * S,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_q31(
  const arm_fir_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_fir_fast_q31(
  const arm_fir_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  






 
  void arm_fir_init_q31(
  arm_fir_instance_q31 * S,
  uint16_t numTaps,
  q31_t * pCoeffs,
  q31_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_f32(
  const arm_fir_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  






 
  void arm_fir_init_f32(
  arm_fir_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);


  

 
  typedef struct
  {
    int8_t numStages;         
    q15_t *pState;            
    q15_t *pCoeffs;           
    int8_t postShift;         
  } arm_biquad_casd_df1_inst_q15;

  

 
  typedef struct
  {
    uint32_t numStages;       
    q31_t *pState;            
    q31_t *pCoeffs;           
    uint8_t postShift;        
  } arm_biquad_casd_df1_inst_q31;

  

 
  typedef struct
  {
    uint32_t numStages;       
    float32_t *pState;        
    float32_t *pCoeffs;       
  } arm_biquad_casd_df1_inst_f32;


  





 
  void arm_biquad_cascade_df1_q15(
  const arm_biquad_casd_df1_inst_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  






 
  void arm_biquad_cascade_df1_init_q15(
  arm_biquad_casd_df1_inst_q15 * S,
  uint8_t numStages,
  q15_t * pCoeffs,
  q15_t * pState,
  int8_t postShift);


  





 
  void arm_biquad_cascade_df1_fast_q15(
  const arm_biquad_casd_df1_inst_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cascade_df1_q31(
  const arm_biquad_casd_df1_inst_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cascade_df1_fast_q31(
  const arm_biquad_casd_df1_inst_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  






 
  void arm_biquad_cascade_df1_init_q31(
  arm_biquad_casd_df1_inst_q31 * S,
  uint8_t numStages,
  q31_t * pCoeffs,
  q31_t * pState,
  int8_t postShift);


  





 
  void arm_biquad_cascade_df1_f32(
  const arm_biquad_casd_df1_inst_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cascade_df1_init_f32(
  arm_biquad_casd_df1_inst_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);


  

 
  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    float32_t *pData;      
  } arm_matrix_instance_f32;


  

 
  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    float64_t *pData;      
  } arm_matrix_instance_f64;

  

 
  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    q15_t *pData;          
  } arm_matrix_instance_q15;

  

 
  typedef struct
  {
    uint16_t numRows;      
    uint16_t numCols;      
    q31_t *pData;          
  } arm_matrix_instance_q31;


  






 
  arm_status arm_mat_add_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst);


  






 
  arm_status arm_mat_add_q15(
  const arm_matrix_instance_q15 * pSrcA,
  const arm_matrix_instance_q15 * pSrcB,
  arm_matrix_instance_q15 * pDst);


  






 
  arm_status arm_mat_add_q31(
  const arm_matrix_instance_q31 * pSrcA,
  const arm_matrix_instance_q31 * pSrcB,
  arm_matrix_instance_q31 * pDst);


  






 
  arm_status arm_mat_cmplx_mult_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst);


  






 
  arm_status arm_mat_cmplx_mult_q15(
  const arm_matrix_instance_q15 * pSrcA,
  const arm_matrix_instance_q15 * pSrcB,
  arm_matrix_instance_q15 * pDst,
  q15_t * pScratch);


  






 
  arm_status arm_mat_cmplx_mult_q31(
  const arm_matrix_instance_q31 * pSrcA,
  const arm_matrix_instance_q31 * pSrcB,
  arm_matrix_instance_q31 * pDst);


  





 
  arm_status arm_mat_trans_f32(
  const arm_matrix_instance_f32 * pSrc,
  arm_matrix_instance_f32 * pDst);


  





 
  arm_status arm_mat_trans_q15(
  const arm_matrix_instance_q15 * pSrc,
  arm_matrix_instance_q15 * pDst);


  





 
  arm_status arm_mat_trans_q31(
  const arm_matrix_instance_q31 * pSrc,
  arm_matrix_instance_q31 * pDst);


  






 
  arm_status arm_mat_mult_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst);


  







 
  arm_status arm_mat_mult_q15(
  const arm_matrix_instance_q15 * pSrcA,
  const arm_matrix_instance_q15 * pSrcB,
  arm_matrix_instance_q15 * pDst,
  q15_t * pState);


  







 
  arm_status arm_mat_mult_fast_q15(
  const arm_matrix_instance_q15 * pSrcA,
  const arm_matrix_instance_q15 * pSrcB,
  arm_matrix_instance_q15 * pDst,
  q15_t * pState);


  






 
  arm_status arm_mat_mult_q31(
  const arm_matrix_instance_q31 * pSrcA,
  const arm_matrix_instance_q31 * pSrcB,
  arm_matrix_instance_q31 * pDst);


  






 
  arm_status arm_mat_mult_fast_q31(
  const arm_matrix_instance_q31 * pSrcA,
  const arm_matrix_instance_q31 * pSrcB,
  arm_matrix_instance_q31 * pDst);


  






 
  arm_status arm_mat_sub_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst);


  






 
  arm_status arm_mat_sub_q15(
  const arm_matrix_instance_q15 * pSrcA,
  const arm_matrix_instance_q15 * pSrcB,
  arm_matrix_instance_q15 * pDst);


  






 
  arm_status arm_mat_sub_q31(
  const arm_matrix_instance_q31 * pSrcA,
  const arm_matrix_instance_q31 * pSrcB,
  arm_matrix_instance_q31 * pDst);


  






 
  arm_status arm_mat_scale_f32(
  const arm_matrix_instance_f32 * pSrc,
  float32_t scale,
  arm_matrix_instance_f32 * pDst);


  







 
  arm_status arm_mat_scale_q15(
  const arm_matrix_instance_q15 * pSrc,
  q15_t scaleFract,
  int32_t shift,
  arm_matrix_instance_q15 * pDst);


  







 
  arm_status arm_mat_scale_q31(
  const arm_matrix_instance_q31 * pSrc,
  q31_t scaleFract,
  int32_t shift,
  arm_matrix_instance_q31 * pDst);


  





 
  void arm_mat_init_q31(
  arm_matrix_instance_q31 * S,
  uint16_t nRows,
  uint16_t nColumns,
  q31_t * pData);


  





 
  void arm_mat_init_q15(
  arm_matrix_instance_q15 * S,
  uint16_t nRows,
  uint16_t nColumns,
  q15_t * pData);


  





 
  void arm_mat_init_f32(
  arm_matrix_instance_f32 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float32_t * pData);



  

 
  typedef struct
  {
    q15_t A0;            




    q31_t A1;            

    q15_t state[3];      
    q15_t Kp;            
    q15_t Ki;            
    q15_t Kd;            
  } arm_pid_instance_q15;

  

 
  typedef struct
  {
    q31_t A0;             
    q31_t A1;             
    q31_t A2;             
    q31_t state[3];       
    q31_t Kp;             
    q31_t Ki;             
    q31_t Kd;             
  } arm_pid_instance_q31;

  

 
  typedef struct
  {
    float32_t A0;           
    float32_t A1;           
    float32_t A2;           
    float32_t state[3];     
    float32_t Kp;           
    float32_t Ki;           
    float32_t Kd;           
  } arm_pid_instance_f32;



  



 
  void arm_pid_init_f32(
  arm_pid_instance_f32 * S,
  int32_t resetStateFlag);


  


 
  void arm_pid_reset_f32(
  arm_pid_instance_f32 * S);


  



 
  void arm_pid_init_q31(
  arm_pid_instance_q31 * S,
  int32_t resetStateFlag);


  


 

  void arm_pid_reset_q31(
  arm_pid_instance_q31 * S);


  



 
  void arm_pid_init_q15(
  arm_pid_instance_q15 * S,
  int32_t resetStateFlag);


  


 
  void arm_pid_reset_q15(
  arm_pid_instance_q15 * S);


  

 
  typedef struct
  {
    uint32_t nValues;            
    float32_t x1;                
    float32_t xSpacing;          
    float32_t *pYData;           
  } arm_linear_interp_instance_f32;

  

 
  typedef struct
  {
    uint16_t numRows;    
    uint16_t numCols;    
    float32_t *pData;    
  } arm_bilinear_interp_instance_f32;

   

 
  typedef struct
  {
    uint16_t numRows;    
    uint16_t numCols;    
    q31_t *pData;        
  } arm_bilinear_interp_instance_q31;

   

 
  typedef struct
  {
    uint16_t numRows;    
    uint16_t numCols;    
    q15_t *pData;        
  } arm_bilinear_interp_instance_q15;

   

 
  typedef struct
  {
    uint16_t numRows;    
    uint16_t numCols;    
    q7_t *pData;         
  } arm_bilinear_interp_instance_q7;


  





 
  void arm_mult_q7(
  q7_t * pSrcA,
  q7_t * pSrcB,
  q7_t * pDst,
  uint32_t blockSize);


  





 
  void arm_mult_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_mult_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_mult_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t fftLen;                  
    uint8_t ifftFlag;                 
    uint8_t bitReverseFlag;           
    q15_t *pTwiddle;                  
    uint16_t *pBitRevTable;           
    uint16_t twidCoefModifier;        
    uint16_t bitRevFactor;            
  } arm_cfft_radix2_instance_q15;

 
  arm_status arm_cfft_radix2_init_q15(
  arm_cfft_radix2_instance_q15 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

 
  void arm_cfft_radix2_q15(
  const arm_cfft_radix2_instance_q15 * S,
  q15_t * pSrc);


  

 
  typedef struct
  {
    uint16_t fftLen;                  
    uint8_t ifftFlag;                 
    uint8_t bitReverseFlag;           
    q15_t *pTwiddle;                  
    uint16_t *pBitRevTable;           
    uint16_t twidCoefModifier;        
    uint16_t bitRevFactor;            
  } arm_cfft_radix4_instance_q15;

 
  arm_status arm_cfft_radix4_init_q15(
  arm_cfft_radix4_instance_q15 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

 
  void arm_cfft_radix4_q15(
  const arm_cfft_radix4_instance_q15 * S,
  q15_t * pSrc);

  

 
  typedef struct
  {
    uint16_t fftLen;                  
    uint8_t ifftFlag;                 
    uint8_t bitReverseFlag;           
    q31_t *pTwiddle;                  
    uint16_t *pBitRevTable;           
    uint16_t twidCoefModifier;        
    uint16_t bitRevFactor;            
  } arm_cfft_radix2_instance_q31;

 
  arm_status arm_cfft_radix2_init_q31(
  arm_cfft_radix2_instance_q31 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

 
  void arm_cfft_radix2_q31(
  const arm_cfft_radix2_instance_q31 * S,
  q31_t * pSrc);

  

 
  typedef struct
  {
    uint16_t fftLen;                  
    uint8_t ifftFlag;                 
    uint8_t bitReverseFlag;           
    q31_t *pTwiddle;                  
    uint16_t *pBitRevTable;           
    uint16_t twidCoefModifier;        
    uint16_t bitRevFactor;            
  } arm_cfft_radix4_instance_q31;

 
  void arm_cfft_radix4_q31(
  const arm_cfft_radix4_instance_q31 * S,
  q31_t * pSrc);

 
  arm_status arm_cfft_radix4_init_q31(
  arm_cfft_radix4_instance_q31 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

  

 
  typedef struct
  {
    uint16_t fftLen;                    
    uint8_t ifftFlag;                   
    uint8_t bitReverseFlag;             
    float32_t *pTwiddle;                
    uint16_t *pBitRevTable;             
    uint16_t twidCoefModifier;          
    uint16_t bitRevFactor;              
    float32_t onebyfftLen;              
  } arm_cfft_radix2_instance_f32;

 
  arm_status arm_cfft_radix2_init_f32(
  arm_cfft_radix2_instance_f32 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

 
  void arm_cfft_radix2_f32(
  const arm_cfft_radix2_instance_f32 * S,
  float32_t * pSrc);

  

 
  typedef struct
  {
    uint16_t fftLen;                    
    uint8_t ifftFlag;                   
    uint8_t bitReverseFlag;             
    float32_t *pTwiddle;                
    uint16_t *pBitRevTable;             
    uint16_t twidCoefModifier;          
    uint16_t bitRevFactor;              
    float32_t onebyfftLen;              
  } arm_cfft_radix4_instance_f32;

 
  arm_status arm_cfft_radix4_init_f32(
  arm_cfft_radix4_instance_f32 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

 
  void arm_cfft_radix4_f32(
  const arm_cfft_radix4_instance_f32 * S,
  float32_t * pSrc);

  

 
  typedef struct
  {
    uint16_t fftLen;                    
    const q15_t *pTwiddle;              
    const uint16_t *pBitRevTable;       
    uint16_t bitRevLength;              
  } arm_cfft_instance_q15;

void arm_cfft_q15(
    const arm_cfft_instance_q15 * S,
    q15_t * p1,
    uint8_t ifftFlag,
    uint8_t bitReverseFlag);

  

 
  typedef struct
  {
    uint16_t fftLen;                    
    const q31_t *pTwiddle;              
    const uint16_t *pBitRevTable;       
    uint16_t bitRevLength;              
  } arm_cfft_instance_q31;

void arm_cfft_q31(
    const arm_cfft_instance_q31 * S,
    q31_t * p1,
    uint8_t ifftFlag,
    uint8_t bitReverseFlag);

  

 
  typedef struct
  {
    uint16_t fftLen;                    
    const float32_t *pTwiddle;          
    const uint16_t *pBitRevTable;       
    uint16_t bitRevLength;              
  } arm_cfft_instance_f32;

  void arm_cfft_f32(
  const arm_cfft_instance_f32 * S,
  float32_t * p1,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag);

  

 
  typedef struct
  {
    uint32_t fftLenReal;                       
    uint8_t ifftFlagR;                         
    uint8_t bitReverseFlagR;                   
    uint32_t twidCoefRModifier;                
    q15_t *pTwiddleAReal;                      
    q15_t *pTwiddleBReal;                      
    const arm_cfft_instance_q15 *pCfft;        
  } arm_rfft_instance_q15;

  arm_status arm_rfft_init_q15(
  arm_rfft_instance_q15 * S,
  uint32_t fftLenReal,
  uint32_t ifftFlagR,
  uint32_t bitReverseFlag);

  void arm_rfft_q15(
  const arm_rfft_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst);

  

 
  typedef struct
  {
    uint32_t fftLenReal;                         
    uint8_t ifftFlagR;                           
    uint8_t bitReverseFlagR;                     
    uint32_t twidCoefRModifier;                  
    q31_t *pTwiddleAReal;                        
    q31_t *pTwiddleBReal;                        
    const arm_cfft_instance_q31 *pCfft;          
  } arm_rfft_instance_q31;

  arm_status arm_rfft_init_q31(
  arm_rfft_instance_q31 * S,
  uint32_t fftLenReal,
  uint32_t ifftFlagR,
  uint32_t bitReverseFlag);

  void arm_rfft_q31(
  const arm_rfft_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst);

  

 
  typedef struct
  {
    uint32_t fftLenReal;                         
    uint16_t fftLenBy2;                          
    uint8_t ifftFlagR;                           
    uint8_t bitReverseFlagR;                     
    uint32_t twidCoefRModifier;                      
    float32_t *pTwiddleAReal;                    
    float32_t *pTwiddleBReal;                    
    arm_cfft_radix4_instance_f32 *pCfft;         
  } arm_rfft_instance_f32;

  arm_status arm_rfft_init_f32(
  arm_rfft_instance_f32 * S,
  arm_cfft_radix4_instance_f32 * S_CFFT,
  uint32_t fftLenReal,
  uint32_t ifftFlagR,
  uint32_t bitReverseFlag);

  void arm_rfft_f32(
  const arm_rfft_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst);

  

 
typedef struct
  {
    arm_cfft_instance_f32 Sint;       
    uint16_t fftLenRFFT;              
    float32_t * pTwiddleRFFT;         
  } arm_rfft_fast_instance_f32 ;

arm_status arm_rfft_fast_init_f32 (
   arm_rfft_fast_instance_f32 * S,
   uint16_t fftLen);

void arm_rfft_fast_f32(
  arm_rfft_fast_instance_f32 * S,
  float32_t * p, float32_t * pOut,
  uint8_t ifftFlag);

  

 
  typedef struct
  {
    uint16_t N;                           
    uint16_t Nby2;                        
    float32_t normalize;                  
    float32_t *pTwiddle;                  
    float32_t *pCosFactor;                
    arm_rfft_instance_f32 *pRfft;         
    arm_cfft_radix4_instance_f32 *pCfft;  
  } arm_dct4_instance_f32;


  








 
  arm_status arm_dct4_init_f32(
  arm_dct4_instance_f32 * S,
  arm_rfft_instance_f32 * S_RFFT,
  arm_cfft_radix4_instance_f32 * S_CFFT,
  uint16_t N,
  uint16_t Nby2,
  float32_t normalize);


  




 
  void arm_dct4_f32(
  const arm_dct4_instance_f32 * S,
  float32_t * pState,
  float32_t * pInlineBuffer);


  

 
  typedef struct
  {
    uint16_t N;                           
    uint16_t Nby2;                        
    q31_t normalize;                      
    q31_t *pTwiddle;                      
    q31_t *pCosFactor;                    
    arm_rfft_instance_q31 *pRfft;         
    arm_cfft_radix4_instance_q31 *pCfft;  
  } arm_dct4_instance_q31;


  








 
  arm_status arm_dct4_init_q31(
  arm_dct4_instance_q31 * S,
  arm_rfft_instance_q31 * S_RFFT,
  arm_cfft_radix4_instance_q31 * S_CFFT,
  uint16_t N,
  uint16_t Nby2,
  q31_t normalize);


  




 
  void arm_dct4_q31(
  const arm_dct4_instance_q31 * S,
  q31_t * pState,
  q31_t * pInlineBuffer);


  

 
  typedef struct
  {
    uint16_t N;                           
    uint16_t Nby2;                        
    q15_t normalize;                      
    q15_t *pTwiddle;                      
    q15_t *pCosFactor;                    
    arm_rfft_instance_q15 *pRfft;         
    arm_cfft_radix4_instance_q15 *pCfft;  
  } arm_dct4_instance_q15;


  








 
  arm_status arm_dct4_init_q15(
  arm_dct4_instance_q15 * S,
  arm_rfft_instance_q15 * S_RFFT,
  arm_cfft_radix4_instance_q15 * S_CFFT,
  uint16_t N,
  uint16_t Nby2,
  q15_t normalize);


  




 
  void arm_dct4_q15(
  const arm_dct4_instance_q15 * S,
  q15_t * pState,
  q15_t * pInlineBuffer);


  





 
  void arm_add_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize);


  





 
  void arm_add_q7(
  q7_t * pSrcA,
  q7_t * pSrcB,
  q7_t * pDst,
  uint32_t blockSize);


  





 
  void arm_add_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_add_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_sub_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t blockSize);


  





 
  void arm_sub_q7(
  q7_t * pSrcA,
  q7_t * pSrcB,
  q7_t * pDst,
  uint32_t blockSize);


  





 
  void arm_sub_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_sub_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_scale_f32(
  float32_t * pSrc,
  float32_t scale,
  float32_t * pDst,
  uint32_t blockSize);


  






 
  void arm_scale_q7(
  q7_t * pSrc,
  q7_t scaleFract,
  int8_t shift,
  q7_t * pDst,
  uint32_t blockSize);


  






 
  void arm_scale_q15(
  q15_t * pSrc,
  q15_t scaleFract,
  int8_t shift,
  q15_t * pDst,
  uint32_t blockSize);


  






 
  void arm_scale_q31(
  q31_t * pSrc,
  q31_t scaleFract,
  int8_t shift,
  q31_t * pDst,
  uint32_t blockSize);


  




 
  void arm_abs_q7(
  q7_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  




 
  void arm_abs_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  




 
  void arm_abs_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  




 
  void arm_abs_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t blockSize,
  float32_t * result);


  





 
  void arm_dot_prod_q7(
  q7_t * pSrcA,
  q7_t * pSrcB,
  uint32_t blockSize,
  q31_t * result);


  





 
  void arm_dot_prod_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  uint32_t blockSize,
  q63_t * result);


  





 
  void arm_dot_prod_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  uint32_t blockSize,
  q63_t * result);


  





 
  void arm_shift_q7(
  q7_t * pSrc,
  int8_t shiftBits,
  q7_t * pDst,
  uint32_t blockSize);


  





 
  void arm_shift_q15(
  q15_t * pSrc,
  int8_t shiftBits,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_shift_q31(
  q31_t * pSrc,
  int8_t shiftBits,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_offset_f32(
  float32_t * pSrc,
  float32_t offset,
  float32_t * pDst,
  uint32_t blockSize);


  





 
  void arm_offset_q7(
  q7_t * pSrc,
  q7_t offset,
  q7_t * pDst,
  uint32_t blockSize);


  





 
  void arm_offset_q15(
  q15_t * pSrc,
  q15_t offset,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_offset_q31(
  q31_t * pSrc,
  q31_t offset,
  q31_t * pDst,
  uint32_t blockSize);


  




 
  void arm_negate_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  




 
  void arm_negate_q7(
  q7_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  




 
  void arm_negate_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  




 
  void arm_negate_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  




 
  void arm_copy_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  




 
  void arm_copy_q7(
  q7_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  




 
  void arm_copy_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  




 
  void arm_copy_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  




 
  void arm_fill_f32(
  float32_t value,
  float32_t * pDst,
  uint32_t blockSize);


  




 
  void arm_fill_q7(
  q7_t value,
  q7_t * pDst,
  uint32_t blockSize);


  




 
  void arm_fill_q15(
  q15_t value,
  q15_t * pDst,
  uint32_t blockSize);


  




 
  void arm_fill_q31(
  q31_t value,
  q31_t * pDst,
  uint32_t blockSize);









 
  void arm_conv_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst);


  








 
  void arm_conv_opt_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  q15_t * pScratch1,
  q15_t * pScratch2);









 
  void arm_conv_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst);


  






 
  void arm_conv_fast_q15(
          q15_t * pSrcA,
          uint32_t srcALen,
          q15_t * pSrcB,
          uint32_t srcBLen,
          q15_t * pDst);


  








 
  void arm_conv_fast_opt_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  q15_t * pScratch1,
  q15_t * pScratch2);


  






 
  void arm_conv_q31(
  q31_t * pSrcA,
  uint32_t srcALen,
  q31_t * pSrcB,
  uint32_t srcBLen,
  q31_t * pDst);


  






 
  void arm_conv_fast_q31(
  q31_t * pSrcA,
  uint32_t srcALen,
  q31_t * pSrcB,
  uint32_t srcBLen,
  q31_t * pDst);


    








 
  void arm_conv_opt_q7(
  q7_t * pSrcA,
  uint32_t srcALen,
  q7_t * pSrcB,
  uint32_t srcBLen,
  q7_t * pDst,
  q15_t * pScratch1,
  q15_t * pScratch2);


  






 
  void arm_conv_q7(
  q7_t * pSrcA,
  uint32_t srcALen,
  q7_t * pSrcB,
  uint32_t srcBLen,
  q7_t * pDst);


  









 
  arm_status arm_conv_partial_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);


  











 
  arm_status arm_conv_partial_opt_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints,
  q15_t * pScratch1,
  q15_t * pScratch2);


  









 
  arm_status arm_conv_partial_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);


  









 
  arm_status arm_conv_partial_fast_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);


  











 
  arm_status arm_conv_partial_fast_opt_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints,
  q15_t * pScratch1,
  q15_t * pScratch2);


  









 
  arm_status arm_conv_partial_q31(
  q31_t * pSrcA,
  uint32_t srcALen,
  q31_t * pSrcB,
  uint32_t srcBLen,
  q31_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);


  









 
  arm_status arm_conv_partial_fast_q31(
  q31_t * pSrcA,
  uint32_t srcALen,
  q31_t * pSrcB,
  uint32_t srcBLen,
  q31_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);


  











 
  arm_status arm_conv_partial_opt_q7(
  q7_t * pSrcA,
  uint32_t srcALen,
  q7_t * pSrcB,
  uint32_t srcBLen,
  q7_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints,
  q15_t * pScratch1,
  q15_t * pScratch2);












 
  arm_status arm_conv_partial_q7(
  q7_t * pSrcA,
  uint32_t srcALen,
  q7_t * pSrcB,
  uint32_t srcBLen,
  q7_t * pDst,
  uint32_t firstIndex,
  uint32_t numPoints);


  

 
  typedef struct
  {
    uint8_t M;                   
    uint16_t numTaps;            
    q15_t *pCoeffs;              
    q15_t *pState;               
  } arm_fir_decimate_instance_q15;

  

 
  typedef struct
  {
    uint8_t M;                   
    uint16_t numTaps;            
    q31_t *pCoeffs;              
    q31_t *pState;               
  } arm_fir_decimate_instance_q31;

  

 
  typedef struct
  {
    uint8_t M;                   
    uint16_t numTaps;            
    float32_t *pCoeffs;          
    float32_t *pState;           
  } arm_fir_decimate_instance_f32;


  





 
  void arm_fir_decimate_f32(
  const arm_fir_decimate_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  









 
  arm_status arm_fir_decimate_init_f32(
  arm_fir_decimate_instance_f32 * S,
  uint16_t numTaps,
  uint8_t M,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_decimate_q15(
  const arm_fir_decimate_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_fir_decimate_fast_q15(
  const arm_fir_decimate_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  









 
  arm_status arm_fir_decimate_init_q15(
  arm_fir_decimate_instance_q15 * S,
  uint16_t numTaps,
  uint8_t M,
  q15_t * pCoeffs,
  q15_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_decimate_q31(
  const arm_fir_decimate_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);

  





 
  void arm_fir_decimate_fast_q31(
  arm_fir_decimate_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  









 
  arm_status arm_fir_decimate_init_q31(
  arm_fir_decimate_instance_q31 * S,
  uint16_t numTaps,
  uint8_t M,
  q31_t * pCoeffs,
  q31_t * pState,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint8_t L;                       
    uint16_t phaseLength;            
    q15_t *pCoeffs;                  
    q15_t *pState;                   
  } arm_fir_interpolate_instance_q15;

  

 
  typedef struct
  {
    uint8_t L;                       
    uint16_t phaseLength;            
    q31_t *pCoeffs;                  
    q31_t *pState;                   
  } arm_fir_interpolate_instance_q31;

  

 
  typedef struct
  {
    uint8_t L;                      
    uint16_t phaseLength;           
    float32_t *pCoeffs;             
    float32_t *pState;              
  } arm_fir_interpolate_instance_f32;


  





 
  void arm_fir_interpolate_q15(
  const arm_fir_interpolate_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  









 
  arm_status arm_fir_interpolate_init_q15(
  arm_fir_interpolate_instance_q15 * S,
  uint8_t L,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_interpolate_q31(
  const arm_fir_interpolate_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  









 
  arm_status arm_fir_interpolate_init_q31(
  arm_fir_interpolate_instance_q31 * S,
  uint8_t L,
  uint16_t numTaps,
  q31_t * pCoeffs,
  q31_t * pState,
  uint32_t blockSize);


  





 
  void arm_fir_interpolate_f32(
  const arm_fir_interpolate_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  









 
  arm_status arm_fir_interpolate_init_f32(
  arm_fir_interpolate_instance_f32 * S,
  uint8_t L,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint8_t numStages;        
    q63_t *pState;            
    q31_t *pCoeffs;           
    uint8_t postShift;        
  } arm_biquad_cas_df1_32x64_ins_q31;


  




 
  void arm_biquad_cas_df1_32x64_q31(
  const arm_biquad_cas_df1_32x64_ins_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cas_df1_32x64_init_q31(
  arm_biquad_cas_df1_32x64_ins_q31 * S,
  uint8_t numStages,
  q31_t * pCoeffs,
  q63_t * pState,
  uint8_t postShift);


  

 
  typedef struct
  {
    uint8_t numStages;          
    float32_t *pState;          
    float32_t *pCoeffs;         
  } arm_biquad_cascade_df2T_instance_f32;

  

 
  typedef struct
  {
    uint8_t numStages;          
    float32_t *pState;          
    float32_t *pCoeffs;         
  } arm_biquad_cascade_stereo_df2T_instance_f32;

  

 
  typedef struct
  {
    uint8_t numStages;          
    float64_t *pState;          
    float64_t *pCoeffs;         
  } arm_biquad_cascade_df2T_instance_f64;


  





 
  void arm_biquad_cascade_df2T_f32(
  const arm_biquad_cascade_df2T_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cascade_stereo_df2T_f32(
  const arm_biquad_cascade_stereo_df2T_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cascade_df2T_f64(
  const arm_biquad_cascade_df2T_instance_f64 * S,
  float64_t * pSrc,
  float64_t * pDst,
  uint32_t blockSize);


  





 
  void arm_biquad_cascade_df2T_init_f32(
  arm_biquad_cascade_df2T_instance_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);


  





 
  void arm_biquad_cascade_stereo_df2T_init_f32(
  arm_biquad_cascade_stereo_df2T_instance_f32 * S,
  uint8_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);


  





 
  void arm_biquad_cascade_df2T_init_f64(
  arm_biquad_cascade_df2T_instance_f64 * S,
  uint8_t numStages,
  float64_t * pCoeffs,
  float64_t * pState);


  

 
  typedef struct
  {
    uint16_t numStages;                   
    q15_t *pState;                        
    q15_t *pCoeffs;                       
  } arm_fir_lattice_instance_q15;

  

 
  typedef struct
  {
    uint16_t numStages;                   
    q31_t *pState;                        
    q31_t *pCoeffs;                       
  } arm_fir_lattice_instance_q31;

  

 
  typedef struct
  {
    uint16_t numStages;                   
    float32_t *pState;                    
    float32_t *pCoeffs;                   
  } arm_fir_lattice_instance_f32;


  





 
  void arm_fir_lattice_init_q15(
  arm_fir_lattice_instance_q15 * S,
  uint16_t numStages,
  q15_t * pCoeffs,
  q15_t * pState);


  





 
  void arm_fir_lattice_q15(
  const arm_fir_lattice_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  





 
  void arm_fir_lattice_init_q31(
  arm_fir_lattice_instance_q31 * S,
  uint16_t numStages,
  q31_t * pCoeffs,
  q31_t * pState);


  





 
  void arm_fir_lattice_q31(
  const arm_fir_lattice_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);








 
  void arm_fir_lattice_init_f32(
  arm_fir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pCoeffs,
  float32_t * pState);


  





 
  void arm_fir_lattice_f32(
  const arm_fir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t numStages;                   
    q15_t *pState;                        
    q15_t *pkCoeffs;                      
    q15_t *pvCoeffs;                      
  } arm_iir_lattice_instance_q15;

  

 
  typedef struct
  {
    uint16_t numStages;                   
    q31_t *pState;                        
    q31_t *pkCoeffs;                      
    q31_t *pvCoeffs;                      
  } arm_iir_lattice_instance_q31;

  

 
  typedef struct
  {
    uint16_t numStages;                   
    float32_t *pState;                    
    float32_t *pkCoeffs;                  
    float32_t *pvCoeffs;                  
  } arm_iir_lattice_instance_f32;


  





 
  void arm_iir_lattice_f32(
  const arm_iir_lattice_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  







 
  void arm_iir_lattice_init_f32(
  arm_iir_lattice_instance_f32 * S,
  uint16_t numStages,
  float32_t * pkCoeffs,
  float32_t * pvCoeffs,
  float32_t * pState,
  uint32_t blockSize);


  





 
  void arm_iir_lattice_q31(
  const arm_iir_lattice_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  







 
  void arm_iir_lattice_init_q31(
  arm_iir_lattice_instance_q31 * S,
  uint16_t numStages,
  q31_t * pkCoeffs,
  q31_t * pvCoeffs,
  q31_t * pState,
  uint32_t blockSize);


  





 
  void arm_iir_lattice_q15(
  const arm_iir_lattice_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);










 
  void arm_iir_lattice_init_q15(
  arm_iir_lattice_instance_q15 * S,
  uint16_t numStages,
  q15_t * pkCoeffs,
  q15_t * pvCoeffs,
  q15_t * pState,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t numTaps;     
    float32_t *pState;    
    float32_t *pCoeffs;   
    float32_t mu;         
  } arm_lms_instance_f32;


  







 
  void arm_lms_f32(
  const arm_lms_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pRef,
  float32_t * pOut,
  float32_t * pErr,
  uint32_t blockSize);


  







 
  void arm_lms_init_f32(
  arm_lms_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  float32_t mu,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t numTaps;     
    q15_t *pState;        
    q15_t *pCoeffs;       
    q15_t mu;             
    uint32_t postShift;   
  } arm_lms_instance_q15;


  








 
  void arm_lms_init_q15(
  arm_lms_instance_q15 * S,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  q15_t mu,
  uint32_t blockSize,
  uint32_t postShift);


  







 
  void arm_lms_q15(
  const arm_lms_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pRef,
  q15_t * pOut,
  q15_t * pErr,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t numTaps;     
    q31_t *pState;        
    q31_t *pCoeffs;       
    q31_t mu;             
    uint32_t postShift;   
  } arm_lms_instance_q31;


  







 
  void arm_lms_q31(
  const arm_lms_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pRef,
  q31_t * pOut,
  q31_t * pErr,
  uint32_t blockSize);


  








 
  void arm_lms_init_q31(
  arm_lms_instance_q31 * S,
  uint16_t numTaps,
  q31_t * pCoeffs,
  q31_t * pState,
  q31_t mu,
  uint32_t blockSize,
  uint32_t postShift);


  

 
  typedef struct
  {
    uint16_t numTaps;      
    float32_t *pState;     
    float32_t *pCoeffs;    
    float32_t mu;          
    float32_t energy;      
    float32_t x0;          
  } arm_lms_norm_instance_f32;


  







 
  void arm_lms_norm_f32(
  arm_lms_norm_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pRef,
  float32_t * pOut,
  float32_t * pErr,
  uint32_t blockSize);


  







 
  void arm_lms_norm_init_f32(
  arm_lms_norm_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  float32_t mu,
  uint32_t blockSize);


  

 
  typedef struct
  {
    uint16_t numTaps;      
    q31_t *pState;         
    q31_t *pCoeffs;        
    q31_t mu;              
    uint8_t postShift;     
    q31_t *recipTable;     
    q31_t energy;          
    q31_t x0;              
  } arm_lms_norm_instance_q31;


  







 
  void arm_lms_norm_q31(
  arm_lms_norm_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pRef,
  q31_t * pOut,
  q31_t * pErr,
  uint32_t blockSize);


  








 
  void arm_lms_norm_init_q31(
  arm_lms_norm_instance_q31 * S,
  uint16_t numTaps,
  q31_t * pCoeffs,
  q31_t * pState,
  q31_t mu,
  uint32_t blockSize,
  uint8_t postShift);


  

 
  typedef struct
  {
    uint16_t numTaps;      
    q15_t *pState;         
    q15_t *pCoeffs;        
    q15_t mu;              
    uint8_t postShift;     
    q15_t *recipTable;     
    q15_t energy;          
    q15_t x0;              
  } arm_lms_norm_instance_q15;


  







 
  void arm_lms_norm_q15(
  arm_lms_norm_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pRef,
  q15_t * pOut,
  q15_t * pErr,
  uint32_t blockSize);


  








 
  void arm_lms_norm_init_q15(
  arm_lms_norm_instance_q15 * S,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  q15_t mu,
  uint32_t blockSize,
  uint8_t postShift);


  






 
  void arm_correlate_f32(
  float32_t * pSrcA,
  uint32_t srcALen,
  float32_t * pSrcB,
  uint32_t srcBLen,
  float32_t * pDst);


   







 
  void arm_correlate_opt_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  q15_t * pScratch);


  






 

  void arm_correlate_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst);


  






 

  void arm_correlate_fast_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst);


  







 
  void arm_correlate_fast_opt_q15(
  q15_t * pSrcA,
  uint32_t srcALen,
  q15_t * pSrcB,
  uint32_t srcBLen,
  q15_t * pDst,
  q15_t * pScratch);


  






 
  void arm_correlate_q31(
  q31_t * pSrcA,
  uint32_t srcALen,
  q31_t * pSrcB,
  uint32_t srcBLen,
  q31_t * pDst);


  






 
  void arm_correlate_fast_q31(
  q31_t * pSrcA,
  uint32_t srcALen,
  q31_t * pSrcB,
  uint32_t srcBLen,
  q31_t * pDst);


 








 
  void arm_correlate_opt_q7(
  q7_t * pSrcA,
  uint32_t srcALen,
  q7_t * pSrcB,
  uint32_t srcBLen,
  q7_t * pDst,
  q15_t * pScratch1,
  q15_t * pScratch2);


  






 
  void arm_correlate_q7(
  q7_t * pSrcA,
  uint32_t srcALen,
  q7_t * pSrcB,
  uint32_t srcBLen,
  q7_t * pDst);


  

 
  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    float32_t *pState;             
    float32_t *pCoeffs;            
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_f32;

  

 
  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    q31_t *pState;                 
    q31_t *pCoeffs;                
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_q31;

  

 
  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    q15_t *pState;                 
    q15_t *pCoeffs;                
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_q15;

  

 
  typedef struct
  {
    uint16_t numTaps;              
    uint16_t stateIndex;           
    q7_t *pState;                  
    q7_t *pCoeffs;                 
    uint16_t maxDelay;             
    int32_t *pTapDelay;            
  } arm_fir_sparse_instance_q7;


  






 
  void arm_fir_sparse_f32(
  arm_fir_sparse_instance_f32 * S,
  float32_t * pSrc,
  float32_t * pDst,
  float32_t * pScratchIn,
  uint32_t blockSize);


  








 
  void arm_fir_sparse_init_f32(
  arm_fir_sparse_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  int32_t * pTapDelay,
  uint16_t maxDelay,
  uint32_t blockSize);


  






 
  void arm_fir_sparse_q31(
  arm_fir_sparse_instance_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  q31_t * pScratchIn,
  uint32_t blockSize);


  








 
  void arm_fir_sparse_init_q31(
  arm_fir_sparse_instance_q31 * S,
  uint16_t numTaps,
  q31_t * pCoeffs,
  q31_t * pState,
  int32_t * pTapDelay,
  uint16_t maxDelay,
  uint32_t blockSize);


  







 
  void arm_fir_sparse_q15(
  arm_fir_sparse_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  q15_t * pScratchIn,
  q31_t * pScratchOut,
  uint32_t blockSize);


  








 
  void arm_fir_sparse_init_q15(
  arm_fir_sparse_instance_q15 * S,
  uint16_t numTaps,
  q15_t * pCoeffs,
  q15_t * pState,
  int32_t * pTapDelay,
  uint16_t maxDelay,
  uint32_t blockSize);


  







 
  void arm_fir_sparse_q7(
  arm_fir_sparse_instance_q7 * S,
  q7_t * pSrc,
  q7_t * pDst,
  q7_t * pScratchIn,
  q31_t * pScratchOut,
  uint32_t blockSize);


  








 
  void arm_fir_sparse_init_q7(
  arm_fir_sparse_instance_q7 * S,
  uint16_t numTaps,
  q7_t * pCoeffs,
  q7_t * pState,
  int32_t * pTapDelay,
  uint16_t maxDelay,
  uint32_t blockSize);


  




 
  void arm_sin_cos_f32(
  float32_t theta,
  float32_t * pSinVal,
  float32_t * pCosVal);


  




 
  void arm_sin_cos_q31(
  q31_t theta,
  q31_t * pSinVal,
  q31_t * pCosVal);


  




 
  void arm_cmplx_conj_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples);

  




 
  void arm_cmplx_conj_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t numSamples);


  




 
  void arm_cmplx_conj_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t numSamples);


  




 
  void arm_cmplx_mag_squared_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples);


  




 
  void arm_cmplx_mag_squared_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t numSamples);


  




 
  void arm_cmplx_mag_squared_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t numSamples);


 

 

  






















































 

  


 

  




 
  __attribute__((always_inline)) static __inline float32_t arm_pid_f32(
  arm_pid_instance_f32 * S,
  float32_t in)
  {
    float32_t out;

     
    out = (S->A0 * in) +
      (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

     
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

     
    return (out);

  }

  












 
  __attribute__((always_inline)) static __inline q31_t arm_pid_q31(
  arm_pid_instance_q31 * S,
  q31_t in)
  {
    q63_t acc;
    q31_t out;

     
    acc = (q63_t) S->A0 * in;

     
    acc += (q63_t) S->A1 * S->state[0];

     
    acc += (q63_t) S->A2 * S->state[1];

     
    out = (q31_t) (acc >> 31U);

     
    out += S->state[2];

     
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

     
    return (out);
  }


  













 
  __attribute__((always_inline)) static __inline q15_t arm_pid_q15(
  arm_pid_instance_q15 * S,
  q15_t in)
  {
    q63_t acc;
    q15_t out;


    int32_t __packed *vstate;

     

     
    acc = (q31_t) __smuad((uint32_t)S->A0, (uint32_t)in);

     
    vstate = ((int32_t __packed *)(S->state));
    acc = (q63_t)__smlald((uint32_t)S->A1, (uint32_t)*vstate, (uint64_t)acc);
#line 4883 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

     
    acc += (q31_t) S->state[2] << 15;

     
    out = (q15_t) (__ssat((acc >> 15), 16));

     
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

     
    return (out);
  }

  

 


  





 
  arm_status arm_mat_inverse_f32(
  const arm_matrix_instance_f32 * src,
  arm_matrix_instance_f32 * dst);


  





 
  arm_status arm_mat_inverse_f64(
  const arm_matrix_instance_f64 * src,
  arm_matrix_instance_f64 * dst);



  

 

  



















 

  


 

  






 
  __attribute__((always_inline)) static __inline void arm_clarke_f32(
  float32_t Ia,
  float32_t Ib,
  float32_t * pIalpha,
  float32_t * pIbeta)
  {
     
    *pIalpha = Ia;

     
    *pIbeta = ((float32_t) 0.57735026919 * Ia + (float32_t) 1.15470053838 * Ib);
  }


  











 
  __attribute__((always_inline)) static __inline void arm_clarke_q31(
  q31_t Ia,
  q31_t Ib,
  q31_t * pIalpha,
  q31_t * pIbeta)
  {
    q31_t product1, product2;                     

     
    *pIalpha = Ia;

     
    product1 = (q31_t) (((q63_t) Ia * 0x24F34E8B) >> 30);

     
    product2 = (q31_t) (((q63_t) Ib * 0x49E69D16) >> 30);

     
    *pIbeta = __qadd(product1, product2);
  }

  

 

  




 
  void arm_q7_to_q31(
  q7_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);



  

 

  













 

  


 

   





 
  __attribute__((always_inline)) static __inline void arm_inv_clarke_f32(
  float32_t Ialpha,
  float32_t Ibeta,
  float32_t * pIa,
  float32_t * pIb)
  {
     
    *pIa = Ialpha;

     
    *pIb = -0.5f * Ialpha + 0.8660254039f * Ibeta;
  }


  











 
  __attribute__((always_inline)) static __inline void arm_inv_clarke_q31(
  q31_t Ialpha,
  q31_t Ibeta,
  q31_t * pIa,
  q31_t * pIb)
  {
    q31_t product1, product2;                     

     
    *pIa = Ialpha;

     
    product1 = (q31_t) (((q63_t) (Ialpha) * (0x40000000)) >> 31);

     
    product2 = (q31_t) (((q63_t) (Ibeta) * (0x6ED9EBA1)) >> 31);

     
    *pIb = __qsub(product2, product1);
  }

  

 

  




 
  void arm_q7_to_q15(
  q7_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);



  

 

  





















 

  


 

  










 
  __attribute__((always_inline)) static __inline void arm_park_f32(
  float32_t Ialpha,
  float32_t Ibeta,
  float32_t * pId,
  float32_t * pIq,
  float32_t sinVal,
  float32_t cosVal)
  {
     
    *pId = Ialpha * cosVal + Ibeta * sinVal;

     
    *pIq = -Ialpha * sinVal + Ibeta * cosVal;
  }


  













 
  __attribute__((always_inline)) static __inline void arm_park_q31(
  q31_t Ialpha,
  q31_t Ibeta,
  q31_t * pId,
  q31_t * pIq,
  q31_t sinVal,
  q31_t cosVal)
  {
    q31_t product1, product2;                     
    q31_t product3, product4;                     

     
    product1 = (q31_t) (((q63_t) (Ialpha) * (cosVal)) >> 31);

     
    product2 = (q31_t) (((q63_t) (Ibeta) * (sinVal)) >> 31);


     
    product3 = (q31_t) (((q63_t) (Ialpha) * (sinVal)) >> 31);

     
    product4 = (q31_t) (((q63_t) (Ibeta) * (cosVal)) >> 31);

     
    *pId = __qadd(product1, product2);

     
    *pIq = __qsub(product4, product3);
  }

  

 

  




 
  void arm_q7_to_float(
  q7_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  

 

  














 

  


 

   







 
  __attribute__((always_inline)) static __inline void arm_inv_park_f32(
  float32_t Id,
  float32_t Iq,
  float32_t * pIalpha,
  float32_t * pIbeta,
  float32_t sinVal,
  float32_t cosVal)
  {
     
    *pIalpha = Id * cosVal - Iq * sinVal;

     
    *pIbeta = Id * sinVal + Iq * cosVal;
  }


  













 
  __attribute__((always_inline)) static __inline void arm_inv_park_q31(
  q31_t Id,
  q31_t Iq,
  q31_t * pIalpha,
  q31_t * pIbeta,
  q31_t sinVal,
  q31_t cosVal)
  {
    q31_t product1, product2;                     
    q31_t product3, product4;                     

     
    product1 = (q31_t) (((q63_t) (Id) * (cosVal)) >> 31);

     
    product2 = (q31_t) (((q63_t) (Iq) * (sinVal)) >> 31);


     
    product3 = (q31_t) (((q63_t) (Id) * (sinVal)) >> 31);

     
    product4 = (q31_t) (((q63_t) (Iq) * (cosVal)) >> 31);

     
    *pIalpha = __qsub(product1, product2);

     
    *pIbeta = __qadd(product4, product3);
  }

  

 


  




 
  void arm_q31_to_float(
  q31_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);

  

 

  





























 

  


 

  





 
  __attribute__((always_inline)) static __inline float32_t arm_linear_interp_f32(
  arm_linear_interp_instance_f32 * S,
  float32_t x)
  {
    float32_t y;
    float32_t x0, x1;                             
    float32_t y0, y1;                             
    float32_t xSpacing = S->xSpacing;             
    int32_t i;                                    
    float32_t *pYData = S->pYData;                

     
    i = (int32_t) ((x - S->x1) / xSpacing);

    if (i < 0)
    {
       
      y = pYData[0];
    }
    else if ((uint32_t)i >= S->nValues)
    {
       
      y = pYData[S->nValues - 1];
    }
    else
    {
       
      x0 = S->x1 +  i      * xSpacing;
      x1 = S->x1 + (i + 1) * xSpacing;

       
      y0 = pYData[i];
      y1 = pYData[i + 1];

       
      y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0));

    }

     
    return (y);
  }


   











 
  __attribute__((always_inline)) static __inline q31_t arm_linear_interp_q31(
  q31_t * pYData,
  q31_t x,
  uint32_t nValues)
  {
    q31_t y;                                      
    q31_t y0, y1;                                 
    q31_t fract;                                  
    int32_t index;                                

     
     
     
    index = ((x & (q31_t)0xFFF00000) >> 20);

    if (index >= (int32_t)(nValues - 1))
    {
      return (pYData[nValues - 1]);
    }
    else if (index < 0)
    {
      return (pYData[0]);
    }
    else
    {
       
       
      fract = (x & 0x000FFFFF) << 11;

       
      y0 = pYData[index];
      y1 = pYData[index + 1];

       
      y = ((q31_t) ((q63_t) y0 * (0x7FFFFFFF - fract) >> 32));

       
      y += ((q31_t) (((q63_t) y1 * fract) >> 32));

       
      return (y << 1U);
    }
  }


  











 
  __attribute__((always_inline)) static __inline q15_t arm_linear_interp_q15(
  q15_t * pYData,
  q31_t x,
  uint32_t nValues)
  {
    q63_t y;                                      
    q15_t y0, y1;                                 
    q31_t fract;                                  
    int32_t index;                                

     
     
     
    index = ((x & (int32_t)0xFFF00000) >> 20);

    if (index >= (int32_t)(nValues - 1))
    {
      return (pYData[nValues - 1]);
    }
    else if (index < 0)
    {
      return (pYData[0]);
    }
    else
    {
       
       
      fract = (x & 0x000FFFFF);

       
      y0 = pYData[index];
      y1 = pYData[index + 1];

       
      y = ((q63_t) y0 * (0xFFFFF - fract));

       
      y += ((q63_t) y1 * (fract));

       
      return (q15_t) (y >> 20);
    }
  }


  










 
  __attribute__((always_inline)) static __inline q7_t arm_linear_interp_q7(
  q7_t * pYData,
  q31_t x,
  uint32_t nValues)
  {
    q31_t y;                                      
    q7_t y0, y1;                                  
    q31_t fract;                                  
    uint32_t index;                               

     
     
     
    if (x < 0)
    {
      return (pYData[0]);
    }
    index = (x >> 20) & 0xfff;

    if (index >= (nValues - 1))
    {
      return (pYData[nValues - 1]);
    }
    else
    {
       
       
      fract = (x & 0x000FFFFF);

       
      y0 = pYData[index];
      y1 = pYData[index + 1];

       
      y = ((y0 * (0xFFFFF - fract)));

       
      y += (y1 * fract);

       
      return (q7_t) (y >> 20);
     }
  }

  

 

  



 
  float32_t arm_sin_f32(
  float32_t x);


  



 
  q31_t arm_sin_q31(
  q31_t x);


  



 
  q15_t arm_sin_q15(
  q15_t x);


  



 
  float32_t arm_cos_f32(
  float32_t x);


  



 
  q31_t arm_cos_q31(
  q31_t x);


  



 
  q15_t arm_cos_q15(
  q15_t x);


  

 


  

















 


  


 

  





 
  __attribute__((always_inline)) static __inline arm_status arm_sqrt_f32(
  float32_t in,
  float32_t * pOut)
  {
    if (in >= 0.0f)
    {


      *pOut = __sqrtf(in);
#line 5744 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

      return (ARM_MATH_SUCCESS);
    }
    else
    {
      *pOut = 0.0f;
      return (ARM_MATH_ARGUMENT_ERROR);
    }
  }


  





 
  arm_status arm_sqrt_q31(
  q31_t in,
  q31_t * pOut);


  





 
  arm_status arm_sqrt_q15(
  q15_t in,
  q15_t * pOut);

  

 


  

 
  __attribute__((always_inline)) static __inline void arm_circularWrite_f32(
  int32_t * circBuffer,
  int32_t L,
  uint16_t * writeOffset,
  int32_t bufferInc,
  const int32_t * src,
  int32_t srcInc,
  uint32_t blockSize)
  {
    uint32_t i = 0U;
    int32_t wOffset;

    
 
    wOffset = *writeOffset;

     
    i = blockSize;

    while (i > 0U)
    {
       
      circBuffer[wOffset] = *src;

       
      src += srcInc;

       
      wOffset += bufferInc;
      if (wOffset >= L)
        wOffset -= L;

       
      i--;
    }

     
    *writeOffset = (uint16_t)wOffset;
  }



  

 
  __attribute__((always_inline)) static __inline void arm_circularRead_f32(
  int32_t * circBuffer,
  int32_t L,
  int32_t * readOffset,
  int32_t bufferInc,
  int32_t * dst,
  int32_t * dst_base,
  int32_t dst_length,
  int32_t dstInc,
  uint32_t blockSize)
  {
    uint32_t i = 0U;
    int32_t rOffset, dst_end;

    
 
    rOffset = *readOffset;
    dst_end = (int32_t) (dst_base + dst_length);

     
    i = blockSize;

    while (i > 0U)
    {
       
      *dst = circBuffer[rOffset];

       
      dst += dstInc;

      if (dst == (int32_t *) dst_end)
      {
        dst = dst_base;
      }

       
      rOffset += bufferInc;

      if (rOffset >= L)
      {
        rOffset -= L;
      }

       
      i--;
    }

     
    *readOffset = rOffset;
  }


  

 
  __attribute__((always_inline)) static __inline void arm_circularWrite_q15(
  q15_t * circBuffer,
  int32_t L,
  uint16_t * writeOffset,
  int32_t bufferInc,
  const q15_t * src,
  int32_t srcInc,
  uint32_t blockSize)
  {
    uint32_t i = 0U;
    int32_t wOffset;

    
 
    wOffset = *writeOffset;

     
    i = blockSize;

    while (i > 0U)
    {
       
      circBuffer[wOffset] = *src;

       
      src += srcInc;

       
      wOffset += bufferInc;
      if (wOffset >= L)
        wOffset -= L;

       
      i--;
    }

     
    *writeOffset = (uint16_t)wOffset;
  }


  

 
  __attribute__((always_inline)) static __inline void arm_circularRead_q15(
  q15_t * circBuffer,
  int32_t L,
  int32_t * readOffset,
  int32_t bufferInc,
  q15_t * dst,
  q15_t * dst_base,
  int32_t dst_length,
  int32_t dstInc,
  uint32_t blockSize)
  {
    uint32_t i = 0;
    int32_t rOffset, dst_end;

    
 
    rOffset = *readOffset;

    dst_end = (int32_t) (dst_base + dst_length);

     
    i = blockSize;

    while (i > 0U)
    {
       
      *dst = circBuffer[rOffset];

       
      dst += dstInc;

      if (dst == (q15_t *) dst_end)
      {
        dst = dst_base;
      }

       
      rOffset += bufferInc;

      if (rOffset >= L)
      {
        rOffset -= L;
      }

       
      i--;
    }

     
    *readOffset = rOffset;
  }


  

 
  __attribute__((always_inline)) static __inline void arm_circularWrite_q7(
  q7_t * circBuffer,
  int32_t L,
  uint16_t * writeOffset,
  int32_t bufferInc,
  const q7_t * src,
  int32_t srcInc,
  uint32_t blockSize)
  {
    uint32_t i = 0U;
    int32_t wOffset;

    
 
    wOffset = *writeOffset;

     
    i = blockSize;

    while (i > 0U)
    {
       
      circBuffer[wOffset] = *src;

       
      src += srcInc;

       
      wOffset += bufferInc;
      if (wOffset >= L)
        wOffset -= L;

       
      i--;
    }

     
    *writeOffset = (uint16_t)wOffset;
  }


  

 
  __attribute__((always_inline)) static __inline void arm_circularRead_q7(
  q7_t * circBuffer,
  int32_t L,
  int32_t * readOffset,
  int32_t bufferInc,
  q7_t * dst,
  q7_t * dst_base,
  int32_t dst_length,
  int32_t dstInc,
  uint32_t blockSize)
  {
    uint32_t i = 0;
    int32_t rOffset, dst_end;

    
 
    rOffset = *readOffset;

    dst_end = (int32_t) (dst_base + dst_length);

     
    i = blockSize;

    while (i > 0U)
    {
       
      *dst = circBuffer[rOffset];

       
      dst += dstInc;

      if (dst == (q7_t *) dst_end)
      {
        dst = dst_base;
      }

       
      rOffset += bufferInc;

      if (rOffset >= L)
      {
        rOffset -= L;
      }

       
      i--;
    }

     
    *readOffset = rOffset;
  }


  




 
  void arm_power_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q63_t * pResult);


  




 
  void arm_power_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);


  




 
  void arm_power_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q63_t * pResult);


  




 
  void arm_power_q7(
  q7_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult);


  




 
  void arm_mean_q7(
  q7_t * pSrc,
  uint32_t blockSize,
  q7_t * pResult);


  




 
  void arm_mean_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult);


  




 
  void arm_mean_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult);


  




 
  void arm_mean_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);


  




 
  void arm_var_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);


  




 
  void arm_var_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult);


  




 
  void arm_var_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult);


  




 
  void arm_rms_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);


  




 
  void arm_rms_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult);


  




 
  void arm_rms_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult);


  




 
  void arm_std_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult);


  




 
  void arm_std_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult);


  




 
  void arm_std_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult);


  




 
  void arm_cmplx_mag_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples);


  




 
  void arm_cmplx_mag_q31(
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t numSamples);


  




 
  void arm_cmplx_mag_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t numSamples);


  






 
  void arm_cmplx_dot_prod_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  uint32_t numSamples,
  q31_t * realResult,
  q31_t * imagResult);


  






 
  void arm_cmplx_dot_prod_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  uint32_t numSamples,
  q63_t * realResult,
  q63_t * imagResult);


  






 
  void arm_cmplx_dot_prod_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  uint32_t numSamples,
  float32_t * realResult,
  float32_t * imagResult);


  





 
  void arm_cmplx_mult_real_q15(
  q15_t * pSrcCmplx,
  q15_t * pSrcReal,
  q15_t * pCmplxDst,
  uint32_t numSamples);


  





 
  void arm_cmplx_mult_real_q31(
  q31_t * pSrcCmplx,
  q31_t * pSrcReal,
  q31_t * pCmplxDst,
  uint32_t numSamples);


  





 
  void arm_cmplx_mult_real_f32(
  float32_t * pSrcCmplx,
  float32_t * pSrcReal,
  float32_t * pCmplxDst,
  uint32_t numSamples);


  





 
  void arm_min_q7(
  q7_t * pSrc,
  uint32_t blockSize,
  q7_t * result,
  uint32_t * index);


  





 
  void arm_min_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult,
  uint32_t * pIndex);


  





 
  void arm_min_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult,
  uint32_t * pIndex);


  





 
  void arm_min_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex);








 
  void arm_max_q7(
  q7_t * pSrc,
  uint32_t blockSize,
  q7_t * pResult,
  uint32_t * pIndex);








 
  void arm_max_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult,
  uint32_t * pIndex);








 
  void arm_max_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult,
  uint32_t * pIndex);








 
  void arm_max_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex);


  





 
  void arm_cmplx_mult_cmplx_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  q15_t * pDst,
  uint32_t numSamples);


  





 
  void arm_cmplx_mult_cmplx_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  q31_t * pDst,
  uint32_t numSamples);


  





 
  void arm_cmplx_mult_cmplx_f32(
  float32_t * pSrcA,
  float32_t * pSrcB,
  float32_t * pDst,
  uint32_t numSamples);


  




 
  void arm_float_to_q31(
  float32_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  




 
  void arm_float_to_q15(
  float32_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  




 
  void arm_float_to_q7(
  float32_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  




 
  void arm_q31_to_q15(
  q31_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);


  




 
  void arm_q31_to_q7(
  q31_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  




 
  void arm_q15_to_float(
  q15_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize);


  




 
  void arm_q15_to_q31(
  q15_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);


  




 
  void arm_q15_to_q7(
  q15_t * pSrc,
  q7_t * pDst,
  uint32_t blockSize);


  

 

  

















































 

  


 


  






 
  __attribute__((always_inline)) static __inline float32_t arm_bilinear_interp_f32(
  const arm_bilinear_interp_instance_f32 * S,
  float32_t X,
  float32_t Y)
  {
    float32_t out;
    float32_t f00, f01, f10, f11;
    float32_t *pData = S->pData;
    int32_t xIndex, yIndex, index;
    float32_t xdiff, ydiff;
    float32_t b1, b2, b3, b4;

    xIndex = (int32_t) X;
    yIndex = (int32_t) Y;

     
     
    if (xIndex < 0 || xIndex > (S->numRows - 1) || yIndex < 0 || yIndex > (S->numCols - 1))
    {
      return (0);
    }

     
    index = (xIndex - 1) + (yIndex - 1) * S->numCols;


     
    f00 = pData[index];
    f01 = pData[index + 1];

     
    index = (xIndex - 1) + (yIndex) * S->numCols;


     
    f10 = pData[index];
    f11 = pData[index + 1];

     
    b1 = f00;
    b2 = f01 - f00;
    b3 = f10 - f00;
    b4 = f00 - f01 - f10 + f11;

     
    xdiff = X - xIndex;

     
    ydiff = Y - yIndex;

     
    out = b1 + b2 * xdiff + b3 * ydiff + b4 * xdiff * ydiff;

     
    return (out);
  }


  






 
  __attribute__((always_inline)) static __inline q31_t arm_bilinear_interp_q31(
  arm_bilinear_interp_instance_q31 * S,
  q31_t X,
  q31_t Y)
  {
    q31_t out;                                    
    q31_t acc = 0;                                
    q31_t xfract, yfract;                         
    q31_t x1, x2, y1, y2;                         
    int32_t rI, cI;                               
    q31_t *pYData = S->pData;                     
    uint32_t nCols = S->numCols;                  

     
     
     
    rI = ((X & (q31_t)0xFFF00000) >> 20);

     
     
     
    cI = ((Y & (q31_t)0xFFF00000) >> 20);

     
     
    if (rI < 0 || rI > (S->numRows - 1) || cI < 0 || cI > (S->numCols - 1))
    {
      return (0);
    }

     
     
    xfract = (X & 0x000FFFFF) << 11U;

     
    x1 = pYData[(rI) + (int32_t)nCols * (cI)    ];
    x2 = pYData[(rI) + (int32_t)nCols * (cI) + 1];

     
     
    yfract = (Y & 0x000FFFFF) << 11U;

     
    y1 = pYData[(rI) + (int32_t)nCols * (cI + 1)    ];
    y2 = pYData[(rI) + (int32_t)nCols * (cI + 1) + 1];

     
    out = ((q31_t) (((q63_t) x1  * (0x7FFFFFFF - xfract)) >> 32));
    acc = ((q31_t) (((q63_t) out * (0x7FFFFFFF - yfract)) >> 32));

     
    out = ((q31_t) ((q63_t) x2 * (0x7FFFFFFF - yfract) >> 32));
    acc += ((q31_t) ((q63_t) out * (xfract) >> 32));

     
    out = ((q31_t) ((q63_t) y1 * (0x7FFFFFFF - xfract) >> 32));
    acc += ((q31_t) ((q63_t) out * (yfract) >> 32));

     
    out = ((q31_t) ((q63_t) y2 * (xfract) >> 32));
    acc += ((q31_t) ((q63_t) out * (yfract) >> 32));

     
    return ((q31_t)(acc << 2));
  }


  





 
  __attribute__((always_inline)) static __inline q15_t arm_bilinear_interp_q15(
  arm_bilinear_interp_instance_q15 * S,
  q31_t X,
  q31_t Y)
  {
    q63_t acc = 0;                                
    q31_t out;                                    
    q15_t x1, x2, y1, y2;                         
    q31_t xfract, yfract;                         
    int32_t rI, cI;                               
    q15_t *pYData = S->pData;                     
    uint32_t nCols = S->numCols;                  

     
     
     
    rI = ((X & (q31_t)0xFFF00000) >> 20);

     
     
     
    cI = ((Y & (q31_t)0xFFF00000) >> 20);

     
     
    if (rI < 0 || rI > (S->numRows - 1) || cI < 0 || cI > (S->numCols - 1))
    {
      return (0);
    }

     
     
    xfract = (X & 0x000FFFFF);

     
    x1 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI)    ];
    x2 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI) + 1];

     
     
    yfract = (Y & 0x000FFFFF);

     
    y1 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI + 1)    ];
    y2 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI + 1) + 1];

     

     
     
    out = (q31_t) (((q63_t) x1 * (0xFFFFF - xfract)) >> 4U);
    acc = ((q63_t) out * (0xFFFFF - yfract));

     
    out = (q31_t) (((q63_t) x2 * (0xFFFFF - yfract)) >> 4U);
    acc += ((q63_t) out * (xfract));

     
    out = (q31_t) (((q63_t) y1 * (0xFFFFF - xfract)) >> 4U);
    acc += ((q63_t) out * (yfract));

     
    out = (q31_t) (((q63_t) y2 * (xfract)) >> 4U);
    acc += ((q63_t) out * (yfract));

     
     
    return ((q15_t)(acc >> 36));
  }


  





 
  __attribute__((always_inline)) static __inline q7_t arm_bilinear_interp_q7(
  arm_bilinear_interp_instance_q7 * S,
  q31_t X,
  q31_t Y)
  {
    q63_t acc = 0;                                
    q31_t out;                                    
    q31_t xfract, yfract;                         
    q7_t x1, x2, y1, y2;                          
    int32_t rI, cI;                               
    q7_t *pYData = S->pData;                      
    uint32_t nCols = S->numCols;                  

     
     
     
    rI = ((X & (q31_t)0xFFF00000) >> 20);

     
     
     
    cI = ((Y & (q31_t)0xFFF00000) >> 20);

     
     
    if (rI < 0 || rI > (S->numRows - 1) || cI < 0 || cI > (S->numCols - 1))
    {
      return (0);
    }

     
     
    xfract = (X & (q31_t)0x000FFFFF);

     
    x1 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI)    ];
    x2 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI) + 1];

     
     
    yfract = (Y & (q31_t)0x000FFFFF);

     
    y1 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI + 1)    ];
    y2 = pYData[((uint32_t)rI) + nCols * ((uint32_t)cI + 1) + 1];

     
    out = ((x1 * (0xFFFFF - xfract)));
    acc = (((q63_t) out * (0xFFFFF - yfract)));

     
    out = ((x2 * (0xFFFFF - yfract)));
    acc += (((q63_t) out * (xfract)));

     
    out = ((y1 * (0xFFFFF - xfract)));
    acc += (((q63_t) out * (yfract)));

     
    out = ((y2 * (yfract)));
    acc += (((q63_t) out * (xfract)));

     
    return ((q7_t)(acc >> 40));
  }

  

 


 



 



 



 



 



 





   
#line 7056 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

   
#line 7064 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"

   


   


#line 7126 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"






 


#line 7151 "..\\Drivers\\CMSIS\\DSP\\Include\\arm_math.h"






 
#line 6 "..\\..\\dwLaser_Application\\MCU_Application\\deviceInfo\\deviceConfig.h"
 
typedef struct{
	char serialNumber[32];
	char dateOfManufacture[32];
	
	float32_t laserNotesIntercept[5];
	float32_t laserNotesB1[5];
	float32_t laserNotesB2[5];
	float32_t laserNotesB3[5];
	
	uint16_t ldTableCh0[30];
	uint16_t ldTableCh1[30];
	uint16_t ldTableCh2[30];
	uint16_t ldTableCh3[30];
	
	uint16_t pdTable[30];
	uint16_t crc;
}deviceConfig_t;

typedef struct{
	uint32_t bootLoaderCrc;
	uint32_t mucAppCrc;
	uint32_t lcdAppCrc;
}firmwareInfo_t;

typedef struct{
	uint32_t powerUpCycle;
	uint32_t runTime;
	uint32_t laserOnTime[5];
	int16_t laserMaxTemper;
	int16_t laserMaxCurrent[5];
	int16_t laserMaxPhotoDiode;
	int16_t mucMaxTemper;
}deviceLogInfo_t;
 
extern deviceConfig_t deviceConfig;
extern firmwareInfo_t firmwareInfo;
extern deviceLogInfo_t deviceLogInfo;
extern uint32_t	UniqueId[3];
 
extern uint16_t cpuGetFlashSize(void);
extern void readStm32UniqueID(void);




#line 16 "..\\Bootloader\\bootLoader.h"
 
#line 1 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"


 
#line 15 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 29 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 				





 



 
#line 55 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 



 
#line 77 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




#line 88 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 104 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 122 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 


 


 




 




 
#line 155 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"



























#line 194 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"

#line 203 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"






























 


 
#line 257 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 278 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 299 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 320 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 

























































 

 



 



 


 
#line 404 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 411 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 
#line 425 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 
#line 439 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 448 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 459 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"

#line 468 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 489 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 

 





 
#line 522 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 



 
#line 535 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 				





 
#line 554 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"



 
#line 577 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
















































 















































 
#line 706 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"

#line 739 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 749 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 756 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 766 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 788 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"

#line 800 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 
#line 816 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 




 


 
#line 846 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 


 



 


 




 
#line 870 "..\\Bootloader\\..\\..\\dwLaser_Application\\MCU_Application\\sPlc\\sPlcConfig.h"
 







#line 18 "..\\Bootloader\\bootLoader.h"
 
extern uint8_t usbReady;
 
void resetInit(void);
void bootLoadInit(void);
void bootLoadProcess(void);





#line 2 "..\\Bootloader\\bootLoader.c"
#line 1 "..\\Bootloader\\exdac.h"


 
#line 5 "..\\Bootloader\\exdac.h"
 
void initChipDac(void);
void CLDAC(void);



#line 3 "..\\Bootloader\\bootLoader.c"
 




 
#line 23 "..\\Bootloader\\bootLoader.c"
 
#line 44 "..\\Bootloader\\bootLoader.c"


 


 
#line 61 "..\\Bootloader\\bootLoader.c"
 



 


































 
typedef enum {
	CLEAR_EPROM_ALL 			= 0x01,
	CLEAR_EPROM_NVRAM			= 0x02,
	CLEAR_EPROM_FDRAM			= 0x03,
	CLEAR_EPROM_FIRMWARE_INFO	= 0x04,
	CLEAR_EPROM_DEVICE_CONFIG	= 0x05,
	CLEAR_EPROM_LOG_INFO		= 0x06,
}clarmEpromCmd_t;
 
uint32_t TmpReadSize = 0x00;
uint32_t RamAddress = 0x00;
static volatile uint32_t LastPGAddress = ((uint32_t)0x08010000);
uint8_t RAM_Buf[((uint16_t)512*64)] = {0x00};
 
uint8_t gddcRxBuf[64];
uint8_t gddcTxBuf[(2048 + 4)];
 
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern USBH_HandleTypeDef hUsbHostFS;
 
FRESULT retUsbH;
FATFS	USBH_fatfs;
 
FIL LogFile;
FIL CfgFile;
FIL McuFile;
FIL LcdFile;
FIL BotFile;
FIL SepromFile;
FIL LepromFile;
 
DIR	FileDir;
FILINFO FileInfo;
static uint8_t bootLoadState;
uint8_t usbReady;
int32_t releaseTime0, releaseTime1, overTime, releaseCounter;
uint32_t JumpAddress;
pFunction Jump_To_Application;
static uint32_t oldcrc32;
const uint32_t crc32Tab[] = {  
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
static uint8_t BID;
 
static void bootLoadFailHandler(uint8_t ftype);
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
static void clearEprom(clarmEpromCmd_t cmd);
static void updateEprom(void);
static void dumpEprom(void);
 
static uint32_t crc32Calculate(uint8_t *buf, uint32_t len);
static uint32_t crc32CalculateAdd(uint8_t dat);
static void crc32Clear(void);
static void crc32SetCrcOld(uint32_t old);
 
static void softDelayMs(uint16_t ms);
 
static HAL_StatusTypeDef epromReadByte(uint16_t ReadAddr, uint8_t *rdat);
static HAL_StatusTypeDef epromWriteByte(uint16_t WriteAddr, uint8_t wdat);
static HAL_StatusTypeDef epromRead(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead);
static HAL_StatusTypeDef epromWrite(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);
 

static void UsbGpioReset(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	 
	do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) |= ((0x1UL << (0U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) & ((0x1UL << (0U)))); (void)tmpreg; } while(0U);
	 
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x1000), GPIO_PIN_RESET);
	 
	GPIO_InitStruct.Pin = ((uint16_t)0x1000);
	GPIO_InitStruct.Mode = 0x00000001U;
	GPIO_InitStruct.Pull = 0x00000002U;
	GPIO_InitStruct.Speed = 0x00000000U;
	HAL_GPIO_Init(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), &GPIO_InitStruct);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x1000), GPIO_PIN_RESET);                                            
	softDelayMs(100);
	
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x1000), GPIO_PIN_SET);
	softDelayMs(100);
	HAL_GPIO_DeInit(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x1000));
	(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR &= ~((0x1UL << (0U))));
	do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) |= ((0x1UL << (6U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR) & ((0x1UL << (6U)))); (void)tmpreg; } while(0U);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x1800UL)),((uint16_t)0x0100), GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = ((uint16_t)0x0100);
	GPIO_InitStruct.Mode = 0x00000001U;
	GPIO_InitStruct.Pull = 0x00000002U;
	GPIO_InitStruct.Speed = 0x00000000U;
	HAL_GPIO_Init(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x1800UL)), &GPIO_InitStruct);
	softDelayMs(200);
	HAL_GPIO_DeInit(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x1800UL)), ((uint16_t)0x1000));
	(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB1ENR &= ~((0x1UL << (6U))));	
	(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->AHB2ENR &= ~((0x1UL << (7U))));
	HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
	HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
}
static void SystemClock_Reset(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	(*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x70U) * 32U) + (0x10U * 4U)) = DISABLE);
	(*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x70U) * 32U) + (0x10U * 4U)) = ENABLE);
	(*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);
	(*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x00U * 4U)) = DISABLE);
	 
	(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR &= ~((0x1UL << (28U))));
	do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) |= ((0x1UL << (28U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) & ((0x1UL << (28U)))); (void)tmpreg; } while(0U);
	do { volatile uint32_t tmpreg = 0x00U; (((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR)) = ((((((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR))) & (~((0x3UL << (14U))))) | (((0x3UL << (14U))))))); tmpreg = ((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) & ((0x3UL << (14U)))); (void)tmpreg; } while(0U);
	 
	RCC_OscInitStruct.OscillatorType = 0x00000002U;
	RCC_OscInitStruct.HSIState = ((uint8_t)0x01);
	RCC_OscInitStruct.HSICalibrationValue = 0x10U;
	RCC_OscInitStruct.PLL.PLLState = ((uint8_t)0x00);
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
		Error_Handler();
	}
	 
	RCC_ClkInitStruct.ClockType = 0x00000002U | 0x00000001U | 0x00000004U | 0x00000008U;
	RCC_ClkInitStruct.SYSCLKSource = 0x00000000U;
	RCC_ClkInitStruct.AHBCLKDivider = 0x00000000U;
	RCC_ClkInitStruct.APB1CLKDivider = 0x00001000U;
	RCC_ClkInitStruct.APB2CLKDivider = 0x00000000U;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 0x00000000U) != HAL_OK){
		Error_Handler();
	}
}
void resetInit(void){
	HAL_DeInit();
	
	SystemClock_Reset();
	UsbGpioReset();
	__enable_irq();
}
 

static uint32_t crc32Calculate(uint8_t *buf, uint32_t len){
    uint32_t i;  
    for (i = 0; i < len; i++){  
       oldcrc32 = crc32Tab[(oldcrc32 ^ buf[i]) & 0xff] ^ (oldcrc32 >> 8);  
    }  
	return (oldcrc32 ^ 0xFFFFFFFF);  
}
static uint32_t crc32CalculateAdd(uint8_t dat){
	oldcrc32 = crc32Tab[(oldcrc32 ^ dat) & 0xff] ^ (oldcrc32 >> 8);
	return (oldcrc32 ^ 0xFFFFFFFF);
}
static void crc32Clear(void){
	oldcrc32 = 0xFFFFFFFF;
}
static void crc32SetCrcOld(uint32_t old){
	oldcrc32 = old;
}
static void softDelayMs(uint16_t ms){
	uint32_t i;
	for(i = 0;i < 1000;i ++){
		__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	}
}
 
void bootLoadInit(void){
	BID = 0;
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0001), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0020), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0C00UL)), ((uint16_t)0x0004), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0100), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x4000), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x8000), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
	initChipDac();

	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0010), GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
	HAL_Delay(200);
	
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_SET);
	HAL_Delay(200);
	
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_SET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_SET);
	
	overTime = HAL_GetTick() + 4000;
	releaseTime0 = 0;
	releaseTime1 = 0;
	usbReady = 0U;
	bootLoadState = 0; 
	printf("\r\n");
	printf("\r\n");
	printf("\r\n");   
	
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0002)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->SYS_ID0       = HIGH!\n");
		BID |= (1 << 0);
	}
	else{
		printf("Bootloader:INPUT->SYS_ID0       = LOW!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0004)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->SYS_ID1       = HIGH!\n");
		BID |= (1 << 1);
	}
	else{
		printf("Bootloader:INPUT->SYS_ID1       = LOW!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0400)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->SYS_ID2       = HIGH!\n");
		BID |= (1 << 2);
	}
	else{
		printf("Bootloader:INPUT->SYS_ID2       = LOW!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x8000)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->FSWITCH_NC    = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->FSWITCH_NC    = Close!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0400)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->FSWITCH_NO    = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->FSWITCH_NO    = Close!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0800)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->ESTOP         = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->ESTOP         = Close!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x1000)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->INTLOCK       = Open!\n");
	}
	else{
		printf("Bootloader:INPUT->INTLOCK       = Close!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x0010)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->PM_ALARM      = High!\n");
	}
	else{
		printf("Bootloader:INPUT->PM_ALARM      = Low!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0010)) == GPIO_PIN_SET){
		printf("Bootloader:INPUT->SOFTPOWER_IN 	= Open!\n");
	}
	else{
		printf("Bootloader:INPUT->SOFTPOWER_IN	= Close!\n");
	}
	
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x0080)) == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->LP_PWM     	= High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->LP_PWM     	= Low!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0C00UL)), ((uint16_t)0x0004)) == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->FAN5V_OUT    = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->FAN5V_OUT    = Low!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0100)) == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->FAN24V_OUT   = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->FAN24V_OUT   = Low!\n");
	}
	if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x8000)) == GPIO_PIN_SET){
		printf("Bootloader:OUTPUT->TEC_OUT      = High!\n");
	}
	else{
		printf("Bootloader:OUTPUT->TEC_OUT      = Low!\n");
	}
	HAL_Delay(10);
}
void bootLoadProcess(void){
	HAL_StatusTypeDef ret;
	uint8_t fileBuff[64];
	uint32_t crcFlash, crcUdisk;
	uint32_t brByte;
	
	
	switch(bootLoadState){
		case 0:{
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_SET);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			printf("Bootloader:Start...............\n");
			readStm32UniqueID();
			printf("Bootloader:Board ID->0x%X\n", BID);
			printf("Bootloader:UniqueID->0x%08X%08X%08X\n", UniqueId[0], UniqueId[1], UniqueId[2]);
			printf("Bootloader:Mcu flash size->%d Kbytes\n", cpuGetFlashSize());
			printf("Bootloader:Ver->0x%08X Build->%s:%s\n", 0x00010001, "Aug  4 2021", "21:13:00");
			if(HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x1000)) == GPIO_PIN_SET &&
			   HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0800)) == GPIO_PIN_SET &&
			   HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0000UL)), ((uint16_t)0x8000)) == GPIO_PIN_RESET &&
			   HAL_GPIO_ReadPin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0400)) == GPIO_PIN_RESET){
				bootLoadState = 1;
			}
			else{
				bootLoadState = 99;
			}
			break;
		}
		case 1:{
			ret = epromRead((7424L), (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
			if(ret == HAL_OK){
				bootLoadState = 2;
				printf("Bootloader:Load eprom done!\n");
			}
			else{
				bootLoadState = 99;
			}
			break;
		}
		case 2:{
			retUsbH = f_mount(&USBH_fatfs, "0:", 0);
			if(retUsbH != FR_OK){
				printf("Bootloader:Mount Fatfs errror:%d!\n", retUsbH);
				bootLoadState = 99;
			}
			else{
				printf("Bootloader:Mount Fatfs sucess!\n");
				bootLoadState = 3;
			}
			break;
		}
		case 3:{
			
			releaseTime0 = (overTime - (int32_t)HAL_GetTick()) / 1000;
			if(releaseTime0 != releaseTime1){
				printf("Bootloader:Wait usb disk init:%d Second!\n", releaseTime0);
				releaseTime1 = releaseTime0;
			} 
			if(releaseTime0 <= 0){
				bootLoadState = 4;
			}
			else if(releaseTime0 <=2 && usbReady == 1U){
				bootLoadState = 4;
			}
			break;
		}
		case 4:{
			
			retUsbH = f_open(&CfgFile, "/las.cfg", 0x00 | 0x01);
			if(retUsbH != FR_OK){
				printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", "/las.cfg", retUsbH);
				bootLoadState = 99;
			}
			else{
				printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", "/las.cfg", retUsbH);
				f_lseek(&CfgFile, 0);
				retUsbH = f_read(&CfgFile, fileBuff, 5, &brByte);
				if((retUsbH != FR_OK) || (brByte < 5)){
					bootLoadFailHandler('0');
				}
				if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '1'){
					if(fileBuff[3] == 'A' && fileBuff[4] == '0'){
						printf("Bootloader:Start upgrade mcu application!\n");
						bootLoadState = 6;
					}
					else{
						bootLoadState = 99;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '2'){
					if(fileBuff[3] == 'A' && fileBuff[4] == '0'){
						printf("Bootloader:Start upgrade lcd application!\n");
						bootLoadState = 7;
					}
					else{
						bootLoadState = 99;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '3'){
					if(fileBuff[3] == 'A' && fileBuff[4] == '0'){
						printf("Bootloader:Start upgrade mcu application & lcd application!\n");
						bootLoadState = 8;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = 99;
					}
				}
				else if(fileBuff[0] == 'U' && fileBuff[1] == '0' && fileBuff[2] == '4'){
					if(fileBuff[3] == 'A' && fileBuff[4] == '0'){
						printf("Bootloader:Start update eprom!\n");
						bootLoadState = 9;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = 99;
					}
				}
				else if(fileBuff[0] == 'D' && fileBuff[1] == '0' && fileBuff[2] == '1'){
					if(fileBuff[3] == 'A' && fileBuff[4] == '0'){
						printf("Bootloader:Start dump eprom to udisk!\n");
						bootLoadState = 10;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = 99;
					}
				}
				else if(fileBuff[0] == 'C' && fileBuff[1] == '0' && fileBuff[2] == '1'){
					if(fileBuff[3] == 'A' && fileBuff[4] == '0'){
						printf("Bootloader:Start clear flash and eprom!\n");
						bootLoadState = 11;
					}
					else{
						printf("Bootloader:Device ID is not mate,run app!\n");
						bootLoadState = 99;
					}
				}
				else{
					bootLoadState = 99;
				}
				f_close(&CfgFile);
			}
			break;
		}
		case 6:{
			crcFlash = getOriginAppCrc();
			crcUdisk = getNewMcuAppCrc();
			if((crcUdisk == firmwareInfo.mucAppCrc) && (crcFlash == firmwareInfo.mucAppCrc)){
				printf("Bootloader:Check mcu app crc same,skip!\n");
				bootLoadState = 90;
				break;
			}
			crcUdisk = updateMcuApp();
			crcFlash = getOriginAppCrc();
			if(crcUdisk != crcFlash){
				bootLoadFailHandler('B');
			}
			else{
				printf("Bootloader:Checksum mcu app sucess.\n");
				firmwareInfo.mucAppCrc = crcFlash;
				epromWrite((7424L), (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
				clearEprom(CLEAR_EPROM_NVRAM);
				printf("Bootloader:Update new crc32 sucess.\n");
			}
			bootLoadState = 90;
			break;
		}
		case 7:{
			crcUdisk = getNewLcdAppCrc();
			if(crcUdisk == firmwareInfo.lcdAppCrc){
				printf("Bootloader:Check lcd app crc same,skip!\n");
				bootLoadState = 99;
				break;
			}
			crcUdisk = updateLcdApp();
			firmwareInfo.lcdAppCrc = crcUdisk;
			epromWrite((7424L), (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
			printf("Bootloader:Update new crc32 sucess.\n");
			bootLoadState = 90;
			break;
		}
		case 8:{
			
			crcFlash = getOriginAppCrc();
			crcUdisk = getNewMcuAppCrc();
			if((crcUdisk == firmwareInfo.mucAppCrc) && (crcFlash == firmwareInfo.mucAppCrc)){
				printf("Bootloader:Check mcu app crc same,skip!\n");
			}
			else{
				crcUdisk = updateMcuApp();	
				crcFlash = getOriginAppCrc();
				if(crcUdisk != crcFlash){
					bootLoadFailHandler('B');
				}
				firmwareInfo.mucAppCrc = crcFlash;
				printf("Bootloader:Check mcu app sucess.\n");
				epromWrite((7424L), (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
				clearEprom(CLEAR_EPROM_NVRAM);
				printf("Bootloader:Update mcu app new crc32 sucess.\n");
			}
			
			crcUdisk = getNewLcdAppCrc();
			if(crcUdisk == firmwareInfo.lcdAppCrc){
				printf("Bootloader:Check lcd app crc same,skip!\n");
			}
			else{
				updateLcdApp();
				firmwareInfo.lcdAppCrc = crcUdisk;
				epromWrite((7424L), (uint8_t*)&firmwareInfo, sizeof(firmwareInfo));
				printf("Bootloader:Update lcd app new crc32 sucess,wait lcd upgrade done\n");
				HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);HAL_Delay(5000);
				HAL_Delay(5000);HAL_Delay(5000);
			}
			bootLoadState = 90;
			break;
		}
		case 9:{
			updateEprom();
			break;
		}
		case 10:{
			dumpEprom();
			break;
		}
		case 11:{
			clearFlash();
			clearEprom(CLEAR_EPROM_ALL);
			bootLoadFailHandler('H');
			break;
		}
		case 90:{
			printf("Bootloader:System Reset\n");
			__set_FAULTMASK(1);
			__NVIC_SystemReset(); 
			break;
		}
		case 99:{
			HAL_FLASH_Lock();
			 
			if((((*(volatile uint32_t *) ((uint32_t)0x08010000)) & 0xFF000000) == 0x20000000) || (((*(volatile uint32_t *) ((uint32_t)0x08010000)) & 0xFF000000) == 0x10000000)){
				 
				printf("Bootloader:Jump application address.\n");
				JumpAddress = *(volatile uint32_t *) (((uint32_t)0x08010000) + 4);
				printf("Bootloader:Jump Address:0x%X\n", JumpAddress);
				Jump_To_Application = (pFunction) JumpAddress;
				 
				__set_MSP(*(volatile uint32_t *) ((uint32_t)0x08010000));
				printf("Bootloader:Stack Pointer:0x%X\n", *(volatile uint32_t *) ((uint32_t)0x08010000));
				printf("\n\n\n\r\r\r");
				
				HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_RESET);
				HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
				HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
				__disable_irq();
				((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;
				
				HAL_NVIC_DisableIRQ(SysTick_IRQn); 
				HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
				HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
				HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
				HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
				HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
				
				
				
				
				
				
				
				
				HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0001), GPIO_PIN_RESET);
				Jump_To_Application();
			}
			bootLoadFailHandler('F');
		}
		default:break;
	}
}
static void beepDiag(uint8_t diag){
	
	switch(diag){
		case '0':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '1':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		};
		case '2':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		};
		case '3':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '4':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '5':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '6':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '7':{	
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '8':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case '9':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'A':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'B':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;			
		}
		case 'C':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'D':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'E':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'F':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'G':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'H':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;			
		}
		case 'I':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'J':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'K':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'L':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(150);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		case 'M':{
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);HAL_Delay(1000);
			
			HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_SET);HAL_Delay(750);HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
			break;
		}
		default:break;
	}
	HAL_Delay(3000);
}
static void bootLoadFailHandler(uint8_t ftype){
	MX_DriverVbusFS(0U);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0800UL)), ((uint16_t)0x0020), GPIO_PIN_RESET);
	printf("Bootloader:SYS_ERR_LED->On!\n");
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0080), GPIO_PIN_RESET);
	HAL_GPIO_WritePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0040), GPIO_PIN_RESET);
	switch(ftype){
		case '0':{
			printf("Bootloader:FailHandler,Read %s fail!.\n", "/las.cfg");
			while(1){
				beepDiag('0');
			};
		}
		case '1':{
			printf("Bootloader:FailHandler,Read %s fail!.\n", "/ld_mcu.bin");
			while(1){
				beepDiag('1');
			};
		}
		case '2':{
			printf("Bootloader:FailHandler,Read %s fail!.\n", "/ld_lcd.pkg");
			while(1){
				beepDiag('2');
			};
		}
		case '5':{
			printf("Bootloader:FailHandler,Erase mcu application fail\n");
			while(1){
				beepDiag('5');
			};
		}
		case '3':{
			printf("Bootloader:FailHandler,Read %s fail!\n", "/leprom.bin");
			while(1){
				beepDiag('3');
			}
		}
		case '4':{
			printf("Bootloader:FailHandler,Write %s fail!\n", "/seprom.bin");
			while(1){
				beepDiag('4');
			}
		}
		case '8':{
			printf("Bootloader:FailHandler,%s size is invalid!\n", "/ld_mcu.bin");
			while(1){
				beepDiag('8');
			};
		}
		case 'A':{
			printf("Bootloader:FailHandler,%s size is invalid!\n", "/ld_lcd.pkg");
			while(1){
				beepDiag('A');
			};
		}
		case 'B':{
			printf("Bootloader:FailHandler,Verify %s fail!.\n", "/ld_mcu.bin");
			while(1){
				beepDiag('B');
			};
		}
		case 'D':{
			printf("Bootloader:FailHandler,LCD is not responsed!.\n");
			while(1){
				beepDiag('D');
			};
		}
		case 'E':{
			printf("Bootloader:FailHandler,LCD download fail!.\n");
			while(1){
				beepDiag('E');
			};
		}
		case 'F':{
			printf("Bootloader:FailHandler,App vector table invalid.\n");
			while(1){
				beepDiag('F');
			};
		}
		case 'G':{
			printf("Bootloader:FailHandler,Flash is not blank!.\n");
			while(1){
				beepDiag('G');
			};
		}
		case 'H':{
			printf("Bootloader:DoneHandler,Flash and Eprom easer done!.\n");
			while(1){
				beepDiag('H');
			}
		}
		case 'I':{
			printf("Bootloader:DoneHandler,Update eprom form udisk done!.\n");
			while(1){
				beepDiag('I');
			}
		}
		case 'J':{
			printf("Bootloader:DoneHandler,Dump eprom to udisk done!\n");
			while(1){
				beepDiag('J');
			}
		}
		default:{
			break;
		}
	}
}
static void clearFlash(void){
	HAL_FLASH_Unlock();
	(((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->SR = ((0x1UL << (16U))|(0x1UL << (0U))|(0x1UL << (7U))|(0x1UL << (4U))));
	if (FLASH_If_EraseApplication() != 0x00){
		bootLoadFailHandler('5');
	}
	checkBlank(((uint32_t)0x08010000), (((uint32_t)0x0817FFFF) - ((uint32_t)0x08010000) + 1));
}
static uint32_t getNewMcuAppCrc(void){
	uint32_t crc32;
	uint32_t i;
	uint8_t readflag = 1U;
	uint16_t bytesread;
	retUsbH = f_open(&McuFile, "/ld_mcu.bin", 0x00 | 0x01);
	if(retUsbH != FR_OK){
		bootLoadFailHandler('1');			
	}
	if(((&McuFile)->obj . objsize) > (((uint32_t)0x0817FFFF) - ((uint32_t)0x08010000) + 1)){
		bootLoadFailHandler('8');
	}
	crc32 = 0;
	crc32Clear();
	while(readflag){
		 
		f_read(&McuFile, RAM_Buf, ((uint16_t)512*64), (void*)&bytesread);
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		 
		TmpReadSize = bytesread;
		 
		if(TmpReadSize < ((uint16_t)512*64)){
			readflag = 0U;
		}
		LastPGAddress += TmpReadSize;
	}
	for(i = LastPGAddress;i < ((uint32_t)0x0817FFFF);i ++){
		crc32 = crc32CalculateAdd(0xFF);
	}
	f_close(&McuFile);
	return crc32;
}
static uint32_t getNewLcdAppCrc(void){
	uint32_t crc32;
	uint8_t readflag = 1U;
	uint16_t bytesread;
	retUsbH = f_open(&LcdFile, "/ld_lcd.pkg", 0x00 | 0x01);
	if(retUsbH != FR_OK){
		bootLoadFailHandler('2');			
	}
	f_lseek(&LcdFile, 0);
	crc32 = 0;
	crc32Clear();
	while(readflag){
		 
		f_read(&LcdFile, RAM_Buf, ((uint16_t)512*64), (void*)&bytesread);
		
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		 
		TmpReadSize = bytesread;
		 
		if(TmpReadSize < ((uint16_t)512*64)){
			readflag = 0U;
		}
		LastPGAddress += TmpReadSize;
	}
	f_close(&LcdFile);
	return crc32;
}
static uint32_t updateMcuApp(void){
	uint32_t crc32;
	uint32_t i;
	uint32_t programcounter = 0x00;
	uint8_t readflag = 1U;
	uint32_t bytesread;
	retUsbH = f_open(&McuFile, "/ld_mcu.bin", 0x00 | 0x01);
	if(retUsbH != FR_OK){
		bootLoadFailHandler('1');			
	}
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", "/ld_mcu.bin", retUsbH);
	if(((&McuFile)->obj . objsize) > (((uint32_t)0x0817FFFF) - ((uint32_t)0x08010000) + 1)){
		bootLoadFailHandler('8');
	}
	HAL_FLASH_Unlock();
	(((FLASH_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3C00UL))->SR = ((0x1UL << (16U))|(0x1UL << (0U))|(0x1UL << (7U))|(0x1UL << (4U))));
	if (FLASH_If_EraseApplication() != 0x00){
		bootLoadFailHandler('5');
	}
	checkBlank(((uint32_t)0x08010000), (((uint32_t)0x0817FFFF) - ((uint32_t)0x08010000) + 1));
	printf("Bootloader:Erase mcu application sucess.\n");
	RamAddress = (uint32_t)&RAM_Buf;
	 
	LastPGAddress = ((uint32_t)0x08010000);
	 
	crc32 = 0;
	crc32Clear();
	while(readflag){
		 
		f_read(&McuFile, RAM_Buf, ((uint16_t)512*64), (void*)&bytesread);
		crc32 = crc32Calculate(RAM_Buf, bytesread);
		 
		TmpReadSize = bytesread;
		 
		if(TmpReadSize < ((uint16_t)512*64)){
			readflag = 0U;
		}
		 
		HAL_GPIO_TogglePin(((GPIO_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x0400UL)), ((uint16_t)0x0020));
		for(programcounter = 0; programcounter < TmpReadSize; programcounter += 4){
			 
			if(FLASH_If_Write((LastPGAddress + programcounter), *(uint32_t *) (RamAddress + programcounter)) != 0x00){
				bootLoadFailHandler('C');
			}
		}
		 
		LastPGAddress += TmpReadSize;
	}
	for(i = LastPGAddress;i < ((uint32_t)0x0817FFFF);i ++){
		crc32 = crc32CalculateAdd(0xFF);
	}
	HAL_FLASH_Lock();
	printf("Bootloader:Write mcu app finish.\n");
	f_close(&McuFile);
	return crc32;
}
static uint32_t updateLcdApp(void){
	UART_HandleTypeDef *puart;
	HAL_StatusTypeDef uRet;
	uint32_t crc32;
	uint8_t baudrateSelect;
	uint8_t signName;
    uint8_t preCmd[] =		   {0x61,0x78,0x72,0x63,0x65,0x6b,0x67,0x64,
					            0x79,0x68,0x74,0x73,0x75,0x6e,0x71,0x77,
								0x70,0x6a,0x62,0x76,0x69,0x66,0x6f,0x6d,
								0x7a,0x6c};
	uint8_t cmd[] = 			{0xEE,0xF1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFC,0xFF,0xFF};        
	uint8_t cmdSnake[] = 		{0xEE,0x04,0xFF,0xFC,0xFF,0xFF};
	uint8_t cmdSnakeBack[] = 	{0xEE,0x55,0xFF,0xFC,0xFF,0xFF};
	uint32_t bufIndex;
	uint32_t baudrateTable[]={9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
	uint32_t fileSize;
	uint32_t blockSize = 2048;
	uint32_t transferByte;
	uint32_t actualByte;
	uint32_t upSpeedBaudrate = 115200;
	uint32_t fileIndex;
	uint16_t checkSum;
	uint8_t lcdRetry;
	puart = &huart5;
	retUsbH = f_open(&LcdFile, "/ld_lcd.pkg", 0x00 | 0x01);
	if(retUsbH != FR_OK){
		bootLoadFailHandler('2');
	}
	f_lseek(&LcdFile, 0);
	printf("Bootloader:Open %s sucess,ECODE=0x%02XH.\n", "/ld_lcd.pkg", retUsbH);
	for(baudrateSelect = 0; baudrateSelect < (sizeof(baudrateTable) / 4); baudrateSelect ++){
		if(baudrateSelect >= (sizeof(baudrateTable) / 4)){
			bootLoadFailHandler('D');
		}
		((puart)->Instance ->CR1 &= ~(0x1UL << (13U)));
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
		puart->Init.BaudRate = baudrateTable[baudrateSelect];
		puart->Init.WordLength = 0x00000000U;
		puart->Init.StopBits = 0x00000000U;
		puart->Init.Parity = 0x00000000U;
		puart->Init.Mode = ((uint32_t)((0x1UL << (3U)) | (0x1UL << (2U))));
		puart->Init.HwFlowCtl = 0x00000000U;
		puart->Init.OverSampling = 0x00000000U;
		if(HAL_UART_Init(puart) != HAL_OK){
			Error_Handler();
		}
		((puart)->Instance ->CR1 |= (0x1UL << (13U)));
		memset(gddcRxBuf, 0x0, 64);
		printf("Bootloader->updateLcdApp:Try lcd serial baudrate %d,send cmdSnake.\n", baudrateTable[baudrateSelect]);
		dp_display_text_num(cmdSnake, 6);
		uRet = HAL_UART_Receive(puart, gddcRxBuf, 6, 2000);
		if(strcmp((char *)cmdSnakeBack, (char *)gddcRxBuf) == 0){
			printf("Bootloader->updateLcdApp:Received cmdSnakeBack,set lcd serial baudrate %d.\n", baudrateTable[baudrateSelect]);
			break;
		}
	}
	dp_display_text_num(preCmd, sizeof(preCmd));
    fileSize =  ((&LcdFile)->obj . objsize);
    
	cmd[2] = (fileSize >> 24) & 0xff;
	cmd[3] = (fileSize >> 16) & 0xff;
	cmd[4] = (fileSize >>  8) & 0xff;
	cmd[5] = (fileSize) & 0xff;    
	cmd[6] = (baudrateTable[baudrateSelect] >> 24) & 0xff;
	cmd[7] = (baudrateTable[baudrateSelect] >> 16) & 0xff;
	cmd[8] = (baudrateTable[baudrateSelect] >>  8) & 0xff;
	cmd[9] = (baudrateTable[baudrateSelect]) & 0xff;    
	
	cmd[10] = cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6] + cmd[7] + cmd[8] + cmd[9];
    
	memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));
	printf("Bootloader->updateLcdApp:Send filesize and up speed baudrate.\n");
	dp_display_text_num(cmd, 15);
	uRet = HAL_UART_Receive(puart, gddcRxBuf, 1, 10000);
	if(uRet != HAL_OK || gddcRxBuf[0] != 0xAA){
		bootLoadFailHandler('D');
	}
	printf("Bootloader->updateLcdApp:Set up baudrate done.\n");
	if(baudrateTable[baudrateSelect] != upSpeedBaudrate){
		
		printf("Bootloader->updateLcdApp:Set lcd serial up baudrate %d.\n", upSpeedBaudrate);
		((puart)->Instance ->CR1 &= ~(0x1UL << (13U)));
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
		huart5 .Init.BaudRate = upSpeedBaudrate;
		huart5 .Init.WordLength = 0x00000000U;
		huart5 .Init.StopBits = 0x00000000U;
		huart5 .Init.Parity = 0x00000000U;
		huart5 .Init.Mode = ((uint32_t)((0x1UL << (3U)) | (0x1UL << (2U))));
		huart5 .Init.HwFlowCtl = 0x00000000U;
		huart5 .Init.OverSampling = 0x00000000U;
		if(HAL_UART_Init(puart) != HAL_OK){
			Error_Handler();
		}
		((puart)->Instance ->CR1 |= (0x1UL << (13U)));
	}
	HAL_Delay(200);
    
    
    
    
    
    
	signName = 0;
	crc32 = 0;
	crc32Clear();
	for(fileIndex = 0; fileIndex < fileSize; fileIndex += blockSize){
		memset(gddcTxBuf, 0x0, sizeof(gddcTxBuf));  
		gddcTxBuf[0] = signName;
		gddcTxBuf[1] = ~signName;
        
        transferByte = blockSize;
		if(fileIndex + transferByte > fileSize){
			transferByte = fileSize - fileIndex;
		}
		retUsbH = f_read(&LcdFile, &gddcTxBuf[2], transferByte, &actualByte);
		crc32 = crc32Calculate(&gddcTxBuf[2], actualByte);
		if(retUsbH != FR_OK){
			bootLoadFailHandler('2');
        }
        
		checkSum = 0x0;
		bufIndex = 0;
		do{
			checkSum += (uint16_t)gddcTxBuf[bufIndex];
			bufIndex ++;
		}while(bufIndex < 2050);
        
		checkSum = (uint16_t)~(checkSum);
		gddcTxBuf[2050] = (checkSum >> 8) & 0xFF;
		gddcTxBuf[2051] = (checkSum) & 0xFF;   
		do{
			lcdRetry ++;
			
			memset(gddcRxBuf, 0x0, sizeof(gddcRxBuf));
			printf("Bootloader->updateLcdApp:Send file block at 0x%08XH,", fileIndex);
			dp_display_text_num(gddcTxBuf, (blockSize + 4));
			uRet = HAL_UART_Receive(puart, gddcRxBuf, 2, 1000);
			if(uRet == HAL_OK){
				if(gddcRxBuf[1] == (uint8_t)(~(signName + 1)) || gddcRxBuf[0] == (signName+1)){
					signName = signName + 1;
					printf("ok!\n");
					break;
				}
				else{
					printf("SignName is not invalid\n");
				}
			}
			else{
				printf("fail!\n");
				HAL_Delay(100);
				if(lcdRetry > 10){
					bootLoadFailHandler('E');
				}
			}
		}while(1);
	}
	f_close(&LcdFile);
	return crc32;
}
static void updateEprom(void){
	HAL_StatusTypeDef ret;
	uint32_t brByte;
	retUsbH = f_open(&LepromFile, "/leprom.bin", 0x00 | 0x01);
	if(retUsbH != FR_OK){
		printf("BootLoader:Open %s fail,ECODE=0x%02XH\n", "/leprom.bin", retUsbH);
				bootLoadFailHandler('3');
	}
	else{
		printf("BootLoader:Open %s sucess,ECODE=0x%02XH\n", "/leprom.bin", retUsbH);
		f_lseek(&LepromFile, 0);
		retUsbH = f_read(&LepromFile, RAM_Buf, 32768, &brByte);
		if((retUsbH != FR_OK) || (brByte !=  32768)){
			bootLoadFailHandler('3');
		}
		f_close(&LepromFile);
		ret = epromWrite(0, RAM_Buf, 32768);
		if(ret != HAL_OK){
			bootLoadFailHandler('7');
		}
		bootLoadFailHandler('I');
	}
}
static void dumpEprom(void){
	uint32_t wrByte;
	epromRead(0x0, RAM_Buf, 32768);
	retUsbH = f_open(&SepromFile, "/leprom.bin", 0x08 | 0x02);
	if(retUsbH != FR_OK){
		bootLoadFailHandler('4');
	}
	retUsbH = f_write(&SepromFile, RAM_Buf, 32768, &wrByte);
	if(retUsbH != FR_OK){
		bootLoadFailHandler('4');
	}
	bootLoadFailHandler('J');
}
static uint32_t getOriginAppCrc(void){
	uint8_t val;
	uint32_t i;
	uint32_t crc32;
	crc32Clear();
	for(i = ((uint32_t)0x08010000);i < ((uint32_t)0x0817FFFF);i ++){
		val = *(volatile uint8_t*)(i);
		crc32 = crc32CalculateAdd(val);
	}
	return crc32;	
}
 

static void DBGU_Printk(uint8_t *buffer){
	while(*buffer != '\0'){
		HAL_UART_Transmit(&huart5, buffer, 1, 0xFFFF);
		buffer ++;
    }
}
static void DBGU_Printk_num(uint8_t *buffer, uint16_t datanum){
    while(datanum != 0){
		HAL_UART_Transmit(&huart5, buffer, 1, 0xFFFF);
		buffer ++;
        datanum--;
    }
}
static void dp_display_text(uint8_t *text){
     
    DBGU_Printk(text);
}
static void dp_display_text_num(uint8_t *text,uint16_t datanum){
     
    DBGU_Printk_num(text, datanum);
}
static void dp_display_value(uint32_t value, int descriptive){
     
    uint8_t print_buf[10];
    if (descriptive == 0){
        sprintf((char *)print_buf, "%lX", value);
        DBGU_Printk(print_buf);
    }
    else if(descriptive == 1){
        sprintf((char *)print_buf, "%ld", value);
        DBGU_Printk(print_buf);
    }
    else if(descriptive == 2){
        sprintf((char *)print_buf, "%c", (uint8_t)value);
        DBGU_Printk(print_buf);
    }
}
static void dp_display_array(uint8_t *value, int bytes, int descriptive){
     
    uint8_t print_buf[10];
    int i;
    for(i=0; i < bytes; i++){
        if (descriptive == 0){
            sprintf((char *)print_buf, "%lX", (uint8_t)value[i]);
            DBGU_Printk(print_buf);
        }
        else if(descriptive == 1){
            sprintf((char *)print_buf, "%ld", (uint8_t)value[i]);
            DBGU_Printk(print_buf);
        }
        else if(descriptive == 2){
            sprintf((char *)print_buf, "%c", (uint8_t)value[i]);
            DBGU_Printk(print_buf);
        }
    }
}
 

static void checkBlank(uint32_t adr, uint32_t size){
	uint8_t val;
	uint32_t i;
	for(i = 0;i < size;i ++){
		val = *(volatile uint8_t*)(adr + i);
		if(val != 0xFF){
			bootLoadFailHandler('G');
		}
	}
}
 

static HAL_StatusTypeDef epromReadByte(uint16_t ReadAddr, uint8_t *rdat){


	HAL_StatusTypeDef ret;
	if(ReadAddr > (32768 - 1)){
		ret = HAL_ERROR;
		return ret;
	}	
	ret = HAL_I2C_Mem_Read(&hi2c1,
	                       0xA1,
	                       ReadAddr,
	                       0x00000010U,
	                       (uint8_t*)(rdat),
	                       1,
	                       1000);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);
		ret = HAL_I2C_Init(&hi2c1);
		printf("sPlc->sPlcEprom:Eprom read byte fail!\n");
	}
	return ret;
}
static HAL_StatusTypeDef epromWriteByte(uint16_t WriteAddr, uint8_t wdat){


	HAL_StatusTypeDef ret;
	if(WriteAddr > (32768 - 1)){
		ret = HAL_ERROR;
		return ret;
	}
	ret = HAL_I2C_Mem_Write(&hi2c1, 
	                        0xA0,
	                        WriteAddr, 
	                        0x00000010U, 
	                        &wdat, 
	                        1, 
	                        1000);
	if(ret != HAL_OK){
		ret = HAL_I2C_DeInit(&hi2c1);
		ret = HAL_I2C_Init(&hi2c1);
		printf("sPlc->sPlcEprom:Eprom write byte fail!\n");
	}
	return ret;
}
static HAL_StatusTypeDef epromRead(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead){



	HAL_StatusTypeDef ret;
	uint16_t rAddr, rBlock, rByte, doBlock;
	uint8_t* rBuffer;
	if(ReadAddr + NumToRead >= (32768 - 1)){
		ret = HAL_ERROR;
		return ret;
	}
	rBlock = NumToRead / 0x08;
	rByte = NumToRead % 0x08;
	rAddr = ReadAddr;
	rBuffer = pBuffer;
	for(doBlock = 0;doBlock < rBlock;doBlock ++){
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, rAddr, 0x00000010U, rBuffer, 0x08, 1000);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);
			ret = HAL_I2C_Init(&hi2c1);
			printf("sPlc->sPlcEprom:Eprom read block fail!\n");
		}
		rAddr += 0x08;
		rBuffer += 0x08;
	}
	if(rByte != 0x0){
		ret = HAL_I2C_Mem_Read(&hi2c1, 0xA1, rAddr, 0x00000010U, rBuffer, rByte ,1000);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);        
			ret = HAL_I2C_Init(&hi2c1);          
		}
		printf("sPlc->sPlcEprom:Eprom read rbyte fail\n");
	}
	return ret;	
}  
static HAL_StatusTypeDef epromWrite(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite){



	HAL_StatusTypeDef ret;
	uint16_t wAddr, wBlock, wByte, doBlock;
	uint8_t* wBuffer;
	if(WriteAddr + NumToWrite >= (32768 - 1)){
		ret = HAL_ERROR;
		return ret;
	}
	wBlock = NumToWrite / 0x08;
	wByte = NumToWrite % 0x08;
	wAddr = WriteAddr;
	wBuffer = pBuffer;
	for(doBlock = 0;doBlock < wBlock;doBlock ++){
		ret = HAL_I2C_Mem_Write(&hi2c1, 0xA0, wAddr, 0x00000010U, wBuffer, 0x08, 1000);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);        
			ret = HAL_I2C_Init(&hi2c1);          
			printf("sPlc->sPlcEprom:Eprom write block fail!\n");	
		}
		wAddr += 0x08;
		wBuffer += 0x08;



	}
	if(wByte != 0x0){
		ret = HAL_I2C_Mem_Write(&hi2c1, 0xA0, wAddr, 0x00000010U, wBuffer, wByte, 1000);
		if(ret != HAL_OK){
			ret = HAL_I2C_DeInit(&hi2c1);        
			ret = HAL_I2C_Init(&hi2c1);          
			printf("sPlc->sPlcEprom:Eprom write remain byte fail!\n");
		}
	}



	return ret;
}
static void clearEprom(clarmEpromCmd_t cmd){
	uint8_t var = 0;
	uint32_t i;	
	switch(cmd){
		case CLEAR_EPROM_ALL:{
			for(i = 0;i < 32768;i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase all eprom sucess!\n");
			break;
		}
		case CLEAR_EPROM_FIRMWARE_INFO:{
			for(i = (7424L);i <= (7551L);i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom firmware info sucess!\n");
			break;
		}
		case CLEAR_EPROM_DEVICE_CONFIG:{
			for(i = (7552L);i <= (8063L);i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom device info sucess!\n");
			break;
		}
		case CLEAR_EPROM_LOG_INFO:{
			for(i = (8064L);i <= (8191L);i ++){
				epromWriteByte(i, var);
			}
			printf("Bootloader->:Erase eprom log info sucess!\n");
			break;
		}
		default:break;
	}
}




