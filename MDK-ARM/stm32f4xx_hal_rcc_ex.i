#line 1 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"





















 

 
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







 



 
 
 


 


 
 


 


 
 
 


 



  
  






 
#line 26 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"



 




 



 
 


 


 
 
 
 
 


 



















 

#line 852 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"

#line 1262 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"















 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;

  uint32_t plli2sq = 0U;

  uint32_t plli2sused = 0U;

   
  ((void)0U);

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000001U) == (0x00000001U))
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x3UL << (25U))))) | ((PeriphClkInit->I2sApb1ClockSelection))))));
     
    if(PeriphClkInit->I2sApb1ClockSelection == 0x00000000U)
    {
      plli2sused = 1U;
    }
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000002U) == (0x00000002U))
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x3UL << (27U))))) | ((PeriphClkInit->I2sApb2ClockSelection))))));
     
    if(PeriphClkInit->I2sApb2ClockSelection == 0x00000000U)
    {
      plli2sused = 1U;
    }
  }
   


   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00002000U) == (0x00002000U))
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x3UL << (20U))))) | ((PeriphClkInit->SaiAClockSelection))))));
     
    if(PeriphClkInit->SaiAClockSelection == 0x00000000U)
    {
      plli2sused = 1U;
    }
     
    if(PeriphClkInit->SaiAClockSelection == ((uint32_t)(0x2UL << (20U))))
    {
       
      ((void)0U);

       
      ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1FUL << (8U))))) | (((PeriphClkInit->PLLDivR)-1U)<<8U)))));
    }
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00004000U) == (0x00004000U))
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x3UL << (22U))))) | ((PeriphClkInit->SaiBClockSelection))))));
     
    if(PeriphClkInit->SaiBClockSelection == 0x00000000U)
    {
      plli2sused = 1U;
    }
     
    if(PeriphClkInit->SaiBClockSelection == ((uint32_t)(0x2UL << (22U))))
    {
       
      ((void)0U);

       
      ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1FUL << (8U))))) | (((PeriphClkInit->PLLDivR)-1U)<<8U)))));
    }
  }
   


   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000008U) == (0x00000008U))
  {
     
    ((void)0U);

     
    do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) |= ((0x1UL << (28U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) & ((0x1UL << (28U)))); (void)tmpreg; } while(0U);

     
    ((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR |= (0x1UL << (8U));

     
    tickstart = HAL_GetTick();

    while((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR & (0x1UL << (8U))) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > 2U)
      {
        return HAL_TIMEOUT;
      }
    }
     
    tmpreg1 = (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR & (0x3UL << (8U)));
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & (0x3UL << (8U)))))
    {
       
      tmpreg1 = (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR & ~((0x3UL << (8U))));
       
      (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x70U) * 32U) + (0x10U * 4U)) = ENABLE);
      (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x70U) * 32U) + (0x10U * 4U)) = DISABLE);
       
      ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR = tmpreg1;

       
      if((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))))
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    do { (((PeriphClkInit->RTCClockSelection) & (0x3UL << (8U))) == (0x3UL << (8U))) ? (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR))) & (~((0x1FUL << (16U))))) | (((PeriphClkInit->RTCClockSelection) & 0xFFFFCFFU))))) : ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR) &= ~((0x1FUL << (16U)))); ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR |= ((PeriphClkInit->RTCClockSelection) & 0x00000FFFU); } while(0U);
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000004U) == (0x00000004U))
  {
     
    (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x8CU) * 32U) + (0x18U * 4U)) = (PeriphClkInit->TIMPresSelection));
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000010U) == 0x00000010U)
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2))) & (~((0x3UL << (22U))))) | ((uint32_t)(PeriphClkInit->Fmpi2c1ClockSelection))))));
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000020U) == 0x00000020U)
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2))) & (~((0x1UL << (27U))))) | ((uint32_t)(PeriphClkInit->Clk48ClockSelection))))));

     
    if(PeriphClkInit->Clk48ClockSelection == ((uint32_t)(0x1UL << (27U))))
    {
      plli2sused = 1U;
    }
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000040U) == 0x00000040U)
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2))) & (~((0x1UL << (28U))))) | ((uint32_t)(PeriphClkInit->SdioClockSelection))))));
  }
   

   
  
 
  if((plli2sused == 1U) || (PeriphClkInit->PeriphClockSelection == 0x00000080U))
  {
     
    (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x1AU * 4U)) = DISABLE);
     
    tickstart = HAL_GetTick();
     
    while((((((((((uint8_t)0x3B)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x3B)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x3B)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x3B)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > 2U)
      {
         
        return HAL_TIMEOUT;
      }
    }

     
    ((void)0U);
    ((void)0U);
    ((void)0U);
     
    (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x84U) * 32U) + (0x16U * 4U)) = (PeriphClkInit->PLLI2SSelection));

     
    if(((((PeriphClkInit->PeriphClockSelection) & 0x00000001U) == 0x00000001U) && (PeriphClkInit->I2sApb1ClockSelection == 0x00000000U)) ||
       ((((PeriphClkInit->PeriphClockSelection) & 0x00000002U) == 0x00000002U) && (PeriphClkInit->I2sApb2ClockSelection == 0x00000000U)) ||
       ((((PeriphClkInit->PeriphClockSelection) & 0x00000020U) == 0x00000020U) && (PeriphClkInit->Clk48ClockSelection == ((uint32_t)(0x1UL << (27U))))) ||
       ((((PeriphClkInit->PeriphClockSelection) & 0x00000040U) == 0x00000040U) && (PeriphClkInit->SdioClockSelection == 0x00000000U) && (PeriphClkInit->Clk48ClockSelection == ((uint32_t)(0x1UL << (27U))))))
    {
       
      ((void)0U);
      ((void)0U);

       
       
       
      (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR = ((PeriphClkInit->PLLI2S . PLLI2SM) | ((PeriphClkInit->PLLI2S . PLLI2SN) << (6U)) | ((PeriphClkInit->PLLI2S . PLLI2SQ) << (24U)) | ((PeriphClkInit->PLLI2S . PLLI2SR) << (28U))));
    }


     
    if(((((PeriphClkInit->PeriphClockSelection) & 0x00002000U) == 0x00002000U) && (PeriphClkInit->SaiAClockSelection == 0x00000000U)) ||
       ((((PeriphClkInit->PeriphClockSelection) & 0x00004000U) == 0x00004000U) && (PeriphClkInit->SaiBClockSelection == 0x00000000U)))
    {
       
      ((void)0U);
       
      ((void)0U);

       
      plli2sq = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0xFUL << (24U))) >> (24U));
       
       
       
       
      (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR = ((PeriphClkInit->PLLI2S . PLLI2SM) | ((PeriphClkInit->PLLI2S . PLLI2SN) << (6U)) | ((plli2sq) << (24U)) | ((PeriphClkInit->PLLI2S . PLLI2SR) << (28U))));

       
      ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1FUL << (0U))))) | ((PeriphClkInit->PLLI2SDivR)-1U)))));
    }


     
    if((PeriphClkInit->PeriphClockSelection & 0x00000080U) == 0x00000080U)
    {
       
      ((void)0U);
      ((void)0U);

       
       
       
      (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR = ((PeriphClkInit->PLLI2S . PLLI2SM) | ((PeriphClkInit->PLLI2S . PLLI2SN) << (6U)) | ((PeriphClkInit->PLLI2S . PLLI2SQ) << (24U)) | ((PeriphClkInit->PLLI2S . PLLI2SR) << (28U))));
    }

     
    (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x1AU * 4U)) = ENABLE);
     
    tickstart = HAL_GetTick();
     
    while((((((((((uint8_t)0x3B)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x3B)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x3B)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x3B)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > 2U)
      {
         
        return HAL_TIMEOUT;
      }
    }
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000100U) == 0x00000100U)
  {
     
    ((void)0U);

     
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1UL << (31U))))) | ((PeriphClkInit->Dfsdm1ClockSelection)))));
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000200U) == 0x00000200U)
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1UL << (15U))))) | ((PeriphClkInit->Dfsdm1AudioClockSelection))))));
  }
   


   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000400U) == 0x00000400U)
  {
     
    ((void)0U);

     
    (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1UL << (31U))))) | ((PeriphClkInit->Dfsdm2ClockSelection)))));
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00000800U) == 0x00000800U)
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR))) & (~((0x1UL << (14U))))) | ((PeriphClkInit->Dfsdm2AudioClockSelection))))));
  }
   

   
  if(((PeriphClkInit->PeriphClockSelection) & 0x00001000U) == 0x00001000U)
  {
     
    ((void)0U);

     
    ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2))) & (~((0x3UL << (30U))))) | ((uint32_t)(PeriphClkInit->Lptim1ClockSelection))))));
  }
   


  return HAL_OK;
}







 
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

   

  PeriphClkInit->PeriphClockSelection = 0x00000001U     | 0x00000002U |                                        0x00000004U          | 0x00000008U      |                                        0x00000010U      | 0x00000020U    |                                        0x00000040U         | 0x00000100U   |                                        0x00000200U | 0x00000400U   |                                        0x00000800U | 0x00001000U   |                                        0x00002000U         | 0x00004000U;
#line 1656 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"



   
  PeriphClkInit->PLLI2S.PLLI2SM = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))) >> (0U));
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x1FFUL << (6U))) >> (6U));
  PeriphClkInit->PLLI2S.PLLI2SQ = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0xFUL << (24U))) >> (24U));
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x7UL << (28U))) >> (28U));

   
  PeriphClkInit->PLLI2SDivR = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR & (0x1FUL << (0U))) >> (0U));
  PeriphClkInit->PLLDivR = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR & (0x1FUL << (8U))) >> (8U));


   
  PeriphClkInit->I2sApb1ClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x3UL << (25U)))));

   
  PeriphClkInit->I2sApb2ClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x3UL << (27U)))));

   
  tempreg = (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x1FUL << (16U)));
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR & (0x3UL << (8U))));

   
  PeriphClkInit->Fmpi2c1ClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2) & ((0x3UL << (22U)))));

   
  PeriphClkInit->Clk48ClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2) & ((0x1UL << (27U)))));

   
  PeriphClkInit->SdioClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2) & ((0x1UL << (28U)))));

   
  PeriphClkInit->Dfsdm1ClockSelection = ((uint32_t)(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x1UL << (31U))))));

   
  PeriphClkInit->Dfsdm1AudioClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x1UL << (15U)))));


   
  PeriphClkInit->Dfsdm2ClockSelection = ((uint32_t)(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x1UL << (31U))))));

   
  PeriphClkInit->Dfsdm2AudioClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x1UL << (14U)))));

   
  PeriphClkInit->Lptim1ClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR2) & ((0x3UL << (30U)))));

   
  PeriphClkInit->SaiAClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x3UL << (20U)))));

   
  PeriphClkInit->SaiBClockSelection = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x3UL << (22U)))));


   
  if ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR & (0x1UL << (24U))) == RESET)
  {
    PeriphClkInit->TIMPresSelection = ((uint8_t)0x00);
  }
  else
  {
    PeriphClkInit->TIMPresSelection = ((uint8_t)0x01);
  }
}









 
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
   
  uint32_t frequency = 0U;
   
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
   
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case 0x00000001U:
    {
       
      srcclk = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x3UL << (25U)))));
      switch (srcclk)
      {
       
      case ((uint32_t)(0x1UL << (25U))):
        {
           
          frequency = ((uint32_t)12288000U);
          break;
        }
       
      case 0x00000000U:
        {
          if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
          {
             
            vcoinput = (uint32_t)(((uint32_t)12288000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))));
          }
          else
          {
             
             
            if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
            {
               
              vcoinput = (uint32_t)(((uint32_t)25000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))));
            }
            else
            {
               
              vcoinput = (uint32_t)(((uint32_t)16000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))));
            }
          }
           
          vcooutput = (uint32_t)(vcoinput * (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x1FFUL << (6U))) >> 6U) & ((0x1FFUL << (6U)) >> 6U)));
           
          frequency = (uint32_t)(vcooutput /(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x7UL << (28U))) >> 28U) & ((0x7UL << (28U)) >> 28U)));
          break;
        }
       
      case ((uint32_t)(0x2UL << (25U))):
        {
           
           
          if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
          {
             
            vcoinput = (uint32_t)(((uint32_t)25000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U))));
          }
          else
          {
             
            vcoinput = (uint32_t)(((uint32_t)16000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U))));
          }

           
          vcooutput = (uint32_t)(vcoinput * (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1FFUL << (6U))) >> 6U) & ((0x1FFUL << (6U)) >> 6U)));
           
          frequency = (uint32_t)(vcooutput /(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x7UL << (28U))) >> 28U) & ((0x7UL << (28U)) >> 28U)));
          break;
        }
       
      case ((uint32_t)(0x3UL << (25U))):
        {
          if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
          {
            frequency = ((uint32_t)25000000U);
          }
          else
          {
            frequency = ((uint32_t)16000000U);
          }
          break;
        }
         
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  case 0x00000002U:
    {
       
      srcclk = (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->DCKCFGR) & ((0x3UL << (27U)))));
      switch (srcclk)
      {
         
      case ((uint32_t)(0x1UL << (27U))):
        {
           
          frequency = ((uint32_t)12288000U);
          break;
        }
         
      case 0x00000000U:
        {
          if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
          {
             
            vcoinput = (uint32_t)(((uint32_t)12288000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))));
          }
          else
          {
             
             
            if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
            {
               
              vcoinput = (uint32_t)(((uint32_t)25000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))));
            }
            else
            {
               
              vcoinput = (uint32_t)(((uint32_t)16000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x3FUL << (0U))));
            }
          }
           
          vcooutput = (uint32_t)(vcoinput * (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x1FFUL << (6U))) >> 6U) & ((0x1FFUL << (6U)) >> 6U)));
           
          frequency = (uint32_t)(vcooutput /(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR & (0x7UL << (28U))) >> 28U) & ((0x7UL << (28U)) >> 28U)));
          break;
        }
         
      case ((uint32_t)(0x2UL << (27U))):
        {
           
           
          if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
          {
             
            vcoinput = (uint32_t)(((uint32_t)25000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U))));
          }
          else
          {
             
            vcoinput = (uint32_t)(((uint32_t)16000000U) / (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U))));
          }

           
          vcooutput = (uint32_t)(vcoinput * (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1FFUL << (6U))) >> 6U) & ((0x1FFUL << (6U)) >> 6U)));
           
          frequency = (uint32_t)(vcooutput /(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x7UL << (28U))) >> 28U) & ((0x7UL << (28U)) >> 28U)));
          break;
        }
         
      case ((uint32_t)(0x3UL << (27U))):
        {
          if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))
          {
            frequency = ((uint32_t)25000000U);
          }
          else
          {
            frequency = ((uint32_t)16000000U);
          }
          break;
        }
       
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}


#line 2149 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"

#line 2500 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"

#line 2754 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"













 
void HAL_RCCEx_SelectLSEMode(uint8_t Mode)
{
   
  ((void)0U);
  if(Mode == ((uint8_t)0x01))
  {
    ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (3U))));
  }
  else
  {
    ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) &= ~((0x1UL << (3U))));
  }
}















 







 
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit)
{
  uint32_t tickstart;

   
  ((void)0U);
  ((void)0U);

  ((void)0U);





  ((void)0U);


   
  (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x1AU * 4U)) = DISABLE);

   
  tickstart = HAL_GetTick();
  while((((((((((uint8_t)0x3B)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x3B)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x3B)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x3B)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
  {
    if((HAL_GetTick() - tickstart ) > 2U)
    {
       
      return HAL_TIMEOUT;
    }
  }

   
#line 2846 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
   
   
   
  (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR = ((PLLI2SInit->PLLI2SM) | ((PLLI2SInit->PLLI2SN) << (6U)) | ((PLLI2SInit->PLLI2SQ) << (24U)) | ((PLLI2SInit->PLLI2SR) << (28U))));
#line 2866 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"

   
  (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x1AU * 4U)) = ENABLE);

   
  tickstart = HAL_GetTick();
  while((((((((((uint8_t)0x3B)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x3B)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x3B)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x3B)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
  {
    if((HAL_GetTick() - tickstart ) > 2U)
    {
       
      return HAL_TIMEOUT;
    }
  }

 return HAL_OK;
}




 
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void)
{
  uint32_t tickstart;

   
  (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x1AU * 4U)) = DISABLE);

   
  tickstart = HAL_GetTick();
  while(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) & ((0x1UL << (27U)))) != RESET)
  {
    if((HAL_GetTick() - tickstart) > 2U)
    {
       
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}



#line 3014 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"



 

#line 3123 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"



 



 













 
HAL_StatusTypeDef HAL_RCC_DeInit(void)
{
  uint32_t tickstart;

   
  tickstart = HAL_GetTick();

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (0U))));

   
  while (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) & ((0x1UL << (1U)))) == RESET)
  {
    if ((HAL_GetTick() - tickstart) > 2U)
    {
      return HAL_TIMEOUT;
    }
  }

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x10UL << (3U))));

   
  tickstart = HAL_GetTick();

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR) = (0x0));

   
  while (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR) & ((0x3UL << (2U)))) != RESET)
  {
    if ((HAL_GetTick() - tickstart) > 5000U)
    {
      return HAL_TIMEOUT;
    }
  }

   
  tickstart = HAL_GetTick();

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (16U)) | (0x1UL << (18U)) | (0x1UL << (19U))));

   
  while (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) & ((0x1UL << (17U)))) != RESET)
  {
    if ((HAL_GetTick() - tickstart) > ((uint32_t)100U))
    {
      return HAL_TIMEOUT;
    }
  }

   
  tickstart = HAL_GetTick();

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (24U))));

   
  while (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) & ((0x1UL << (25U)))) != RESET)
  {
    if ((HAL_GetTick() - tickstart) > 2U)
    {
      return HAL_TIMEOUT;
    }
  }


   
  tickstart = HAL_GetTick();

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (26U))));

   
  while (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) & ((0x1UL << (27U)))) != RESET)
  {
    if ((HAL_GetTick() - tickstart) > 2U)
    {
      return HAL_TIMEOUT;
    }
  }


#line 3245 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"

   


  ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR = (0x10UL << (0U)) | (0x040UL << (6U)) | (0x080UL << (6U)) | (0x4UL << (24U)) | (0x2UL << (28U));






   


  ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLI2SCFGR = (0x10UL << (0U)) | (0x040UL << (6U)) | (0x080UL << (6U)) | (0x4UL << (24U)) | (0x2UL << (28U));
#line 3267 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"

   






   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR) &= ~((0x1UL << (8U)) | (0x1UL << (9U)) | (0x1UL << (10U)) | (0x1UL << (11U)) | (0x1UL << (12U))));


  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR) &= ~((0x1UL << (13U))));






   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR) |= ((0x1UL << (16U)) | (0x1UL << (17U)) | (0x1UL << (18U)) | (0x1UL << (19U)) | (0x1UL << (20U)) | (0x1UL << (23U))));


  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR) |= ((0x1UL << (21U))));






   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR) &= ~((0x1UL << (0U))));

   
  ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR) |= ((0x1UL << (24U))));

   
  SystemCoreClock = ((uint32_t)16000000U);

   
  if(HAL_InitTick(uwTickPrio) != HAL_OK)
  {
    return HAL_ERROR;
  }
  else
  {
    return HAL_OK;
  }
}


















 
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
  uint32_t tickstart = 0U;

   
  ((void)0U);
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000001U) == 0x00000001U)
  {
     
    ((void)0U);
     





    if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000004U)                                                                     ||      (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000008U) && ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == (0x1UL << (22U)))))


    {
      if(((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET) && (RCC_OscInitStruct->HSEState == 0x00000000U))
      {
        return HAL_ERROR;
      }
    }
    else
    {
       
      do { if ((RCC_OscInitStruct->HSEState) == (0x1UL << (16U))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (16U)))); } else if ((RCC_OscInitStruct->HSEState) == ((uint32_t)((0x1UL << (18U)) | (0x1UL << (16U))))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (18U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) |= ((0x1UL << (16U)))); } else { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (16U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR) &= ~((0x1UL << (18U)))); } } while(0U);

       
      if((RCC_OscInitStruct->HSEState) != 0x00000000U)
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)100U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)100U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000002U) == 0x00000002U)
  {
     
    ((void)0U);
    ((void)0U);

     





    if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000000U)                                                                     ||      (((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) == 0x00000008U) && ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U))) == 0x00000000U)))


    {
       
      if(((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET) && (RCC_OscInitStruct->HSIState != ((uint8_t)0x01)))
      {
        return HAL_ERROR;
      }
       
      else
      {
         
        ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR))) & (~((0x1FUL << (3U))))) | ((uint32_t)(RCC_OscInitStruct->HSICalibrationValue) << (3U))))));
      }
    }
    else
    {
       
      if((RCC_OscInitStruct->HSIState)!= ((uint8_t)0x00))
      {
         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x00U * 4U)) = ENABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }

         
        ((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR))) & (~((0x1FUL << (3U))))) | ((uint32_t)(RCC_OscInitStruct->HSICalibrationValue) << (3U))))));
      }
      else
      {
         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x00U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000008U) == 0x00000008U)
  {
     
    ((void)0U);

     
    if((RCC_OscInitStruct->LSIState)!= ((uint8_t)0x00))
    {
       
      (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x74U) * 32U) + (0x00U * 4U)) = ENABLE);

       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x61)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x61)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x61)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x61)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
       
      (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x74U) * 32U) + (0x00U * 4U)) = DISABLE);

       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x61)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x61)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x61)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x61)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000004U) == 0x00000004U)
  {
    FlagStatus       pwrclkchanged = RESET;

     
    ((void)0U);

     
     
    if(((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR & ((0x1UL << (28U)))) == RESET))
    {
      do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) |= ((0x1UL << (28U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR) & ((0x1UL << (28U)))); (void)tmpreg; } while(0U);
      pwrclkchanged = SET;
    }

    if((((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) & ((0x1UL << (8U)))) == 0U))
    {
       
      ((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) |= ((0x1UL << (8U))));

       
      tickstart = HAL_GetTick();

      while((((((PWR_TypeDef *) (0x40000000UL + 0x7000UL))->CR) & ((0x1UL << (8U)))) == 0U))
      {
        if((HAL_GetTick() - tickstart) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }

     
    do { if((RCC_OscInitStruct->LSEState) == (0x1UL << (0U))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (0U)))); } else if((RCC_OscInitStruct->LSEState) == ((uint32_t)((0x1UL << (2U)) | (0x1UL << (0U))))) { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (2U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) |= ((0x1UL << (0U)))); } else { ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) &= ~((0x1UL << (0U)))); ((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR) &= ~((0x1UL << (2U)))); } } while(0U);
     
    if((RCC_OscInitStruct->LSEState) != 0x00000000U)
    {
       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
        {
          return HAL_TIMEOUT;
        }
      }
    }

     
    if(pwrclkchanged == SET)
    {
      (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->APB1ENR &= ~((0x1UL << (28U))));
    }
  }
   
   
  ((void)0U);
  if ((RCC_OscInitStruct->PLL.PLLState) != ((uint8_t)0x00))
  {
     
    if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CFGR & (0x3UL << (2U))) != 0x00000008U)
    {
      if((RCC_OscInitStruct->PLL.PLLState) == ((uint8_t)0x02))
      {
         
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);

         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }

         
        (((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR = ((RCC_OscInitStruct->PLL . PLLSource) | (RCC_OscInitStruct->PLL . PLLM) | ((RCC_OscInitStruct->PLL . PLLN) << (6U)) | ((((RCC_OscInitStruct->PLL . PLLP) >> 1U) -1U) << (16U)) | ((RCC_OscInitStruct->PLL . PLLQ) << (24U)) | ((RCC_OscInitStruct->PLL . PLLR) << (28U))));






         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = ENABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
         
        (*(volatile uint32_t *) (0x42000000UL + (((((0x40000000UL + 0x00020000UL) + 0x3800UL) - 0x40000000UL) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR :((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}









 
void HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
   
  RCC_OscInitStruct->OscillatorType = 0x00000001U | 0x00000002U | 0x00000004U | 0x00000008U;

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (18U))) == (0x1UL << (18U)))
  {
    RCC_OscInitStruct->HSEState = ((uint32_t)((0x1UL << (18U)) | (0x1UL << (16U))));
  }
  else if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (16U))) == (0x1UL << (16U)))
  {
    RCC_OscInitStruct->HSEState = (0x1UL << (16U));
  }
  else
  {
    RCC_OscInitStruct->HSEState = 0x00000000U;
  }

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (0U))) == (0x1UL << (0U)))
  {
    RCC_OscInitStruct->HSIState = ((uint8_t)0x01);
  }
  else
  {
    RCC_OscInitStruct->HSIState = ((uint8_t)0x00);
  }

  RCC_OscInitStruct->HSICalibrationValue = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1FUL << (3U))) >> (3U));

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR &(0x1UL << (2U))) == (0x1UL << (2U)))
  {
    RCC_OscInitStruct->LSEState = ((uint32_t)((0x1UL << (2U)) | (0x1UL << (0U))));
  }
  else if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->BDCR &(0x1UL << (0U))) == (0x1UL << (0U)))
  {
    RCC_OscInitStruct->LSEState = (0x1UL << (0U));
  }
  else
  {
    RCC_OscInitStruct->LSEState = 0x00000000U;
  }

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CSR &(0x1UL << (0U))) == (0x1UL << (0U)))
  {
    RCC_OscInitStruct->LSIState = ((uint8_t)0x01);
  }
  else
  {
    RCC_OscInitStruct->LSIState = ((uint8_t)0x00);
  }

   
  if((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->CR &(0x1UL << (24U))) == (0x1UL << (24U)))
  {
    RCC_OscInitStruct->PLL.PLLState = ((uint8_t)0x02);
  }
  else
  {
    RCC_OscInitStruct->PLL.PLLState = ((uint8_t)0x01);
  }
  RCC_OscInitStruct->PLL.PLLSource = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1UL << (22U)));
  RCC_OscInitStruct->PLL.PLLM = (uint32_t)(((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3FUL << (0U)));
  RCC_OscInitStruct->PLL.PLLN = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x1FFUL << (6U))) >> (6U));
  RCC_OscInitStruct->PLL.PLLP = (uint32_t)((((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x3UL << (16U))) + (0x1UL << (16U))) << 1U) >> (16U));
  RCC_OscInitStruct->PLL.PLLQ = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0xFUL << (24U))) >> (24U));
  RCC_OscInitStruct->PLL.PLLR = (uint32_t)((((RCC_TypeDef *) ((0x40000000UL + 0x00020000UL) + 0x3800UL))->PLLCFGR & (0x7UL << (28U))) >> (28U));
}





 



 

 
