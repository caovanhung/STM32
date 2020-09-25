#ifndef __STM32F103_MYLIB_H
#define __STM32F103_MYLIB_H

typedef enum IRQn
{
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */
	
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
  TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
  RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                      */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                      */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                      */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                      */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                      */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                      */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                      */
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42      /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */  
} IRQn_Type;


#include "core_cm3.h"
#include "system_stm32f10x.h"
#include <stdint.h>


/***********************************************************GPIIO**************************************************************************************/

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO  uint32_t CRL;                              
  __IO  uint32_t CRH;                              
  __I   uint32_t IDR;                              
  __IO  uint32_t ODR;                              
  __O   uint32_t BSRR;                             
  __O   uint32_t BRR;                              
  __IO  uint32_t LCRK; 
} GPIO_Type;

#define GPIO_CRL_CRL_MASK							(0xFFFFFFFFU)
#define GPIO_CRL_CRL_SHIFT						(0U)
#define GPIO_CRL_CRL(x)								(((uint32_t)((uint32_t)1 << x)) & GPIO_CRL_CRL_MASK)
#define GPIO_CRH_CRH_MASK							(0xFFFFFFFFU)
#define GPIO_CRH_CRH_SHIFT						(0U)
#define GPIO_CRH_CRH(x)								(((uint32_t)((uint32_t)1 << x)) & GPIO_CRH_CRH_MASK)
#define GPIO_IDR_ID_MASK							(0xFFFFFFFFU)
#define GPIO_IDR_ID_SHIFT							(0U)
#define GPIO_IDR_ID(x)								(((uint32_t)((uint32_t)1 << x)) & GPIO_IDR_ID_MASK)
#define GPIO_ODR_OD_MASK							(0xFFFFFFFFU)
#define GPIO_ODR_OD_SHIFT							(0U)
#define GPIO_ODR_OD(x)								(((uint32_t)((uint32_t)1 << x)) & GPIO_ODR_OD_MASK)
#define GPIO_BSRR_BSR_MASK						(0xFFFFFFFFU)
#define GPIO_BSRR_BSR_SHIFT						(0U)	
#define GPIO_BSRR_BSR(x)							(((uint32_t)((uint32_t)1 << x)) & GPIO_BSRR_BSR_MASK)
#define GPIO_BRR_BR_MASK							(0xFFFFFFFFU)
#define GPIO_BRR_BR_SHIFT							(0U)
#define GPIO_BRR_BR(x)								(((uint32_t)((uint32_t)1 << x)) & GPIO_BRR_BR_MASK)
#define GPIO_LCKR_LCK_MASK						(0xFFFFFFFFU)
#define GPIO_LCKR_LCK_SHIFT						(0U)
#define GPIO_LCKR_LCK(x)							(((uint32_t)((uint32_t)1 << x)) & GPIO_LCKR_LCK_MASK)

/*GPIO- Peripheral instance base address */
/** Peripheral GPIOA base address */
#define GPIOA_BASE                               (0x40010800u)
/** Peripheral GPIOA base pointer */
#define GPIOA                                    ((GPIO_Type *)GPIOA_BASE)
/** Peripheral GPIOB base address */
#define GPIOB_BASE                               (0x40010C00u)
/** Peripheral GPIOB base pointer */
#define GPIOB                                    ((GPIO_Type *)GPIOB_BASE)
/** Peripheral GPIOC base address */
#define GPIOC_BASE                               (0x40011000u)
/** Peripheral GPIOC base pointer */
#define GPIOC                                    ((GPIO_Type *)GPIOC_BASE)
/** Peripheral GPIOD base address */
#define GPIOD_BASE                               (0x40011400u)
/** Peripheral GPIOD base pointer */
#define GPIOD                                    ((GPIO_Type *)GPIOD_BASE)
/** Peripheral GPIOE base address */
#define GPIOE_BASE                               (0x40011800u)
/** Peripheral GPIOE base pointer */
#define GPIOE                                    ((GPIO_Type *)GPIOE_BASE)
/** Peripheral GPIOF base address */
#define GPIOF_BASE                               (0x40011C00u)
/** Peripheral GPIOF base pointer */
#define GPIOF                                    ((GPIO_Type *)GPIOE_BASE)
/** Peripheral GPIOG base address */
#define GPIOG_BASE                               (0x40012000u)
/** Peripheral GPIOG base pointer */
#define GPIOG                                    ((GPIO_Type *)GPIOE_BASE)


/***********************************************************RCC**************************************************************************************/

/**RCC- Reset and Clock Control                         **/
#define RCC_BASE                               (0x40021000u)
#define RCC                               	   ((RCC_Type *)RCC_BASE)

/** RCC - Register Layout Typedef */
typedef struct {
  __IO  uint32_t CR;                              
  __IO  uint32_t CFGR;                              
  __IO  uint32_t CIR;                              
  __IO  uint32_t APB2RSTR;                              
  __IO  uint32_t APB1RSTR;                             
  __IO  uint32_t AHBENR;                              
  __IO  uint32_t APB2ENR; 
  __IO  uint32_t APB1ENR; 
  __IO  uint32_t BDCR; 
  __IO  uint32_t CSR; 
} RCC_Type;

#define RCC_CR_C_MASK								(0xFFFFFFFFU)
#define RCC_CR_C(x) 								(((uint32_t)((uint32_t)1 << x)) & RCC_CR_C_MASK)
#define RCC_CFGR_CFG_MASK							(0xFFFFFFFFU)
#define RCC_CFGR_CFG(x) 							(((uint32_t)((uint32_t)1 << x)) & RCC_CFGR_CFG_MASK)
#define RCC_CIR_CI_MASK								(0xFFFFFFFFU)
#define RCC_CIR_CI(x) 								(((uint32_t)((uint32_t)1 << x)) & RCC_CIR_CI_MASK)
#define RCC_APB2RSTR_APB2RST_MASK					(0xFFFFFFFFU)
#define RCC_APB2RSTR_APB2RST(x) 					(((uint32_t)((uint32_t)1 << x)) & RCC_APB2RSTR_APB2RST_MASK)
#define RCC_APB1RSTR_APB1RST_MASK					(0xFFFFFFFFU)
#define RCC_APB1RSTR_APB1RST(x) 					(((uint32_t)((uint32_t)1 << x)) & RCC_APB1RSTR_APB1RST_MASK)
#define RCC_AHBENR_AHBEN_MASK						(0xFFFFFFFFU)
#define RCC_AHBENR_AHBEN(x) 						(((uint32_t)((uint32_t)1 << x)) & RCC_AHBENR_AHBEN_MASK)
#define RCC_APB2ENR_APB2EN_MASK						(0xFFFFFFFFU)
#define RCC_APB2ENR_APB2EN(x) 						(((uint32_t)((uint32_t)1 << x)) & RCC_APB2ENR_APB2EN_MASK)
#define RCC_APB1ENR_APB1EN_MASK						(0xFFFFFFFFU)
#define RCC_APB1ENR_APB1EN(x) 						(((uint32_t)((uint32_t)1 << x)) & RCC_APB1ENR_APB1EN_MASK)
#define RCC_BDCR_BDC_MASK							(0xFFFFFFFFU)
#define RCC_BDCR_BDC(x) 							(((uint32_t)((uint32_t)1 << x)) & RCC_BDCR_BDC_MASK)
#define RCC_CSR_CS_MASK								(0xFFFFFFFFU)
#define RCC_CSR_CS(x) 								(((uint32_t)((uint32_t)1 << x)) & RCC_CSR_CS_MASK)


#endif /* __STM32F103_MYLIB_H */
