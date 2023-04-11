#ifndef _GD32VF103_H_
#define _GD32VF103_H_

#include "riscv_encoding.h"

#include <stdint.h>


/*!
    \file    gd32vf103.h
    \brief   general definitions for GD32VF103

    \version 2019-06-05, V1.0.0, firmware for GD32VF103
    \version 2020-08-04, V1.1.0, firmware for GD32VF103
    \version 2022-12-21, solid header based on V1.1.0 firmware for GD32VF103
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifdef __cplusplus
 extern "C" {
#endif 

 /* IO definitions (access restrictions to peripheral registers) */
 /**

     <strong>IO Type Qualifiers</strong> are used
     \li to specify the access to peripheral variables.
     \li for automatic generation of peripheral register debug information.
 */
 #ifdef __cplusplus
   #define   __I     volatile             /*!< Defines 'read only' permissions                 */
 #else
   #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
 #endif
 #define     __O     volatile             /*!< Defines 'write only' permissions                */
 #define     __IO    volatile             /*!< Defines 'read / write' permissions              */

// define CK_SYS value, Hz
#ifndef CK_SYS_VALUE
#define CK_SYS_VALUE  ((uint32_t)48000000)
#endif

extern uint32_t SystemCoreClock;
extern volatile uint32_t g_milliseconds;

/* define value of high speed crystal oscillator (HXTAL) in Hz */
#ifndef HXTAL_VALUE  
#define HXTAL_VALUE    ((uint32_t)8000000) /*!< value of the external oscillator in Hz */
#endif /* high speed crystal oscillator value */

/* define startup timeout value of high speed crystal oscillator (HXTAL) */
#ifndef HXTAL_STARTUP_TIMEOUT
#define HXTAL_STARTUP_TIMEOUT   ((uint16_t)0xFFFF)
#endif /* high speed crystal oscillator startup timeout */

/* define value of internal 8MHz RC oscillator (IRC8M) in Hz */
#ifndef IRC8M_VALUE
#define IRC8M_VALUE  ((uint32_t)8000000)
#endif /* internal 8MHz RC oscillator value */

/* define startup timeout value of internal 8MHz RC oscillator (IRC8M) */
#ifndef IRC8M_STARTUP_TIMEOUT
#define IRC8M_STARTUP_TIMEOUT   ((uint16_t)0x0500)
#endif /* internal 8MHz RC oscillator startup timeout */

/* define value of internal 40KHz RC oscillator(IRC40K) in Hz */
#ifndef IRC40K_VALUE
#define IRC40K_VALUE  ((uint32_t)40000)
#endif /* internal 40KHz RC oscillator value */

/* define value of low speed crystal oscillator (LXTAL)in Hz */
#ifndef LXTAL_VALUE
#define LXTAL_VALUE  ((uint32_t)32768)
#endif /* low speed crystal oscillator value */

/* define interrupt number */
typedef enum IRQn
{

    CLIC_INT_RESERVED            = 0,       /*!< RISC-V reserved                                        */
    CLIC_INT_SFT                 = 3,       /*!< Software interrupt                                     */
    CLIC_INT_TMR                 = 7,       /*!< CPU Timer interrupt                                    */
    CLIC_INT_BWEI                = 17,      /*!< Bus Error interrupt                                    */
    CLIC_INT_PMOVI               = 18,      /*!< Performance Monitor                                    */

    /* interruput numbers */
    WWDGT_IRQn                   = 19,      /*!< window watchDog timer interrupt                         */
    LVD_IRQn                     = 20,      /*!< LVD through EXTI line detect interrupt                  */
    TAMPER_IRQn                  = 21,      /*!< tamper through EXTI line detect                         */
    RTC_IRQn                     = 22,      /*!< RTC alarm interrupt                                     */
    FMC_IRQn                     = 23,      /*!< FMC interrupt                                           */
    RCU_CTC_IRQn                 = 24,      /*!< RCU and CTC interrupt                                   */
    EXTI0_IRQn                   = 25,      /*!< EXTI line 0 interrupts                                  */
    EXTI1_IRQn                   = 26,      /*!< EXTI line 1 interrupts                                  */
    EXTI2_IRQn                   = 27,      /*!< EXTI line 2 interrupts                                  */
    EXTI3_IRQn                   = 28,      /*!< EXTI line 3 interrupts                                  */
    EXTI4_IRQn                   = 29,     /*!< EXTI line 4 interrupts                                   */
    DMA0_Channel0_IRQn           = 30,     /*!< DMA0 channel0 interrupt                                  */
    DMA0_Channel1_IRQn           = 31,     /*!< DMA0 channel1 interrupt                                  */
    DMA0_Channel2_IRQn           = 32,     /*!< DMA0 channel2 interrupt                                  */
    DMA0_Channel3_IRQn           = 33,     /*!< DMA0 channel3 interrupt                                  */
    DMA0_Channel4_IRQn           = 34,     /*!< DMA0 channel4 interrupt                                  */
    DMA0_Channel5_IRQn           = 35,     /*!< DMA0 channel5 interrupt                                  */
    DMA0_Channel6_IRQn           = 36,     /*!< DMA0 channel6 interrupt                                  */
    ADC0_1_IRQn                  = 37,     /*!< ADC0 and ADC1 interrupt                                  */
    CAN0_TX_IRQn                 = 38,     /*!< CAN0 TX interrupts                                       */
    CAN0_RX0_IRQn                = 39,     /*!< CAN0 RX0 interrupts                                      */
    CAN0_RX1_IRQn                = 40,     /*!< CAN0 RX1 interrupts                                      */
    CAN0_EWMC_IRQn               = 41,     /*!< CAN0 EWMC interrupts                                     */
    EXTI5_9_IRQn                 = 42,     /*!< EXTI[9:5] interrupts                                     */
    TIMER0_BRK_IRQn              = 43,     /*!< TIMER0 break interrupts                                  */
    TIMER0_UP_IRQn               = 44,     /*!< TIMER0 update interrupts                                 */
    TIMER0_TRG_CMT_IRQn          = 45,     /*!< TIMER0 trigger and commutation interrupts                */
    TIMER0_Channel_IRQn          = 46,     /*!< TIMER0 channel capture compare interrupts                */
    TIMER1_IRQn                  = 47,     /*!< TIMER1 interrupt                                         */
    TIMER2_IRQn                  = 48,     /*!< TIMER2 interrupt                                         */
    TIMER3_IRQn                  = 49,     /*!< TIMER3 interrupts                                        */
    I2C0_EV_IRQn                 = 50,     /*!< I2C0 event interrupt                                     */
    I2C0_ER_IRQn                 = 51,     /*!< I2C0 error interrupt                                     */
    I2C1_EV_IRQn                 = 52,     /*!< I2C1 event interrupt                                     */
    I2C1_ER_IRQn                 = 53,     /*!< I2C1 error interrupt                                     */
    SPI0_IRQn                    = 54,     /*!< SPI0 interrupt                                           */
    SPI1_IRQn                    = 55,     /*!< SPI1 interrupt                                           */
    USART0_IRQn                  = 56,     /*!< USART0 interrupt                                         */
    USART1_IRQn                  = 57,     /*!< USART1 interrupt                                         */
    USART2_IRQn                  = 58,     /*!< USART2 interrupt                                         */
    EXTI10_15_IRQn               = 59,     /*!< EXTI[15:10] interrupts                                   */
    RTC_ALARM_IRQn               = 60,     /*!< RTC alarm interrupt EXTI                                 */
    USBFS_WKUP_IRQn              = 61,     /*!< USBFS wakeup interrupt                                   */

    EXMC_IRQn                    = 67,     /*!< EXMC global interrupt                                    */

    TIMER4_IRQn                  = 69,     /*!< TIMER4 global interrupt                                  */
    SPI2_IRQn                    = 70,     /*!< SPI2 global interrupt                                    */
    UART3_IRQn                   = 71,     /*!< UART3 global interrupt                                   */
    UART4_IRQn                   = 72,     /*!< UART4 global interrupt                                   */
    TIMER5_IRQn                  = 73,     /*!< TIMER5 global interrupt                                  */
    TIMER6_IRQn                  = 74,     /*!< TIMER6 global interrupt                                  */
    DMA1_Channel0_IRQn           = 75,     /*!< DMA1 channel0 global interrupt                           */
    DMA1_Channel1_IRQn           = 76,     /*!< DMA1 channel1 global interrupt                           */
    DMA1_Channel2_IRQn           = 77,     /*!< DMA1 channel2 global interrupt                           */
    DMA1_Channel3_IRQn           = 78,     /*!< DMA1 channel3 global interrupt                           */
    DMA1_Channel4_IRQn           = 79,     /*!< DMA1 channel3 global interrupt                           */

    CAN1_TX_IRQn                 = 82,     /*!< CAN1 TX interrupt                                        */
    CAN1_RX0_IRQn                = 83,     /*!< CAN1 RX0 interrupt                                       */
    CAN1_RX1_IRQn                = 84,     /*!< CAN1 RX1 interrupt                                       */
    CAN1_EWMC_IRQn               = 85,     /*!< CAN1 EWMC interrupt                                      */
    USBFS_IRQn                   = 86,     /*!< USBFS global interrupt                                   */

    ECLIC_NUM_INTERRUPTS
} IRQn_Type;

/* enum definitions */
typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
typedef enum {RESET = 0, SET = !RESET} FlagStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;

/* bit operations */
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end)))) 
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))

/* main flash and SRAM memory map */
#define FLASH_BASE            ((uint32_t)0x08000000U)        /*!< main FLASH base address          */
#define SRAM_BASE             ((uint32_t)0x20000000U)        /*!< SRAM0 base address               */
#define OB_BASE               ((uint32_t)0x1FFFF800U)        /*!< OB base address                  */
#define DBG_BASE              ((uint32_t)0xE0042000U)        /*!< DBG base address                 */
#define EXMC_BASE             ((uint32_t)0xA0000000U)        /*!< EXMC register base address       */

/* peripheral memory map */
#define APB1_BUS_BASE         ((uint32_t)0x40000000U)        /*!< apb1 base address                */
#define APB2_BUS_BASE         ((uint32_t)0x40010000U)        /*!< apb2 base address                */
#define AHB1_BUS_BASE         ((uint32_t)0x40018000U)        /*!< ahb1 base address                */
#define AHB3_BUS_BASE         ((uint32_t)0x60000000U)        /*!< ahb3 base address                */

/* advanced peripheral bus 1 memory map */
#define TIMER_BASE            (APB1_BUS_BASE + 0x00000000U)  /*!< TIMER base address               */
#define RTC_BASE              (APB1_BUS_BASE + 0x00002800U)  /*!< RTC base address                 */
#define WWDGT_BASE            (APB1_BUS_BASE + 0x00002C00U)  /*!< WWDGT base address               */
#define FWDGT_BASE            (APB1_BUS_BASE + 0x00003000U)  /*!< FWDGT base address               */
#define SPI_BASE              (APB1_BUS_BASE + 0x00003800U)  /*!< SPI base address                 */
#define USART_BASE            (APB1_BUS_BASE + 0x00004400U)  /*!< USART base address               */
#define I2C_BASE              (APB1_BUS_BASE + 0x00005400U)  /*!< I2C base address                 */
#define CAN_BASE              (APB1_BUS_BASE + 0x00006400U)  /*!< CAN base address                 */
#define BKP_BASE              (APB1_BUS_BASE + 0x00006C00U)  /*!< BKP base address                 */
#define PMU_BASE              (APB1_BUS_BASE + 0x00007000U)  /*!< PMU base address                 */
#define DAC_BASE              (APB1_BUS_BASE + 0x00007400U)  /*!< DAC base address                 */

/* advanced peripheral bus 2 memory map */
#define AFIO_BASE             (APB2_BUS_BASE + 0x00000000U)  /*!< AFIO base address                */
#define EXTI_BASE             (APB2_BUS_BASE + 0x00000400U)  /*!< EXTI base address                */
#define GPIO_BASE             (APB2_BUS_BASE + 0x00000800U)  /*!< GPIO base address                */
#define ADC_BASE              (APB2_BUS_BASE + 0x00002400U)  /*!< ADC base address                 */

/* advanced high performance bus 1 memory map */
#define DMA_BASE              (AHB1_BUS_BASE + 0x00008000U)  /*!< DMA base address                 */
#define RCU_BASE              (AHB1_BUS_BASE + 0x00009000U)  /*!< RCU base address                 */
#define FMC_BASE              (AHB1_BUS_BASE + 0x0000A000U)  /*!< FMC base address                 */
#define CRC_BASE              (AHB1_BUS_BASE + 0x0000B000U)  /*!< CRC base address                 */
#define USBFS_BASE            (AHB1_BUS_BASE + 0x0FFE8000U)  /*!< USBFS base address               */


////////////////////////////////////////////////////////////////////////
/* ADC definitions */
#define ADC0                            ADC_BASE
#define ADC1                            (ADC_BASE + 0x400U)

/* registers definitions */
#define ADC_STAT(adcx)                  REG32((adcx) + 0x00U)            /*!< ADC status register */
#define ADC_CTL0(adcx)                  REG32((adcx) + 0x04U)            /*!< ADC control register 0 */
#define ADC_CTL1(adcx)                  REG32((adcx) + 0x08U)            /*!< ADC control register 1 */
#define ADC_SAMPT0(adcx)                REG32((adcx) + 0x0CU)            /*!< ADC sampling time register 0 */
#define ADC_SAMPT1(adcx)                REG32((adcx) + 0x10U)            /*!< ADC sampling time register 1 */
#define ADC_IOFF0(adcx)                 REG32((adcx) + 0x14U)            /*!< ADC inserted channel data offset register 0 */
#define ADC_IOFF1(adcx)                 REG32((adcx) + 0x18U)            /*!< ADC inserted channel data offset register 1 */
#define ADC_IOFF2(adcx)                 REG32((adcx) + 0x1CU)            /*!< ADC inserted channel data offset register 2 */
#define ADC_IOFF3(adcx)                 REG32((adcx) + 0x20U)            /*!< ADC inserted channel data offset register 3 */
#define ADC_WDHT(adcx)                  REG32((adcx) + 0x24U)            /*!< ADC watchdog high threshold register */
#define ADC_WDLT(adcx)                  REG32((adcx) + 0x28U)            /*!< ADC watchdog low threshold register */
#define ADC_RSQ0(adcx)                  REG32((adcx) + 0x2CU)            /*!< ADC regular sequence register 0 */
#define ADC_RSQ1(adcx)                  REG32((adcx) + 0x30U)            /*!< ADC regular sequence register 1 */
#define ADC_RSQ2(adcx)                  REG32((adcx) + 0x34U)            /*!< ADC regular sequence register 2 */
#define ADC_ISQ(adcx)                   REG32((adcx) + 0x38U)            /*!< ADC inserted sequence register */
#define ADC_IDATA0(adcx)                REG32((adcx) + 0x3CU)            /*!< ADC inserted data register 0 */
#define ADC_IDATA1(adcx)                REG32((adcx) + 0x40U)            /*!< ADC inserted data register 1 */
#define ADC_IDATA2(adcx)                REG32((adcx) + 0x44U)            /*!< ADC inserted data register 2 */
#define ADC_IDATA3(adcx)                REG32((adcx) + 0x48U)            /*!< ADC inserted data register 3 */
#define ADC_RDATA(adcx)                 REG32((adcx) + 0x4CU)            /*!< ADC regular data register */
#define ADC_OVSCR(adcx)                 REG32((adcx) + 0x80U)            /*!< ADC oversample control register */

/* bits definitions */
/* ADC_STAT */
#define ADC_STAT_WDE                    BIT(0)                           /*!< analog watchdog event flag */
#define ADC_STAT_EOC                    BIT(1)                           /*!< end of conversion */
#define ADC_STAT_EOIC                   BIT(2)                           /*!< inserted channel end of conversion */
#define ADC_STAT_STIC                   BIT(3)                           /*!< inserted channel start flag */
#define ADC_STAT_STRC                   BIT(4)                           /*!< regular channel start flag */

/* ADC_CTL0 */
#define ADC_CTL0_WDCHSEL                BITS(0,4)                        /*!< analog watchdog channel select bits */
#define ADC_CTL0_EOCIE                  BIT(5)                           /*!< interrupt enable for EOC */
#define ADC_CTL0_WDEIE                  BIT(6)                           /*!< analog watchdog interrupt enable */
#define ADC_CTL0_EOICIE                 BIT(7)                           /*!< interrupt enable for inserted channels */
#define ADC_CTL0_SM                     BIT(8)                           /*!< scan mode */
#define ADC_CTL0_WDSC                   BIT(9)                           /*!< when in scan mode, analog watchdog is effective on a single channel */
#define ADC_CTL0_ICA                    BIT(10)                          /*!< automatic inserted group conversion */
#define ADC_CTL0_DISRC                  BIT(11)                          /*!< discontinuous mode on regular channels */
#define ADC_CTL0_DISIC                  BIT(12)                          /*!< discontinuous mode on inserted channels */
#define ADC_CTL0_DISNUM                 BITS(13,15)                      /*!< discontinuous mode channel count */
#define ADC_CTL0_SYNCM                  BITS(16,19)                      /*!< sync mode selection */
#define ADC_CTL0_IWDEN                  BIT(22)                          /*!< analog watchdog enable on inserted channels */
#define ADC_CTL0_RWDEN                  BIT(23)                          /*!< analog watchdog enable on regular channels */

/* ADC_CTL1 */
#define ADC_CTL1_ADCON                  BIT(0)                           /*!< ADC converter on */
#define ADC_CTL1_CTN                    BIT(1)                           /*!< continuous conversion */
#define ADC_CTL1_CLB                    BIT(2)                           /*!< ADC calibration */
#define ADC_CTL1_RSTCLB                 BIT(3)                           /*!< reset calibration */
#define ADC_CTL1_DMA                    BIT(8)                           /*!< direct memory access mode */
#define ADC_CTL1_DAL                    BIT(11)                          /*!< data alignment */
#define ADC_CTL1_ETSIC                  BITS(12,14)                      /*!< external trigger select for inserted channel */
#define ADC_CTL1_ETEIC                  BIT(15)                          /*!< external trigger enable for inserted channel */
#define ADC_CTL1_ETSRC                  BITS(17,19)                      /*!< external trigger select for regular channel */
#define ADC_CTL1_ETERC                  BIT(20)                          /*!< external trigger conversion mode for inserted channels */
#define ADC_CTL1_SWICST                 BIT(21)                          /*!< start on inserted channel */
#define ADC_CTL1_SWRCST                 BIT(22)                          /*!< start on regular channel */
#define ADC_CTL1_TSVREN                 BIT(23)                          /*!< channel 16 and 17 enable of ADC0 */

/* ADC_SAMPTx x=0..1 */
#define ADC_SAMPTX_SPTN                 BITS(0,2)                        /*!< channel n sample time selection */

/* ADC_IOFFx x=0..3 */
#define ADC_IOFFX_IOFF                  BITS(0,11)                       /*!< data offset for inserted channel x */

/* ADC_WDHT */
#define ADC_WDHT_WDHT                   BITS(0,11)                       /*!< analog watchdog high threshold */

/* ADC_WDLT */
#define ADC_WDLT_WDLT                   BITS(0,11)                       /*!< analog watchdog low threshold */

/* ADC_RSQx x=0..2 */
#define ADC_RSQX_RSQN                   BITS(0,4)                        /*!< nth conversion in regular sequence */
#define ADC_RSQ0_RL                     BITS(20,23)                      /*!< regular channel sequence length */

/* ADC_ISQ */
#define ADC_ISQ_ISQN                    BITS(0,4)                        /*!< nth conversion in inserted sequence */
#define ADC_ISQ_IL                      BITS(20,21)                      /*!< inserted sequence length */

/* ADC_IDATAx x=0..3*/
#define ADC_IDATAX_IDATAN               BITS(0,15)                       /*!< inserted data n */

/* ADC_RDATA */
#define ADC_RDATA_RDATA                 BITS(0,15)                       /*!< regular data */
#define ADC_RDATA_ADC1RDTR              BITS(16,31)                      /*!< ADC1 regular channel data */

/* ADC_OVSCR */
#define ADC_OVSCR_OVSEN                 BIT(0)                           /*!< oversampling enable */
#define ADC_OVSCR_OVSR                  BITS(2,4)                        /*!< oversampling ratio */
#define ADC_OVSCR_OVSS                  BITS(5,8)                        /*!< oversampling shift */
#define ADC_OVSCR_TOVS                  BIT(9)                           /*!< triggered oversampling */
#define ADC_OVSCR_DRES                  BITS(12,13)                      /*!< ADC data resolution */

/* constants definitions */
/* adc_stat register value */
#define ADC_FLAG_WDE                    ADC_STAT_WDE                                 /*!< analog watchdog event flag */
#define ADC_FLAG_EOC                    ADC_STAT_EOC                                 /*!< end of conversion */
#define ADC_FLAG_EOIC                   ADC_STAT_EOIC                                /*!< inserted channel end of conversion */
#define ADC_FLAG_STIC                   ADC_STAT_STIC                                /*!< inserted channel start flag */
#define ADC_FLAG_STRC                   ADC_STAT_STRC                                /*!< regular channel start flag */

/* adc_ctl0 register value */
#define CTL0_DISNUM(regval)             (BITS(13,15) & ((uint32_t)(regval) << 13))   /*!< write value to ADC_CTL0_DISNUM bit field */

/* scan mode */
#define ADC_SCAN_MODE                   ADC_CTL0_SM                                  /*!< scan mode */

/* inserted channel group convert automatically */
#define ADC_INSERTED_CHANNEL_AUTO       ADC_CTL0_ICA                                 /*!< inserted channel group convert automatically */

/* ADC sync mode */
#define CTL0_SYNCM(regval)              (BITS(16,19) & ((uint32_t)(regval) << 16))   /*!< write value to ADC_CTL0_SYNCM bit field */
#define ADC_MODE_FREE                                        CTL0_SYNCM(0)           /*!< all the ADCs work independently */
#define ADC_DAUL_REGULAL_PARALLEL_INSERTED_PARALLEL          CTL0_SYNCM(1)           /*!< ADC0 and ADC1 work in combined regular parallel + inserted parallel mode */
#define ADC_DAUL_REGULAL_PARALLEL_INSERTED_ROTATION          CTL0_SYNCM(2)           /*!< ADC0 and ADC1 work in combined regular parallel + trigger rotation mode */
#define ADC_DAUL_INSERTED_PARALLEL_REGULAL_FOLLOWUP_FAST     CTL0_SYNCM(3)           /*!< ADC0 and ADC1 work in combined inserted parallel + follow-up fast mode */
#define ADC_DAUL_INSERTED_PARALLEL_REGULAL_FOLLOWUP_SLOW     CTL0_SYNCM(4)           /*!< ADC0 and ADC1 work in combined inserted parallel + follow-up slow mode */
#define ADC_DAUL_INSERTED_PARALLEL                           CTL0_SYNCM(5)           /*!< ADC0 and ADC1 work in inserted parallel mode only */
#define ADC_DAUL_REGULAL_PARALLEL                            CTL0_SYNCM(6)           /*!< ADC0 and ADC1 work in regular parallel mode only */
#define ADC_DAUL_REGULAL_FOLLOWUP_FAST                       CTL0_SYNCM(7)           /*!< ADC0 and ADC1 work in follow-up fast mode only */
#define ADC_DAUL_REGULAL_FOLLOWUP_SLOW                       CTL0_SYNCM(8)           /*!< ADC0 and ADC1 work in follow-up slow mode only */
#define ADC_DAUL_INSERTED_TRIGGER_ROTATION                   CTL0_SYNCM(9)           /*!< ADC0 and ADC1 work in trigger rotation mode only */

/* adc_ctl1 register value */
#define ADC_DATAALIGN_RIGHT              ((uint32_t)0x00000000U)                     /*!< LSB alignment */
#define ADC_DATAALIGN_LEFT               ADC_CTL1_DAL                                /*!< MSB alignment */

/* continuous mode */
#define ADC_CONTINUOUS_MODE              ADC_CTL1_CTN                                /*!< continuous mode */

/* external trigger select for regular channel */
#define CTL1_ETSRC(regval)               (BITS(17,19) & ((uint32_t)(regval) << 17))  /*!< write value to ADC_CTL1_ETSRC bit field */
/* for ADC0 and ADC1 regular channel */
#define ADC0_1_EXTTRIG_REGULAR_T0_CH0    CTL1_ETSRC(0)                               /*!< TIMER0 CH0 event select */
#define ADC0_1_EXTTRIG_REGULAR_T0_CH1    CTL1_ETSRC(1)                               /*!< TIMER0 CH1 event select */
#define ADC0_1_EXTTRIG_REGULAR_T0_CH2    CTL1_ETSRC(2)                               /*!< TIMER0 CH2 event select */
#define ADC0_1_EXTTRIG_REGULAR_T1_CH1    CTL1_ETSRC(3)                               /*!< TIMER1 CH1 event select */
#define ADC0_1_EXTTRIG_REGULAR_T2_TRGO   CTL1_ETSRC(4)                               /*!< TIMER2 TRGO event select */
#define ADC0_1_EXTTRIG_REGULAR_T3_CH3    CTL1_ETSRC(5)                               /*!< TIMER3 CH3 event select */
#define ADC0_1_EXTTRIG_REGULAR_EXTI_11   CTL1_ETSRC(6)                               /*!< external interrupt line 11 */
#define ADC0_1_EXTTRIG_REGULAR_NONE      CTL1_ETSRC(7)                               /*!< software trigger */

/* external trigger mode for inserted channel */
#define CTL1_ETSIC(regval)               (BITS(12,14) & ((uint32_t)(regval) << 12))  /*!< write value to ADC_CTL1_ETSIC bit field */
/* for ADC0 and ADC1 inserted channel */
#define ADC0_1_EXTTRIG_INSERTED_T0_TRGO  CTL1_ETSIC(0)                               /*!< TIMER0 TRGO event select */
#define ADC0_1_EXTTRIG_INSERTED_T0_CH3   CTL1_ETSIC(1)                               /*!< TIMER0 CH3 event select */
#define ADC0_1_EXTTRIG_INSERTED_T1_TRGO  CTL1_ETSIC(2)                               /*!< TIMER1 TRGO event select */
#define ADC0_1_EXTTRIG_INSERTED_T1_CH0   CTL1_ETSIC(3)                               /*!< TIMER1 CH0 event select */
#define ADC0_1_EXTTRIG_INSERTED_T2_CH3   CTL1_ETSIC(4)                               /*!< TIMER2 CH3 event select */
#define ADC0_1_EXTTRIG_INSERTED_T3_TRGO  CTL1_ETSIC(5)                               /*!< TIMER3 TRGO event select */
#define ADC0_1_EXTTRIG_INSERTED_EXTI_15  CTL1_ETSIC(6)                               /*!< external interrupt line 15 */
#define ADC0_1_EXTTRIG_INSERTED_NONE     CTL1_ETSIC(7)                               /*!< software trigger */

/* adc_samptx register value */
#define SAMPTX_SPT(regval)               (BITS(0,2) & ((uint32_t)(regval) << 0))     /*!< write value to ADC_SAMPTX_SPT bit field */
#define ADC_SAMPLETIME_1POINT5           SAMPTX_SPT(0)                               /*!< 1.5 sampling cycles */
#define ADC_SAMPLETIME_7POINT5           SAMPTX_SPT(1)                               /*!< 7.5 sampling cycles */
#define ADC_SAMPLETIME_13POINT5          SAMPTX_SPT(2)                               /*!< 13.5 sampling cycles */
#define ADC_SAMPLETIME_28POINT5          SAMPTX_SPT(3)                               /*!< 28.5 sampling cycles */
#define ADC_SAMPLETIME_41POINT5          SAMPTX_SPT(4)                               /*!< 41.5 sampling cycles */
#define ADC_SAMPLETIME_55POINT5          SAMPTX_SPT(5)                               /*!< 55.5 sampling cycles */
#define ADC_SAMPLETIME_71POINT5          SAMPTX_SPT(6)                               /*!< 71.5 sampling cycles */
#define ADC_SAMPLETIME_239POINT5         SAMPTX_SPT(7)                               /*!< 239.5 sampling cycles */

/* adc_ioffx register value */
#define IOFFX_IOFF(regval)               (BITS(0,11) & ((uint32_t)(regval) << 0))    /*!< write value to ADC_IOFFX_IOFF bit field */

/* adc_wdht register value */
#define WDHT_WDHT(regval)                (BITS(0,11) & ((uint32_t)(regval) << 0))    /*!< write value to ADC_WDHT_WDHT bit field */

/* adc_wdlt register value */
#define WDLT_WDLT(regval)                (BITS(0,11) & ((uint32_t)(regval) << 0))    /*!< write value to ADC_WDLT_WDLT bit field */

/* adc_rsqx register value */
#define RSQ0_RL(regval)                  (BITS(20,23) & ((uint32_t)(regval) << 20))  /*!< write value to ADC_RSQ0_RL bit field */

/* adc_isq register value */
#define ISQ_IL(regval)                   (BITS(20,21) & ((uint32_t)(regval) << 20))  /*!< write value to ADC_ISQ_IL bit field */

/* ADC channel group definitions */
#define ADC_REGULAR_CHANNEL              ((uint8_t)0x01U)                            /*!< adc regular channel group */
#define ADC_INSERTED_CHANNEL             ((uint8_t)0x02U)                            /*!< adc inserted channel group */
#define ADC_REGULAR_INSERTED_CHANNEL     ((uint8_t)0x03U)                            /*!< both regular and inserted channel group */

#define ADC_CHANNEL_DISCON_DISABLE       ((uint8_t)0x04U)                            /*!< disable discontinuous mode of regular & inserted channel */

/* ADC inserted channel definitions */
#define ADC_INSERTED_CHANNEL_0           ((uint8_t)0x00U)                            /*!< adc inserted channel 0 */
#define ADC_INSERTED_CHANNEL_1           ((uint8_t)0x01U)                            /*!< adc inserted channel 1 */
#define ADC_INSERTED_CHANNEL_2           ((uint8_t)0x02U)                            /*!< adc inserted channel 2 */
#define ADC_INSERTED_CHANNEL_3           ((uint8_t)0x03U)                            /*!< adc inserted channel 3 */

/* ADC channel definitions */
#define ADC_CHANNEL_0                    ((uint8_t)0x00U)                            /*!< ADC channel 0 */
#define ADC_CHANNEL_1                    ((uint8_t)0x01U)                            /*!< ADC channel 1 */
#define ADC_CHANNEL_2                    ((uint8_t)0x02U)                            /*!< ADC channel 2 */
#define ADC_CHANNEL_3                    ((uint8_t)0x03U)                            /*!< ADC channel 3 */
#define ADC_CHANNEL_4                    ((uint8_t)0x04U)                            /*!< ADC channel 4 */
#define ADC_CHANNEL_5                    ((uint8_t)0x05U)                            /*!< ADC channel 5 */
#define ADC_CHANNEL_6                    ((uint8_t)0x06U)                            /*!< ADC channel 6 */
#define ADC_CHANNEL_7                    ((uint8_t)0x07U)                            /*!< ADC channel 7 */
#define ADC_CHANNEL_8                    ((uint8_t)0x08U)                            /*!< ADC channel 8 */
#define ADC_CHANNEL_9                    ((uint8_t)0x09U)                            /*!< ADC channel 9 */
#define ADC_CHANNEL_10                   ((uint8_t)0x0AU)                            /*!< ADC channel 10 */
#define ADC_CHANNEL_11                   ((uint8_t)0x0BU)                            /*!< ADC channel 11 */
#define ADC_CHANNEL_12                   ((uint8_t)0x0CU)                            /*!< ADC channel 12 */
#define ADC_CHANNEL_13                   ((uint8_t)0x0DU)                            /*!< ADC channel 13 */
#define ADC_CHANNEL_14                   ((uint8_t)0x0EU)                            /*!< ADC channel 14 */
#define ADC_CHANNEL_15                   ((uint8_t)0x0FU)                            /*!< ADC channel 15 */
#define ADC_CHANNEL_16                   ((uint8_t)0x10U)                            /*!< ADC channel 16 */
#define ADC_CHANNEL_17                   ((uint8_t)0x11U)                            /*!< ADC channel 17 */

/* ADC interrupt */
#define ADC_INT_WDE                      ADC_STAT_WDE                                /*!< analog watchdog event interrupt */
#define ADC_INT_EOC                      ADC_STAT_EOC                                /*!< end of group conversion interrupt */
#define ADC_INT_EOIC                     ADC_STAT_EOIC                               /*!< end of inserted group conversion interrupt */

/* ADC interrupt flag */
#define ADC_INT_FLAG_WDE                 ADC_STAT_WDE                                /*!< analog watchdog event interrupt flag */
#define ADC_INT_FLAG_EOC                 ADC_STAT_EOC                                /*!< end of group conversion interrupt flag */
#define ADC_INT_FLAG_EOIC                ADC_STAT_EOIC                               /*!< end of inserted group conversion interrupt flag */

/* ADC resolution definitions */
#define OVSCR_DRES(regval)               (BITS(12,13) & ((uint32_t)(regval) << 12))
#define ADC_RESOLUTION_12B               OVSCR_DRES(0)                               /*!< 12-bit ADC resolution */
#define ADC_RESOLUTION_10B               OVSCR_DRES(1)                               /*!< 10-bit ADC resolution */
#define ADC_RESOLUTION_8B                OVSCR_DRES(2)                               /*!< 8-bit ADC resolution */
#define ADC_RESOLUTION_6B                OVSCR_DRES(3)                               /*!< 6-bit ADC resolution */

/* ADC oversampling mode */
#define ADC_OVERSAMPLING_ALL_CONVERT     0                                           /*!< all oversampled conversions for a channel are done consecutively after a trigger */
#define ADC_OVERSAMPLING_ONE_CONVERT     1                                           /*!< each oversampled conversion for a channel needs a trigger */

/* ADC oversampling shift */
#define OVSCR_OVSS(regval)               (BITS(5,8) & ((uint32_t)(regval) << 5))
#define ADC_OVERSAMPLING_SHIFT_NONE      OVSCR_OVSS(0)                               /*!< no oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_1B        OVSCR_OVSS(1)                               /*!< 1-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_2B        OVSCR_OVSS(2)                               /*!< 2-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_3B        OVSCR_OVSS(3)                               /*!< 3-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_4B        OVSCR_OVSS(4)                               /*!< 4-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_5B        OVSCR_OVSS(5)                               /*!< 5-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_6B        OVSCR_OVSS(6)                               /*!< 6-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_7B        OVSCR_OVSS(7)                               /*!< 7-bit oversampling shift */
#define ADC_OVERSAMPLING_SHIFT_8B        OVSCR_OVSS(8)                               /*!< 8-bit oversampling shift */

/* ADC oversampling ratio */
#define OVSCR_OVSR(regval)               (BITS(2,4) & ((uint32_t)(regval) << 2))
#define ADC_OVERSAMPLING_RATIO_MUL2      OVSCR_OVSR(0)                               /*!< oversampling ratio X2 */
#define ADC_OVERSAMPLING_RATIO_MUL4      OVSCR_OVSR(1)                               /*!< oversampling ratio X4 */
#define ADC_OVERSAMPLING_RATIO_MUL8      OVSCR_OVSR(2)                               /*!< oversampling ratio X8 */
#define ADC_OVERSAMPLING_RATIO_MUL16     OVSCR_OVSR(3)                               /*!< oversampling ratio X16 */
#define ADC_OVERSAMPLING_RATIO_MUL32     OVSCR_OVSR(4)                               /*!< oversampling ratio X32 */
#define ADC_OVERSAMPLING_RATIO_MUL64     OVSCR_OVSR(5)                               /*!< oversampling ratio X64 */
#define ADC_OVERSAMPLING_RATIO_MUL128    OVSCR_OVSR(6)                               /*!< oversampling ratio X128 */
#define ADC_OVERSAMPLING_RATIO_MUL256    OVSCR_OVSR(7)                               /*!< oversampling ratio X256 */


////////////////////////////////////////////////////////////////////////
/* BKP definitions */
#define BKP                             BKP_BASE                 /*!< BKP base address */

/* registers definitions */
#define BKP_DATA0                       REG16((BKP) + 0x04U)     /*!< BKP data register 0 */
#define BKP_DATA1                       REG16((BKP) + 0x08U)     /*!< BKP data register 1 */
#define BKP_DATA2                       REG16((BKP) + 0x0CU)     /*!< BKP data register 2 */
#define BKP_DATA3                       REG16((BKP) + 0x10U)     /*!< BKP data register 3 */
#define BKP_DATA4                       REG16((BKP) + 0x14U)     /*!< BKP data register 4 */
#define BKP_DATA5                       REG16((BKP) + 0x18U)     /*!< BKP data register 5 */
#define BKP_DATA6                       REG16((BKP) + 0x1CU)     /*!< BKP data register 6 */
#define BKP_DATA7                       REG16((BKP) + 0x20U)     /*!< BKP data register 7 */
#define BKP_DATA8                       REG16((BKP) + 0x24U)     /*!< BKP data register 8 */
#define BKP_DATA9                       REG16((BKP) + 0x28U)     /*!< BKP data register 9 */
#define BKP_DATA10                      REG16((BKP) + 0x40U)     /*!< BKP data register 10 */
#define BKP_DATA11                      REG16((BKP) + 0x44U)     /*!< BKP data register 11 */
#define BKP_DATA12                      REG16((BKP) + 0x48U)     /*!< BKP data register 12 */
#define BKP_DATA13                      REG16((BKP) + 0x4CU)     /*!< BKP data register 13 */
#define BKP_DATA14                      REG16((BKP) + 0x50U)     /*!< BKP data register 14 */
#define BKP_DATA15                      REG16((BKP) + 0x54U)     /*!< BKP data register 15 */
#define BKP_DATA16                      REG16((BKP) + 0x58U)     /*!< BKP data register 16 */
#define BKP_DATA17                      REG16((BKP) + 0x5CU)     /*!< BKP data register 17 */
#define BKP_DATA18                      REG16((BKP) + 0x60U)     /*!< BKP data register 18 */
#define BKP_DATA19                      REG16((BKP) + 0x64U)     /*!< BKP data register 19 */
#define BKP_DATA20                      REG16((BKP) + 0x68U)     /*!< BKP data register 20 */
#define BKP_DATA21                      REG16((BKP) + 0x6CU)     /*!< BKP data register 21 */
#define BKP_DATA22                      REG16((BKP) + 0x70U)     /*!< BKP data register 22 */
#define BKP_DATA23                      REG16((BKP) + 0x74U)     /*!< BKP data register 23 */
#define BKP_DATA24                      REG16((BKP) + 0x78U)     /*!< BKP data register 24 */
#define BKP_DATA25                      REG16((BKP) + 0x7CU)     /*!< BKP data register 25 */
#define BKP_DATA26                      REG16((BKP) + 0x80U)     /*!< BKP data register 26 */
#define BKP_DATA27                      REG16((BKP) + 0x84U)     /*!< BKP data register 27 */
#define BKP_DATA28                      REG16((BKP) + 0x88U)     /*!< BKP data register 28 */
#define BKP_DATA29                      REG16((BKP) + 0x8CU)     /*!< BKP data register 29 */
#define BKP_DATA30                      REG16((BKP) + 0x90U)     /*!< BKP data register 30 */
#define BKP_DATA31                      REG16((BKP) + 0x94U)     /*!< BKP data register 31 */
#define BKP_DATA32                      REG16((BKP) + 0x98U)     /*!< BKP data register 32 */
#define BKP_DATA33                      REG16((BKP) + 0x9CU)     /*!< BKP data register 33 */
#define BKP_DATA34                      REG16((BKP) + 0xA0U)     /*!< BKP data register 34 */
#define BKP_DATA35                      REG16((BKP) + 0xA4U)     /*!< BKP data register 35 */
#define BKP_DATA36                      REG16((BKP) + 0xA8U)     /*!< BKP data register 36 */
#define BKP_DATA37                      REG16((BKP) + 0xACU)     /*!< BKP data register 37 */
#define BKP_DATA38                      REG16((BKP) + 0xB0U)     /*!< BKP data register 38 */
#define BKP_DATA39                      REG16((BKP) + 0xB4U)     /*!< BKP data register 39 */
#define BKP_DATA40                      REG16((BKP) + 0xB8U)     /*!< BKP data register 40 */
#define BKP_DATA41                      REG16((BKP) + 0xBCU)     /*!< BKP data register 41 */
#define BKP_OCTL                        REG16((BKP) + 0x2CU)     /*!< RTC signal output control register */
#define BKP_TPCTL                       REG16((BKP) + 0x30U)     /*!< tamper pin control register */
#define BKP_TPCS                        REG16((BKP) + 0x34U)     /*!< tamper control and status register */

/* bits definitions */
/* BKP_DATA */
#define BKP_DATA                        BITS(0,15)               /*!< backup data */

/* BKP_OCTL */
#define BKP_OCTL_RCCV                   BITS(0,6)                /*!< RTC clock calibration value */
#define BKP_OCTL_COEN                   BIT(7)                   /*!< RTC clock calibration output enable */
#define BKP_OCTL_ASOEN                  BIT(8)                   /*!< RTC alarm or second signal output enable */
#define BKP_OCTL_ROSEL                  BIT(9)                   /*!< RTC output selection */

/* BKP_TPCTL */
#define BKP_TPCTL_TPEN                  BIT(0)                   /*!< tamper detection enable */
#define BKP_TPCTL_TPAL                  BIT(1)                   /*!< tamper pin active level */

/* BKP_TPCS */
#define BKP_TPCS_TER                    BIT(0)                   /*!< tamper event reset */
#define BKP_TPCS_TIR                    BIT(1)                   /*!< tamper interrupt reset */
#define BKP_TPCS_TPIE                   BIT(2)                   /*!< tamper interrupt enable */
#define BKP_TPCS_TEF                    BIT(8)                   /*!< tamper event flag */
#define BKP_TPCS_TIF                    BIT(9)                   /*!< tamper interrupt flag */

/* constants definitions */
/* BKP data register number */
typedef enum 
{
    BKP_DATA_0 = 1,                                              /*!< BKP data register 0 */
    BKP_DATA_1,                                                  /*!< BKP data register 1 */
    BKP_DATA_2,                                                  /*!< BKP data register 2 */
    BKP_DATA_3,                                                  /*!< BKP data register 3 */
    BKP_DATA_4,                                                  /*!< BKP data register 4 */
    BKP_DATA_5,                                                  /*!< BKP data register 5 */
    BKP_DATA_6,                                                  /*!< BKP data register 6 */
    BKP_DATA_7,                                                  /*!< BKP data register 7 */
    BKP_DATA_8,                                                  /*!< BKP data register 8 */
    BKP_DATA_9,                                                  /*!< BKP data register 9 */
    BKP_DATA_10,                                                 /*!< BKP data register 10 */
    BKP_DATA_11,                                                 /*!< BKP data register 11 */
    BKP_DATA_12,                                                 /*!< BKP data register 12 */
    BKP_DATA_13,                                                 /*!< BKP data register 13 */
    BKP_DATA_14,                                                 /*!< BKP data register 14 */
    BKP_DATA_15,                                                 /*!< BKP data register 15 */
    BKP_DATA_16,                                                 /*!< BKP data register 16 */
    BKP_DATA_17,                                                 /*!< BKP data register 17 */
    BKP_DATA_18,                                                 /*!< BKP data register 18 */
    BKP_DATA_19,                                                 /*!< BKP data register 19 */
    BKP_DATA_20,                                                 /*!< BKP data register 20 */
    BKP_DATA_21,                                                 /*!< BKP data register 21 */
    BKP_DATA_22,                                                 /*!< BKP data register 22 */
    BKP_DATA_23,                                                 /*!< BKP data register 23 */
    BKP_DATA_24,                                                 /*!< BKP data register 24 */
    BKP_DATA_25,                                                 /*!< BKP data register 25 */
    BKP_DATA_26,                                                 /*!< BKP data register 26 */
    BKP_DATA_27,                                                 /*!< BKP data register 27 */
    BKP_DATA_28,                                                 /*!< BKP data register 28 */
    BKP_DATA_29,                                                 /*!< BKP data register 29 */
    BKP_DATA_30,                                                 /*!< BKP data register 30 */
    BKP_DATA_31,                                                 /*!< BKP data register 31 */
    BKP_DATA_32,                                                 /*!< BKP data register 32 */
    BKP_DATA_33,                                                 /*!< BKP data register 33 */
    BKP_DATA_34,                                                 /*!< BKP data register 34 */
    BKP_DATA_35,                                                 /*!< BKP data register 35 */
    BKP_DATA_36,                                                 /*!< BKP data register 36 */
    BKP_DATA_37,                                                 /*!< BKP data register 37 */
    BKP_DATA_38,                                                 /*!< BKP data register 38 */
    BKP_DATA_39,                                                 /*!< BKP data register 39 */
    BKP_DATA_40,                                                 /*!< BKP data register 40 */
    BKP_DATA_41,                                                 /*!< BKP data register 41 */
}bkp_data_register_enum;

/* BKP register */
#define BKP_DATA0_9(number)             REG16((BKP) + 0x04U + (number) * 0x04U)
#define BKP_DATA10_41(number)           REG16((BKP) + 0x40U + ((number)-10U) * 0x04U)

/* get data of BKP data register */
#define BKP_DATA_GET(regval)            GET_BITS((uint32_t)(regval), 0, 15)

/* RTC clock calibration value */
#define OCTL_RCCV(regval)               (BITS(0,6) & ((uint32_t)(regval) << 0))

/* RTC output selection */
#define RTC_OUTPUT_ALARM_PULSE          ((uint16_t)0x0000U)      /*!< RTC alarm pulse is selected as the RTC output */
#define RTC_OUTPUT_SECOND_PULSE         ((uint16_t)0x0200U)      /*!< RTC second pulse is selected as the RTC output */

/* tamper pin active level */
#define TAMPER_PIN_ACTIVE_HIGH          ((uint16_t)0x0000U)      /*!< the tamper pin is active high */
#define TAMPER_PIN_ACTIVE_LOW           ((uint16_t)0x0002U)      /*!< the tamper pin is active low */

/* tamper flag */
#define BKP_FLAG_TAMPER                 BKP_TPCS_TEF             /*!< tamper event flag */

/* tamper interrupt flag */
#define BKP_INT_FLAG_TAMPER             BKP_TPCS_TIF             /*!< tamper interrupt flag */


////////////////////////////////////////////////////////////////////////
/* CAN definitions */
#define CAN0                               CAN_BASE                      /*!< CAN0 base address */
#define CAN1                               (CAN0 + 0x00000400U)          /*!< CAN1 base address */

/* registers definitions */
#define CAN_CTL(canx)                      REG32((canx) + 0x00U)         /*!< CAN control register */
#define CAN_STAT(canx)                     REG32((canx) + 0x04U)         /*!< CAN status register */
#define CAN_TSTAT(canx)                    REG32((canx) + 0x08U)         /*!< CAN transmit status register*/
#define CAN_RFIFO0(canx)                   REG32((canx) + 0x0CU)         /*!< CAN receive FIFO0 register */
#define CAN_RFIFO1(canx)                   REG32((canx) + 0x10U)         /*!< CAN receive FIFO1 register */
#define CAN_INTEN(canx)                    REG32((canx) + 0x14U)         /*!< CAN interrupt enable register */
#define CAN_ERR(canx)                      REG32((canx) + 0x18U)         /*!< CAN error register */
#define CAN_BT(canx)                       REG32((canx) + 0x1CU)         /*!< CAN bit timing register */
#define CAN_TMI0(canx)                     REG32((canx) + 0x180U)        /*!< CAN transmit mailbox0 identifier register */
#define CAN_TMP0(canx)                     REG32((canx) + 0x184U)        /*!< CAN transmit mailbox0 property register */
#define CAN_TMDATA00(canx)                 REG32((canx) + 0x188U)        /*!< CAN transmit mailbox0 data0 register */
#define CAN_TMDATA10(canx)                 REG32((canx) + 0x18CU)        /*!< CAN transmit mailbox0 data1 register */
#define CAN_TMI1(canx)                     REG32((canx) + 0x190U)        /*!< CAN transmit mailbox1 identifier register */
#define CAN_TMP1(canx)                     REG32((canx) + 0x194U)        /*!< CAN transmit mailbox1 property register */
#define CAN_TMDATA01(canx)                 REG32((canx) + 0x198U)        /*!< CAN transmit mailbox1 data0 register */
#define CAN_TMDATA11(canx)                 REG32((canx) + 0x19CU)        /*!< CAN transmit mailbox1 data1 register */
#define CAN_TMI2(canx)                     REG32((canx) + 0x1A0U)        /*!< CAN transmit mailbox2 identifier register */
#define CAN_TMP2(canx)                     REG32((canx) + 0x1A4U)        /*!< CAN transmit mailbox2 property register */
#define CAN_TMDATA02(canx)                 REG32((canx) + 0x1A8U)        /*!< CAN transmit mailbox2 data0 register */
#define CAN_TMDATA12(canx)                 REG32((canx) + 0x1ACU)        /*!< CAN transmit mailbox2 data1 register */
#define CAN_RFIFOMI0(canx)                 REG32((canx) + 0x1B0U)        /*!< CAN receive FIFO0 mailbox identifier register */
#define CAN_RFIFOMP0(canx)                 REG32((canx) + 0x1B4U)        /*!< CAN receive FIFO0 mailbox property register */
#define CAN_RFIFOMDATA00(canx)             REG32((canx) + 0x1B8U)        /*!< CAN receive FIFO0 mailbox data0 register */
#define CAN_RFIFOMDATA10(canx)             REG32((canx) + 0x1BCU)        /*!< CAN receive FIFO0 mailbox data1 register */
#define CAN_RFIFOMI1(canx)                 REG32((canx) + 0x1C0U)        /*!< CAN receive FIFO1 mailbox identifier register */
#define CAN_RFIFOMP1(canx)                 REG32((canx) + 0x1C4U)        /*!< CAN receive FIFO1 mailbox property register */
#define CAN_RFIFOMDATA01(canx)             REG32((canx) + 0x1C8U)        /*!< CAN receive FIFO1 mailbox data0 register */
#define CAN_RFIFOMDATA11(canx)             REG32((canx) + 0x1CCU)        /*!< CAN receive FIFO1 mailbox data1 register */
#define CAN_FCTL(canx)                     REG32((canx) + 0x200U)        /*!< CAN filter control register */
#define CAN_FMCFG(canx)                    REG32((canx) + 0x204U)        /*!< CAN filter mode register */
#define CAN_FSCFG(canx)                    REG32((canx) + 0x20CU)        /*!< CAN filter scale register */
#define CAN_FAFIFO(canx)                   REG32((canx) + 0x214U)        /*!< CAN filter associated FIFO register */
#define CAN_FW(canx)                       REG32((canx) + 0x21CU)        /*!< CAN filter working register */
#define CAN_F0DATA0(canx)                  REG32((canx) + 0x240U)        /*!< CAN filter 0 data 0 register */
#define CAN_F1DATA0(canx)                  REG32((canx) + 0x248U)        /*!< CAN filter 1 data 0 register */
#define CAN_F2DATA0(canx)                  REG32((canx) + 0x250U)        /*!< CAN filter 2 data 0 register */
#define CAN_F3DATA0(canx)                  REG32((canx) + 0x258U)        /*!< CAN filter 3 data 0 register */
#define CAN_F4DATA0(canx)                  REG32((canx) + 0x260U)        /*!< CAN filter 4 data 0 register */
#define CAN_F5DATA0(canx)                  REG32((canx) + 0x268U)        /*!< CAN filter 5 data 0 register */
#define CAN_F6DATA0(canx)                  REG32((canx) + 0x270U)        /*!< CAN filter 6 data 0 register */
#define CAN_F7DATA0(canx)                  REG32((canx) + 0x278U)        /*!< CAN filter 7 data 0 register */
#define CAN_F8DATA0(canx)                  REG32((canx) + 0x280U)        /*!< CAN filter 8 data 0 register */
#define CAN_F9DATA0(canx)                  REG32((canx) + 0x288U)        /*!< CAN filter 9 data 0 register */
#define CAN_F10DATA0(canx)                 REG32((canx) + 0x290U)        /*!< CAN filter 10 data 0 register */
#define CAN_F11DATA0(canx)                 REG32((canx) + 0x298U)        /*!< CAN filter 11 data 0 register */
#define CAN_F12DATA0(canx)                 REG32((canx) + 0x2A0U)        /*!< CAN filter 12 data 0 register */
#define CAN_F13DATA0(canx)                 REG32((canx) + 0x2A8U)        /*!< CAN filter 13 data 0 register */
#define CAN_F14DATA0(canx)                 REG32((canx) + 0x2B0U)        /*!< CAN filter 14 data 0 register */
#define CAN_F15DATA0(canx)                 REG32((canx) + 0x2B8U)        /*!< CAN filter 15 data 0 register */
#define CAN_F16DATA0(canx)                 REG32((canx) + 0x2C0U)        /*!< CAN filter 16 data 0 register */
#define CAN_F17DATA0(canx)                 REG32((canx) + 0x2C8U)        /*!< CAN filter 17 data 0 register */
#define CAN_F18DATA0(canx)                 REG32((canx) + 0x2D0U)        /*!< CAN filter 18 data 0 register */
#define CAN_F19DATA0(canx)                 REG32((canx) + 0x2D8U)        /*!< CAN filter 19 data 0 register */
#define CAN_F20DATA0(canx)                 REG32((canx) + 0x2E0U)        /*!< CAN filter 20 data 0 register */
#define CAN_F21DATA0(canx)                 REG32((canx) + 0x2E8U)        /*!< CAN filter 21 data 0 register */
#define CAN_F22DATA0(canx)                 REG32((canx) + 0x2F0U)        /*!< CAN filter 22 data 0 register */
#define CAN_F23DATA0(canx)                 REG32((canx) + 0x3F8U)        /*!< CAN filter 23 data 0 register */
#define CAN_F24DATA0(canx)                 REG32((canx) + 0x300U)        /*!< CAN filter 24 data 0 register */
#define CAN_F25DATA0(canx)                 REG32((canx) + 0x308U)        /*!< CAN filter 25 data 0 register */
#define CAN_F26DATA0(canx)                 REG32((canx) + 0x310U)        /*!< CAN filter 26 data 0 register */
#define CAN_F27DATA0(canx)                 REG32((canx) + 0x318U)        /*!< CAN filter 27 data 0 register */
#define CAN_F0DATA1(canx)                  REG32((canx) + 0x244U)        /*!< CAN filter 0 data 1 register */
#define CAN_F1DATA1(canx)                  REG32((canx) + 0x24CU)        /*!< CAN filter 1 data 1 register */
#define CAN_F2DATA1(canx)                  REG32((canx) + 0x254U)        /*!< CAN filter 2 data 1 register */
#define CAN_F3DATA1(canx)                  REG32((canx) + 0x25CU)        /*!< CAN filter 3 data 1 register */
#define CAN_F4DATA1(canx)                  REG32((canx) + 0x264U)        /*!< CAN filter 4 data 1 register */
#define CAN_F5DATA1(canx)                  REG32((canx) + 0x26CU)        /*!< CAN filter 5 data 1 register */
#define CAN_F6DATA1(canx)                  REG32((canx) + 0x274U)        /*!< CAN filter 6 data 1 register */
#define CAN_F7DATA1(canx)                  REG32((canx) + 0x27CU)        /*!< CAN filter 7 data 1 register */
#define CAN_F8DATA1(canx)                  REG32((canx) + 0x284U)        /*!< CAN filter 8 data 1 register */
#define CAN_F9DATA1(canx)                  REG32((canx) + 0x28CU)        /*!< CAN filter 9 data 1 register */
#define CAN_F10DATA1(canx)                 REG32((canx) + 0x294U)        /*!< CAN filter 10 data 1 register */
#define CAN_F11DATA1(canx)                 REG32((canx) + 0x29CU)        /*!< CAN filter 11 data 1 register */
#define CAN_F12DATA1(canx)                 REG32((canx) + 0x2A4U)        /*!< CAN filter 12 data 1 register */
#define CAN_F13DATA1(canx)                 REG32((canx) + 0x2ACU)        /*!< CAN filter 13 data 1 register */
#define CAN_F14DATA1(canx)                 REG32((canx) + 0x2B4U)        /*!< CAN filter 14 data 1 register */
#define CAN_F15DATA1(canx)                 REG32((canx) + 0x2BCU)        /*!< CAN filter 15 data 1 register */
#define CAN_F16DATA1(canx)                 REG32((canx) + 0x2C4U)        /*!< CAN filter 16 data 1 register */
#define CAN_F17DATA1(canx)                 REG32((canx) + 0x24CU)        /*!< CAN filter 17 data 1 register */
#define CAN_F18DATA1(canx)                 REG32((canx) + 0x2D4U)        /*!< CAN filter 18 data 1 register */
#define CAN_F19DATA1(canx)                 REG32((canx) + 0x2DCU)        /*!< CAN filter 19 data 1 register */
#define CAN_F20DATA1(canx)                 REG32((canx) + 0x2E4U)        /*!< CAN filter 20 data 1 register */
#define CAN_F21DATA1(canx)                 REG32((canx) + 0x2ECU)        /*!< CAN filter 21 data 1 register */
#define CAN_F22DATA1(canx)                 REG32((canx) + 0x2F4U)        /*!< CAN filter 22 data 1 register */
#define CAN_F23DATA1(canx)                 REG32((canx) + 0x2FCU)        /*!< CAN filter 23 data 1 register */
#define CAN_F24DATA1(canx)                 REG32((canx) + 0x304U)        /*!< CAN filter 24 data 1 register */
#define CAN_F25DATA1(canx)                 REG32((canx) + 0x30CU)        /*!< CAN filter 25 data 1 register */
#define CAN_F26DATA1(canx)                 REG32((canx) + 0x314U)        /*!< CAN filter 26 data 1 register */
#define CAN_F27DATA1(canx)                 REG32((canx) + 0x31CU)        /*!< CAN filter 27 data 1 register */

/* CAN transmit mailbox bank */
#define CAN_TMI(canx, bank)                REG32((canx) + 0x180U + ((bank) * 0x10U))        /*!< CAN transmit mailbox identifier register */
#define CAN_TMP(canx, bank)                REG32((canx) + 0x184U + ((bank) * 0x10U))        /*!< CAN transmit mailbox property register */
#define CAN_TMDATA0(canx, bank)            REG32((canx) + 0x188U + ((bank) * 0x10U))        /*!< CAN transmit mailbox data0 register */
#define CAN_TMDATA1(canx, bank)            REG32((canx) + 0x18CU + ((bank) * 0x10U))        /*!< CAN transmit mailbox data1 register */

/* CAN filter bank */
#define CAN_FDATA0(canx, bank)             REG32((canx) + 0x240U + ((bank) * 0x8U) + 0x0U)  /*!< CAN filter data 0 register */
#define CAN_FDATA1(canx, bank)             REG32((canx) + 0x240U + ((bank) * 0x8U) + 0x4U)  /*!< CAN filter data 1 register */

/* CAN receive fifo mailbox bank */
#define CAN_RFIFOMI(canx, bank)            REG32((canx) + 0x1B0U + ((bank) * 0x10U))        /*!< CAN receive FIFO mailbox identifier register */
#define CAN_RFIFOMP(canx, bank)            REG32((canx) + 0x1B4U + ((bank) * 0x10U))        /*!< CAN receive FIFO mailbox property register */
#define CAN_RFIFOMDATA0(canx, bank)        REG32((canx) + 0x1B8U + ((bank) * 0x10U))        /*!< CAN receive FIFO mailbox data0 register */
#define CAN_RFIFOMDATA1(canx, bank)        REG32((canx) + 0x1BCU + ((bank) * 0x10U))        /*!< CAN receive FIFO mailbox data1 register */

/* bits definitions */
/* CAN_CTL */
#define CAN_CTL_IWMOD                      BIT(0)                       /*!< initial working mode */
#define CAN_CTL_SLPWMOD                    BIT(1)                       /*!< sleep working mode */
#define CAN_CTL_TFO                        BIT(2)                       /*!< transmit FIFO order */
#define CAN_CTL_RFOD                       BIT(3)                       /*!< receive FIFO overwrite disable */
#define CAN_CTL_ARD                        BIT(4)                       /*!< automatic retransmission disable */
#define CAN_CTL_AWU                        BIT(5)                       /*!< automatic wakeup */
#define CAN_CTL_ABOR                       BIT(6)                       /*!< automatic bus-off recovery */
#define CAN_CTL_TTC                        BIT(7)                       /*!< time triggered communication */
#define CAN_CTL_SWRST                      BIT(15)                      /*!< CAN software reset */
#define CAN_CTL_DFZ                        BIT(16)                      /*!< CAN debug freeze */

/* CAN_STAT */
#define CAN_STAT_IWS                       BIT(0)                       /*!< initial working state */
#define CAN_STAT_SLPWS                     BIT(1)                       /*!< sleep working state */
#define CAN_STAT_ERRIF                     BIT(2)                       /*!< error interrupt flag*/
#define CAN_STAT_WUIF                      BIT(3)                       /*!< status change interrupt flag of wakeup from sleep working mode */
#define CAN_STAT_SLPIF                     BIT(4)                       /*!< status change interrupt flag of sleep working mode entering */
#define CAN_STAT_TS                        BIT(8)                       /*!< transmitting state */
#define CAN_STAT_RS                        BIT(9)                       /*!< receiving state */
#define CAN_STAT_LASTRX                    BIT(10)                      /*!< last sample value of rx pin */
#define CAN_STAT_RXL                       BIT(11)                      /*!< CAN rx signal */

/* CAN_TSTAT */
#define CAN_TSTAT_MTF0                     BIT(0)                       /*!< mailbox0 transmit finished */
#define CAN_TSTAT_MTFNERR0                 BIT(1)                       /*!< mailbox0 transmit finished and no error */
#define CAN_TSTAT_MAL0                     BIT(2)                       /*!< mailbox0 arbitration lost */
#define CAN_TSTAT_MTE0                     BIT(3)                       /*!< mailbox0 transmit error */
#define CAN_TSTAT_MST0                     BIT(7)                       /*!< mailbox0 stop transmitting */
#define CAN_TSTAT_MTF1                     BIT(8)                       /*!< mailbox1 transmit finished */
#define CAN_TSTAT_MTFNERR1                 BIT(9)                       /*!< mailbox1 transmit finished and no error */
#define CAN_TSTAT_MAL1                     BIT(10)                      /*!< mailbox1 arbitration lost */
#define CAN_TSTAT_MTE1                     BIT(11)                      /*!< mailbox1 transmit error */
#define CAN_TSTAT_MST1                     BIT(15)                      /*!< mailbox1 stop transmitting */
#define CAN_TSTAT_MTF2                     BIT(16)                      /*!< mailbox2 transmit finished */
#define CAN_TSTAT_MTFNERR2                 BIT(17)                      /*!< mailbox2 transmit finished and no error */
#define CAN_TSTAT_MAL2                     BIT(18)                      /*!< mailbox2 arbitration lost */
#define CAN_TSTAT_MTE2                     BIT(19)                      /*!< mailbox2 transmit error */
#define CAN_TSTAT_MST2                     BIT(23)                      /*!< mailbox2 stop transmitting */
#define CAN_TSTAT_NUM                      BITS(24,25)                  /*!< mailbox number */
#define CAN_TSTAT_TME0                     BIT(26)                      /*!< transmit mailbox0 empty */
#define CAN_TSTAT_TME1                     BIT(27)                      /*!< transmit mailbox1 empty */
#define CAN_TSTAT_TME2                     BIT(28)                      /*!< transmit mailbox2 empty */
#define CAN_TSTAT_TMLS0                    BIT(29)                      /*!< last sending priority flag for mailbox0 */
#define CAN_TSTAT_TMLS1                    BIT(30)                      /*!< last sending priority flag for mailbox1 */
#define CAN_TSTAT_TMLS2                    BIT(31)                      /*!< last sending priority flag for mailbox2 */

/* CAN_RFIFO0 */
#define CAN_RFIFO0_RFL0                    BITS(0,1)                    /*!< receive FIFO0 length */
#define CAN_RFIFO0_RFF0                    BIT(3)                       /*!< receive FIFO0 full */
#define CAN_RFIFO0_RFO0                    BIT(4)                       /*!< receive FIFO0 overfull */
#define CAN_RFIFO0_RFD0                    BIT(5)                       /*!< receive FIFO0 dequeue */

/* CAN_RFIFO1 */
#define CAN_RFIFO1_RFL1                    BITS(0,1)                    /*!< receive FIFO1 length */
#define CAN_RFIFO1_RFF1                    BIT(3)                       /*!< receive FIFO1 full */
#define CAN_RFIFO1_RFO1                    BIT(4)                       /*!< receive FIFO1 overfull */
#define CAN_RFIFO1_RFD1                    BIT(5)                       /*!< receive FIFO1 dequeue */

/* CAN_INTEN */
#define CAN_INTEN_TMEIE                    BIT(0)                       /*!< transmit mailbox empty interrupt enable */
#define CAN_INTEN_RFNEIE0                  BIT(1)                       /*!< receive FIFO0 not empty interrupt enable */
#define CAN_INTEN_RFFIE0                   BIT(2)                       /*!< receive FIFO0 full interrupt enable */
#define CAN_INTEN_RFOIE0                   BIT(3)                       /*!< receive FIFO0 overfull interrupt enable */
#define CAN_INTEN_RFNEIE1                  BIT(4)                       /*!< receive FIFO1 not empty interrupt enable */
#define CAN_INTEN_RFFIE1                   BIT(5)                       /*!< receive FIFO1 full interrupt enable */
#define CAN_INTEN_RFOIE1                   BIT(6)                       /*!< receive FIFO1 overfull interrupt enable */
#define CAN_INTEN_WERRIE                   BIT(8)                       /*!< warning error interrupt enable */
#define CAN_INTEN_PERRIE                   BIT(9)                       /*!< passive error interrupt enable */
#define CAN_INTEN_BOIE                     BIT(10)                      /*!< bus-off interrupt enable */
#define CAN_INTEN_ERRNIE                   BIT(11)                      /*!< error number interrupt enable */
#define CAN_INTEN_ERRIE                    BIT(15)                      /*!< error interrupt enable */
#define CAN_INTEN_WIE                      BIT(16)                      /*!< wakeup interrupt enable */
#define CAN_INTEN_SLPWIE                   BIT(17)                      /*!< sleep working interrupt enable */

/* CAN_ERR */
#define CAN_ERR_WERR                       BIT(0)                       /*!< warning error */
#define CAN_ERR_PERR                       BIT(1)                       /*!< passive error */
#define CAN_ERR_BOERR                      BIT(2)                       /*!< bus-off error */
#define CAN_ERR_ERRN                       BITS(4,6)                    /*!< error number */
#define CAN_ERR_TECNT                      BITS(16,23)                  /*!< transmit error count */
#define CAN_ERR_RECNT                      BITS(24,31)                  /*!< receive error count */

/* CAN_BT */
#define CAN_BT_BAUDPSC                     BITS(0,9)                    /*!< baudrate prescaler */
#define CAN_BT_BS1                         BITS(16,19)                  /*!< bit segment 1 */
#define CAN_BT_BS2                         BITS(20,22)                  /*!< bit segment 2 */
#define CAN_BT_SJW                         BITS(24,25)                  /*!< resynchronization jump width */
#define CAN_BT_LCMOD                       BIT(30)                      /*!< loopback communication mode */
#define CAN_BT_SCMOD                       BIT(31)                      /*!< silent communication mode */

/* CAN_TMIx */
#define CAN_TMI_TEN                        BIT(0)                       /*!< transmit enable */
#define CAN_TMI_FT                         BIT(1)                       /*!< frame type */
#define CAN_TMI_FF                         BIT(2)                       /*!< frame format */
#define CAN_TMI_EFID                       BITS(3,31)                   /*!< the frame identifier */
#define CAN_TMI_SFID                       BITS(21,31)                  /*!< the frame identifier */

/* CAN_TMPx */
#define CAN_TMP_DLENC                      BITS(0,3)                    /*!< data length code */
#define CAN_TMP_TSEN                       BIT(8)                       /*!< time stamp enable */
#define CAN_TMP_TS                         BITS(16,31)                  /*!< time stamp */

/* CAN_TMDATA0x */
#define CAN_TMDATA0_DB0                    BITS(0,7)                    /*!< transmit data byte 0 */
#define CAN_TMDATA0_DB1                    BITS(8,15)                   /*!< transmit data byte 1 */
#define CAN_TMDATA0_DB2                    BITS(16,23)                  /*!< transmit data byte 2 */
#define CAN_TMDATA0_DB3                    BITS(24,31)                  /*!< transmit data byte 3 */

/* CAN_TMDATA1x */
#define CAN_TMDATA1_DB4                    BITS(0,7)                    /*!< transmit data byte 4 */
#define CAN_TMDATA1_DB5                    BITS(8,15)                   /*!< transmit data byte 5 */
#define CAN_TMDATA1_DB6                    BITS(16,23)                  /*!< transmit data byte 6 */
#define CAN_TMDATA1_DB7                    BITS(24,31)                  /*!< transmit data byte 7 */

/* CAN_RFIFOMIx */
#define CAN_RFIFOMI_FT                     BIT(1)                       /*!< frame type */
#define CAN_RFIFOMI_FF                     BIT(2)                       /*!< frame format */
#define CAN_RFIFOMI_EFID                   BITS(3,31)                   /*!< the frame identifier */
#define CAN_RFIFOMI_SFID                   BITS(21,31)                  /*!< the frame identifier */

/* CAN_RFIFOMPx */
#define CAN_RFIFOMP_DLENC                  BITS(0,3)                    /*!< receive data length code */
#define CAN_RFIFOMP_FI                     BITS(8,15)                   /*!< filter index */
#define CAN_RFIFOMP_TS                     BITS(16,31)                  /*!< time stamp */

/* CAN_RFIFOMDATA0x */
#define CAN_RFIFOMDATA0_DB0                BITS(0,7)                    /*!< receive data byte 0 */
#define CAN_RFIFOMDATA0_DB1                BITS(8,15)                   /*!< receive data byte 1 */
#define CAN_RFIFOMDATA0_DB2                BITS(16,23)                  /*!< receive data byte 2 */
#define CAN_RFIFOMDATA0_DB3                BITS(24,31)                  /*!< receive data byte 3 */

/* CAN_RFIFOMDATA1x */
#define CAN_RFIFOMDATA1_DB4                BITS(0,7)                    /*!< receive data byte 4 */
#define CAN_RFIFOMDATA1_DB5                BITS(8,15)                   /*!< receive data byte 5 */
#define CAN_RFIFOMDATA1_DB6                BITS(16,23)                  /*!< receive data byte 6 */
#define CAN_RFIFOMDATA1_DB7                BITS(24,31)                  /*!< receive data byte 7 */

/* CAN_FCTL */
#define CAN_FCTL_FLD                       BIT(0)                       /*!< filter lock disable */
#define CAN_FCTL_HBC1F                     BITS(8,13)                   /*!< header bank of CAN1 filter */

/* CAN_FMCFG */
#define CAN_FMCFG_FMOD(regval)             BIT(regval)                  /*!< filter mode, list or mask*/

/* CAN_FSCFG */
#define CAN_FSCFG_FS(regval)               BIT(regval)                  /*!< filter scale, 32 bits or 16 bits*/

/* CAN_FAFIFO */
#define CAN_FAFIFOR_FAF(regval)            BIT(regval)                  /*!< filter associated with FIFO */

/* CAN_FW */
#define CAN_FW_FW(regval)                  BIT(regval)                  /*!< filter working */

/* CAN_FxDATAy */
#define CAN_FDATA_FD(regval)               BIT(regval)                  /*!< filter data */

/* consts definitions */
/* define the CAN bit position and its register index offset */
#define CAN_REGIDX_BIT(regidx, bitpos)              (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define CAN_REG_VAL(canx, offset)                   (REG32((canx) + ((uint32_t)(offset) >> 6)))
#define CAN_BIT_POS(val)                            ((uint32_t)(val) & 0x1FU)

#define CAN_REGIDX_BITS(regidx, bitpos0, bitpos1)   (((uint32_t)(regidx) << 12) | ((uint32_t)(bitpos0) << 6) | (uint32_t)(bitpos1))
#define CAN_REG_VALS(canx, offset)                  (REG32((canx) + ((uint32_t)(offset) >> 12)))
#define CAN_BIT_POS0(val)                           (((uint32_t)(val) >> 6) & 0x1FU)
#define CAN_BIT_POS1(val)                           ((uint32_t)(val) & 0x1FU)

/* register offset */
#define STAT_REG_OFFSET                    ((uint8_t)0x04U)             /*!< STAT register offset */
#define TSTAT_REG_OFFSET                   ((uint8_t)0x08U)             /*!< TSTAT register offset */
#define RFIFO0_REG_OFFSET                  ((uint8_t)0x0CU)             /*!< RFIFO0 register offset */
#define RFIFO1_REG_OFFSET                  ((uint8_t)0x10U)             /*!< RFIFO1 register offset */
#define ERR_REG_OFFSET                     ((uint8_t)0x18U)             /*!< ERR register offset */

/* CAN flags */
typedef enum
{
    /* flags in STAT register */
    CAN_FLAG_RXL      = CAN_REGIDX_BIT(STAT_REG_OFFSET, 11U),           /*!< RX level */ 
    CAN_FLAG_LASTRX   = CAN_REGIDX_BIT(STAT_REG_OFFSET, 10U),           /*!< last sample value of RX pin */ 
    CAN_FLAG_RS       = CAN_REGIDX_BIT(STAT_REG_OFFSET, 9U),            /*!< receiving state */ 
    CAN_FLAG_TS       = CAN_REGIDX_BIT(STAT_REG_OFFSET, 8U),            /*!< transmitting state */ 
    CAN_FLAG_SLPIF    = CAN_REGIDX_BIT(STAT_REG_OFFSET, 4U),            /*!< status change flag of entering sleep working mode */ 
    CAN_FLAG_WUIF     = CAN_REGIDX_BIT(STAT_REG_OFFSET, 3U),            /*!< status change flag of wakeup from sleep working mode */ 
    CAN_FLAG_ERRIF    = CAN_REGIDX_BIT(STAT_REG_OFFSET, 2U),            /*!< error flag */ 
    CAN_FLAG_SLPWS    = CAN_REGIDX_BIT(STAT_REG_OFFSET, 1U),            /*!< sleep working state */ 
    CAN_FLAG_IWS      = CAN_REGIDX_BIT(STAT_REG_OFFSET, 0U),            /*!< initial working state */ 
    /* flags in TSTAT register */
    CAN_FLAG_TMLS2    = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 31U),          /*!< transmit mailbox 2 last sending in Tx FIFO */ 
    CAN_FLAG_TMLS1    = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 30U),          /*!< transmit mailbox 1 last sending in Tx FIFO */ 
    CAN_FLAG_TMLS0    = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 29U),          /*!< transmit mailbox 0 last sending in Tx FIFO */ 
    CAN_FLAG_TME2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 28U),          /*!< transmit mailbox 2 empty */ 
    CAN_FLAG_TME1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 27U),          /*!< transmit mailbox 1 empty */ 
    CAN_FLAG_TME0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 26U),          /*!< transmit mailbox 0 empty */ 
    CAN_FLAG_MTE2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 19U),          /*!< mailbox 2 transmit error */ 
    CAN_FLAG_MTE1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 11U),          /*!< mailbox 1 transmit error */ 
    CAN_FLAG_MTE0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 3U),           /*!< mailbox 0 transmit error */ 
    CAN_FLAG_MAL2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 18U),          /*!< mailbox 2 arbitration lost */ 
    CAN_FLAG_MAL1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 10U),          /*!< mailbox 1 arbitration lost */ 
    CAN_FLAG_MAL0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 2U),           /*!< mailbox 0 arbitration lost */ 
    CAN_FLAG_MTFNERR2 = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 17U),          /*!< mailbox 2 transmit finished with no error */ 
    CAN_FLAG_MTFNERR1 = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 9U),           /*!< mailbox 1 transmit finished with no error */ 
    CAN_FLAG_MTFNERR0 = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 1U),           /*!< mailbox 0 transmit finished with no error */ 
    CAN_FLAG_MTF2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 16U),          /*!< mailbox 2 transmit finished */ 
    CAN_FLAG_MTF1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 8U),           /*!< mailbox 1 transmit finished */ 
    CAN_FLAG_MTF0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 0U),           /*!< mailbox 0 transmit finished */ 
    /* flags in RFIFO0 register */
    CAN_FLAG_RFO0     = CAN_REGIDX_BIT(RFIFO0_REG_OFFSET, 4U),          /*!< receive FIFO0 overfull */ 
    CAN_FLAG_RFF0     = CAN_REGIDX_BIT(RFIFO0_REG_OFFSET, 3U),          /*!< receive FIFO0 full */ 
    /* flags in RFIFO1 register */
    CAN_FLAG_RFO1     = CAN_REGIDX_BIT(RFIFO1_REG_OFFSET, 4U),          /*!< receive FIFO1 overfull */ 
    CAN_FLAG_RFF1     = CAN_REGIDX_BIT(RFIFO1_REG_OFFSET, 3U),          /*!< receive FIFO1 full */ 
    /* flags in ERR register */
    CAN_FLAG_BOERR    = CAN_REGIDX_BIT(ERR_REG_OFFSET, 2U),             /*!< bus-off error */ 
    CAN_FLAG_PERR     = CAN_REGIDX_BIT(ERR_REG_OFFSET, 1U),             /*!< passive error */ 
    CAN_FLAG_WERR     = CAN_REGIDX_BIT(ERR_REG_OFFSET, 0U),             /*!< warning error */ 
}can_flag_enum;

/* CAN interrupt flags */
typedef enum
{
    /* interrupt flags in STAT register */
    CAN_INT_FLAG_SLPIF = CAN_REGIDX_BITS(STAT_REG_OFFSET, 4U, 17U),     /*!< status change interrupt flag of sleep working mode entering */ 
    CAN_INT_FLAG_WUIF  = CAN_REGIDX_BITS(STAT_REG_OFFSET, 3U, 16),      /*!< status change interrupt flag of wakeup from sleep working mode */ 
    CAN_INT_FLAG_ERRIF = CAN_REGIDX_BITS(STAT_REG_OFFSET, 2U, 15),      /*!< error interrupt flag */ 
    /* interrupt flags in TSTAT register */
    CAN_INT_FLAG_MTF2  = CAN_REGIDX_BITS(TSTAT_REG_OFFSET, 16U, 0U),    /*!< mailbox 2 transmit finished interrupt flag */
    CAN_INT_FLAG_MTF1  = CAN_REGIDX_BITS(TSTAT_REG_OFFSET, 8U, 0U),     /*!< mailbox 1 transmit finished interrupt flag */
    CAN_INT_FLAG_MTF0  = CAN_REGIDX_BITS(TSTAT_REG_OFFSET, 0U, 0U),     /*!< mailbox 0 transmit finished interrupt flag */
    /* interrupt flags in RFIFO0 register */
    CAN_INT_FLAG_RFO0  = CAN_REGIDX_BITS(RFIFO0_REG_OFFSET, 4U, 3U),    /*!< receive FIFO0 overfull interrupt flag */
    CAN_INT_FLAG_RFF0  = CAN_REGIDX_BITS(RFIFO0_REG_OFFSET, 3U, 2U),    /*!< receive FIFO0 full interrupt flag */
    CAN_INT_FLAG_RFL0  = CAN_REGIDX_BITS(RFIFO0_REG_OFFSET, 2U, 1U),    /*!< receive FIFO0 not empty interrupt flag */
    /* interrupt flags in RFIFO0 register */
    CAN_INT_FLAG_RFO1  = CAN_REGIDX_BITS(RFIFO1_REG_OFFSET, 4U, 6U),    /*!< receive FIFO1 overfull interrupt flag */
    CAN_INT_FLAG_RFF1  = CAN_REGIDX_BITS(RFIFO1_REG_OFFSET, 3U, 5U),    /*!< receive FIFO1 full interrupt flag */
    CAN_INT_FLAG_RFL1  = CAN_REGIDX_BITS(RFIFO1_REG_OFFSET, 2U, 4U),    /*!< receive FIFO1 not empty interrupt flag */
    /* interrupt flags in ERR register */
    CAN_INT_FLAG_ERRN  = CAN_REGIDX_BITS(ERR_REG_OFFSET, 3U, 11U),      /*!< error number interrupt flag */ 
    CAN_INT_FLAG_BOERR = CAN_REGIDX_BITS(ERR_REG_OFFSET, 2U, 10U),      /*!< bus-off error interrupt flag */ 
    CAN_INT_FLAG_PERR  = CAN_REGIDX_BITS(ERR_REG_OFFSET, 1U, 9U),       /*!< passive error interrupt flag */ 
    CAN_INT_FLAG_WERR  = CAN_REGIDX_BITS(ERR_REG_OFFSET, 0U, 8U),       /*!< warning error interrupt flag */ 
}can_interrupt_flag_enum;

/* CAN initiliaze parameters struct */
typedef struct
{
    uint8_t working_mode;                                               /*!< CAN working mode */ 
    uint8_t resync_jump_width;                                          /*!< CAN resynchronization jump width */
    uint8_t time_segment_1;                                             /*!< time segment 1 */
    uint8_t time_segment_2;                                             /*!< time segment 2 */
    ControlStatus time_triggered;                                       /*!< time triggered communication mode */
    ControlStatus auto_bus_off_recovery;                                /*!< automatic bus-off recovery */
    ControlStatus auto_wake_up;                                         /*!< automatic wake-up mode */
    ControlStatus auto_retrans;                                         /*!< automatic retransmission mode */
    ControlStatus rec_fifo_overwrite;                                   /*!< receive FIFO overwrite mode */
    ControlStatus trans_fifo_order;                                     /*!< transmit FIFO order */
    uint16_t prescaler;                                                 /*!< baudrate prescaler */
}can_parameter_struct;

/* CAN transmit message struct */
typedef struct
{
    uint32_t tx_sfid;                                                   /*!< standard format frame identifier */
    uint32_t tx_efid;                                                   /*!< extended format frame identifier */
    uint8_t tx_ff;                                                      /*!< format of frame, standard or extended format */
    uint8_t tx_ft;                                                      /*!< type of frame, data or remote */
    uint8_t tx_dlen;                                                    /*!< data length */
    uint8_t tx_data[8];                                                 /*!< transmit data */
}can_trasnmit_message_struct;

/* CAN receive message struct */
typedef struct
{
    uint32_t rx_sfid;                                                   /*!< standard format frame identifier */
    uint32_t rx_efid;                                                   /*!< extended format frame identifier */
    uint8_t rx_ff;                                                      /*!< format of frame, standard or extended format */
    uint8_t rx_ft;                                                      /*!< type of frame, data or remote */
    uint8_t rx_dlen;                                                    /*!< data length */
    uint8_t rx_data[8];                                                 /*!< receive data */
    uint8_t rx_fi;                                                      /*!< filtering index */
} can_receive_message_struct;

/* CAN filter parameters struct */
typedef struct
{
    uint16_t filter_list_high;                                          /*!< filter list number high bits*/
    uint16_t filter_list_low;                                           /*!< filter list number low bits */
    uint16_t filter_mask_high;                                          /*!< filter mask number high bits */
    uint16_t filter_mask_low;                                           /*!< filter mask number low bits */
    uint16_t filter_fifo_number;                                        /*!< receive FIFO associated with the filter */
    uint16_t filter_number;                                             /*!< filter number */
    uint16_t filter_mode;                                               /*!< filter mode, list or mask */
    uint16_t filter_bits;                                               /*!< filter scale */
    ControlStatus filter_enable;                                        /*!< filter work or not */
}can_filter_parameter_struct;

/* CAN errors */
typedef enum
{
    CAN_ERROR_NONE = 0,                                                 /*!< no error */
    CAN_ERROR_FILL,                                                     /*!< fill error */
    CAN_ERROR_FORMATE,                                                  /*!< format error */
    CAN_ERROR_ACK,                                                      /*!< ACK error */
    CAN_ERROR_BITRECESSIVE,                                             /*!< bit recessive error */
    CAN_ERROR_BITDOMINANTER,                                            /*!< bit dominant error */
    CAN_ERROR_CRC,                                                      /*!< CRC error */
    CAN_ERROR_SOFTWARECFG,                                              /*!< software configure */
}can_error_enum;

/* transmit states */
typedef enum
{
    CAN_TRANSMIT_FAILED = 0U,                                            /*!< CAN transmitted failure */
    CAN_TRANSMIT_OK = 1U,                                                /*!< CAN transmitted success */
    CAN_TRANSMIT_PENDING = 2U,                                           /*!< CAN transmitted pending */
    CAN_TRANSMIT_NOMAILBOX = 4U,                                         /*!< no empty mailbox to be used for CAN */
}can_transmit_state_enum;

typedef enum
{
    CAN_INIT_STRUCT = 0,                                                /* CAN initiliaze parameters struct */
    CAN_FILTER_STRUCT,                                                  /* CAN filter parameters struct */
    CAN_TX_MESSAGE_STRUCT,                                              /* CAN transmit message struct */
    CAN_RX_MESSAGE_STRUCT,                                              /* CAN receive message struct */
}can_struct_type_enum;

/* CAN baudrate prescaler*/
#define BT_BAUDPSC(regval)                 (BITS(0,9) & ((uint32_t)(regval) << 0))

/* CAN bit segment 1*/
#define BT_BS1(regval)                     (BITS(16,19) & ((uint32_t)(regval) << 16))

/* CAN bit segment 2*/
#define BT_BS2(regval)                     (BITS(20,22) & ((uint32_t)(regval) << 20))

/* CAN resynchronization jump width*/
#define BT_SJW(regval)                     (BITS(24,25) & ((uint32_t)(regval) << 24))

/* CAN communication mode*/
#define BT_MODE(regval)                    (BITS(30,31) & ((uint32_t)(regval) << 30))

/* CAN FDATA high 16 bits */
#define FDATA_MASK_HIGH(regval)            (BITS(16,31) & ((uint32_t)(regval) << 16))

/* CAN FDATA low 16 bits */
#define FDATA_MASK_LOW(regval)             (BITS(0,15) & ((uint32_t)(regval) << 0))

/* CAN1 filter start bank_number*/
#define FCTL_HBC1F(regval)                 (BITS(8,13) & ((uint32_t)(regval) << 8))

/* CAN transmit mailbox extended identifier*/
#define TMI_EFID(regval)                   (BITS(3,31) & ((uint32_t)(regval) << 3))

/* CAN transmit mailbox standard identifier*/
#define TMI_SFID(regval)                   (BITS(21,31) & ((uint32_t)(regval) << 21))

/* transmit data byte 0 */
#define TMDATA0_DB0(regval)                (BITS(0,7) & ((uint32_t)(regval) << 0))

/* transmit data byte 1 */
#define TMDATA0_DB1(regval)                (BITS(8,15) & ((uint32_t)(regval) << 8))

/* transmit data byte 2 */
#define TMDATA0_DB2(regval)                (BITS(16,23) & ((uint32_t)(regval) << 16))

/* transmit data byte 3 */
#define TMDATA0_DB3(regval)                (BITS(24,31) & ((uint32_t)(regval) << 24))

/* transmit data byte 4 */
#define TMDATA1_DB4(regval)                (BITS(0,7) & ((uint32_t)(regval) << 0))

/* transmit data byte 5 */
#define TMDATA1_DB5(regval)                (BITS(8,15) & ((uint32_t)(regval) << 8))

/* transmit data byte 6 */
#define TMDATA1_DB6(regval)                (BITS(16,23) & ((uint32_t)(regval) << 16))

/* transmit data byte 7 */
#define TMDATA1_DB7(regval)                (BITS(24,31) & ((uint32_t)(regval) << 24))

/* receive mailbox extended identifier*/
#define GET_RFIFOMI_EFID(regval)           GET_BITS((uint32_t)(regval), 3U, 31U)

/* receive mailbox standrad identifier*/
#define GET_RFIFOMI_SFID(regval)           GET_BITS((uint32_t)(regval), 21U, 31U)

/* receive data length */
#define GET_RFIFOMP_DLENC(regval)          GET_BITS((uint32_t)(regval), 0U, 3U)

/* the index of the filter by which the frame is passed */
#define GET_RFIFOMP_FI(regval)             GET_BITS((uint32_t)(regval), 8U, 15U)

/* receive data byte 0 */
#define GET_RFIFOMDATA0_DB0(regval)        GET_BITS((uint32_t)(regval), 0U, 7U)

/* receive data byte 1 */
#define GET_RFIFOMDATA0_DB1(regval)        GET_BITS((uint32_t)(regval), 8U, 15U)

/* receive data byte 2 */
#define GET_RFIFOMDATA0_DB2(regval)        GET_BITS((uint32_t)(regval), 16U, 23U)

/* receive data byte 3 */
#define GET_RFIFOMDATA0_DB3(regval)        GET_BITS((uint32_t)(regval), 24U, 31U)

/* receive data byte 4 */
#define GET_RFIFOMDATA1_DB4(regval)        GET_BITS((uint32_t)(regval), 0U, 7U)

/* receive data byte 5 */
#define GET_RFIFOMDATA1_DB5(regval)        GET_BITS((uint32_t)(regval), 8U, 15U)

/* receive data byte 6 */
#define GET_RFIFOMDATA1_DB6(regval)        GET_BITS((uint32_t)(regval), 16U, 23U)

/* receive data byte 7 */
#define GET_RFIFOMDATA1_DB7(regval)        GET_BITS((uint32_t)(regval), 24U, 31U)

/* error number */        
#define GET_ERR_ERRN(regval)               GET_BITS((uint32_t)(regval), 4U, 6U)

/* transmit error count */        
#define GET_ERR_TECNT(regval)              GET_BITS((uint32_t)(regval), 16U, 23U)

/* receive  error count */        
#define GET_ERR_RECNT(regval)              GET_BITS((uint32_t)(regval), 24U, 31U)

/* CAN errors */
#define ERR_ERRN(regval)                   (BITS(4,6) & ((uint32_t)(regval) << 4))
#define CAN_ERRN_0                         ERR_ERRN(0U)                  /* no error */
#define CAN_ERRN_1                         ERR_ERRN(1U)                  /*!< fill error */
#define CAN_ERRN_2                         ERR_ERRN(2U)                  /*!< format error */
#define CAN_ERRN_3                         ERR_ERRN(3U)                  /*!< ACK error */
#define CAN_ERRN_4                         ERR_ERRN(4U)                  /*!< bit recessive error */
#define CAN_ERRN_5                         ERR_ERRN(5U)                  /*!< bit dominant error */
#define CAN_ERRN_6                         ERR_ERRN(6U)                  /*!< CRC error */
#define CAN_ERRN_7                         ERR_ERRN(7U)                  /*!< software error */

#define CAN_STATE_PENDING                  ((uint32_t)0x00000000U)      /*!< CAN pending */

/* CAN communication mode */
#define CAN_NORMAL_MODE                    ((uint8_t)0x00U)             /*!< normal communication mode */
#define CAN_LOOPBACK_MODE                  ((uint8_t)0x01U)             /*!< loopback communication mode */
#define CAN_SILENT_MODE                    ((uint8_t)0x02U)             /*!< silent communication mode */
#define CAN_SILENT_LOOPBACK_MODE           ((uint8_t)0x03U)             /*!< loopback and silent communication mode */

/* CAN resynchronisation jump width */
#define CAN_BT_SJW_1TQ                     ((uint8_t)0x00U)             /*!< 1 time quanta */
#define CAN_BT_SJW_2TQ                     ((uint8_t)0x01U)             /*!< 2 time quanta */
#define CAN_BT_SJW_3TQ                     ((uint8_t)0x02U)             /*!< 3 time quanta */
#define CAN_BT_SJW_4TQ                     ((uint8_t)0x03U)             /*!< 4 time quanta */

/* CAN time segment 1 */
#define CAN_BT_BS1_1TQ                     ((uint8_t)0x00U)             /*!< 1 time quanta */
#define CAN_BT_BS1_2TQ                     ((uint8_t)0x01U)             /*!< 2 time quanta */
#define CAN_BT_BS1_3TQ                     ((uint8_t)0x02U)             /*!< 3 time quanta */
#define CAN_BT_BS1_4TQ                     ((uint8_t)0x03U)             /*!< 4 time quanta */
#define CAN_BT_BS1_5TQ                     ((uint8_t)0x04U)             /*!< 5 time quanta */
#define CAN_BT_BS1_6TQ                     ((uint8_t)0x05U)             /*!< 6 time quanta */
#define CAN_BT_BS1_7TQ                     ((uint8_t)0x06U)             /*!< 7 time quanta */
#define CAN_BT_BS1_8TQ                     ((uint8_t)0x07U)             /*!< 8 time quanta */
#define CAN_BT_BS1_9TQ                     ((uint8_t)0x08U)             /*!< 9 time quanta */
#define CAN_BT_BS1_10TQ                    ((uint8_t)0x09U)             /*!< 10 time quanta */
#define CAN_BT_BS1_11TQ                    ((uint8_t)0x0AU)             /*!< 11 time quanta */
#define CAN_BT_BS1_12TQ                    ((uint8_t)0x0BU)             /*!< 12 time quanta */
#define CAN_BT_BS1_13TQ                    ((uint8_t)0x0CU)             /*!< 13 time quanta */
#define CAN_BT_BS1_14TQ                    ((uint8_t)0x0DU)             /*!< 14 time quanta */
#define CAN_BT_BS1_15TQ                    ((uint8_t)0x0EU)             /*!< 15 time quanta */
#define CAN_BT_BS1_16TQ                    ((uint8_t)0x0FU)             /*!< 16 time quanta */

/* CAN time segment 2 */
#define CAN_BT_BS2_1TQ                     ((uint8_t)0x00U)             /*!< 1 time quanta */
#define CAN_BT_BS2_2TQ                     ((uint8_t)0x01U)             /*!< 2 time quanta */
#define CAN_BT_BS2_3TQ                     ((uint8_t)0x02U)             /*!< 3 time quanta */
#define CAN_BT_BS2_4TQ                     ((uint8_t)0x03U)             /*!< 4 time quanta */
#define CAN_BT_BS2_5TQ                     ((uint8_t)0x04U)             /*!< 5 time quanta */
#define CAN_BT_BS2_6TQ                     ((uint8_t)0x05U)             /*!< 6 time quanta */
#define CAN_BT_BS2_7TQ                     ((uint8_t)0x06U)             /*!< 7 time quanta */
#define CAN_BT_BS2_8TQ                     ((uint8_t)0x07U)             /*!< 8 time quanta */

/* CAN mailbox number */
#define CAN_MAILBOX0                       ((uint8_t)0x00U)             /*!< mailbox0 */
#define CAN_MAILBOX1                       ((uint8_t)0x01U)             /*!< mailbox1 */
#define CAN_MAILBOX2                       ((uint8_t)0x02U)             /*!< mailbox2 */
#define CAN_NOMAILBOX                      ((uint8_t)0x03U)             /*!< no mailbox empty */

/* CAN frame format */
#define CAN_FF_STANDARD                    ((uint32_t)0x00000000U)      /*!< standard frame */
#define CAN_FF_EXTENDED                    ((uint32_t)0x00000004U)      /*!< extended frame */

/* CAN receive fifo */
#define CAN_FIFO0                          ((uint8_t)0x00U)             /*!< receive FIFO0 */
#define CAN_FIFO1                          ((uint8_t)0x01U)             /*!< receive FIFO1 */

/* frame number of receive fifo */
#define CAN_RFIF_RFL_MASK                  ((uint32_t)0x00000003U)      /*!< mask for frame number in receive FIFOx */

#define CAN_SFID_MASK                      ((uint32_t)0x000007FFU)      /*!< mask of standard identifier */
#define CAN_EFID_MASK                      ((uint32_t)0x1FFFFFFFU)      /*!< mask of extended identifier */

/* CAN working mode */
#define CAN_MODE_INITIALIZE                ((uint8_t)0x01U)             /*!< CAN initialize mode */
#define CAN_MODE_NORMAL                    ((uint8_t)0x02U)             /*!< CAN normal mode */
#define CAN_MODE_SLEEP                     ((uint8_t)0x04U)             /*!< CAN sleep mode */

/* filter bits */
#define CAN_FILTERBITS_16BIT               ((uint8_t)0x00U)             /*!< CAN filter 16 bits */
#define CAN_FILTERBITS_32BIT               ((uint8_t)0x01U)             /*!< CAN filter 32 bits */

/* filter mode */
#define CAN_FILTERMODE_MASK                ((uint8_t)0x00U)             /*!< mask mode */
#define CAN_FILTERMODE_LIST                ((uint8_t)0x01U)             /*!< list mode */

/* filter 16 bits mask */
#define CAN_FILTER_MASK_16BITS             ((uint32_t)0x0000FFFFU)      /*!< can filter 16 bits mask */

/* frame type */
#define CAN_FT_DATA                        ((uint32_t)0x00000000U)      /*!< data frame */
#define CAN_FT_REMOTE                      ((uint32_t)0x00000002U)      /*!< remote frame */

/* CAN timeout */
#define CAN_TIMEOUT                        ((uint32_t)0x0000FFFFU)      /*!< timeout value */

/* interrupt enable bits */
#define CAN_INT_TME                        CAN_INTEN_TMEIE              /*!< transmit mailbox empty interrupt enable */
#define CAN_INT_RFNE0                      CAN_INTEN_RFNEIE0            /*!< receive FIFO0 not empty interrupt enable */
#define CAN_INT_RFF0                       CAN_INTEN_RFFIE0             /*!< receive FIFO0 full interrupt enable */
#define CAN_INT_RFO0                       CAN_INTEN_RFOIE0             /*!< receive FIFO0 overfull interrupt enable */
#define CAN_INT_RFNE1                      CAN_INTEN_RFNEIE1            /*!< receive FIFO1 not empty interrupt enable */
#define CAN_INT_RFF1                       CAN_INTEN_RFFIE1             /*!< receive FIFO1 full interrupt enable */
#define CAN_INT_RFO1                       CAN_INTEN_RFOIE1             /*!< receive FIFO1 overfull interrupt enable */
#define CAN_INT_WERR                       CAN_INTEN_WERRIE             /*!< warning error interrupt enable */
#define CAN_INT_PERR                       CAN_INTEN_PERRIE             /*!< passive error interrupt enable */
#define CAN_INT_BO                         CAN_INTEN_BOIE               /*!< bus-off interrupt enable */
#define CAN_INT_ERRN                       CAN_INTEN_ERRNIE             /*!< error number interrupt enable */
#define CAN_INT_ERR                        CAN_INTEN_ERRIE              /*!< error interrupt enable */
#define CAN_INT_WAKEUP                     CAN_INTEN_WIE                /*!< wakeup interrupt enable */
#define CAN_INT_SLPW                       CAN_INTEN_SLPWIE             /*!< sleep working interrupt enable */


////////////////////////////////////////////////////////////////////////
/* CRC definitions */
#define CRC                            CRC_BASE

/* registers definitions */
#define CRC_DATA                       REG32(CRC + 0x00U)              /*!< CRC data register */
#define CRC_FDATA                      REG32(CRC + 0x04U)              /*!< CRC free data register */
#define CRC_CTL                        REG32(CRC + 0x08U)              /*!< CRC control register */

/* bits definitions */
/* CRC_DATA */
#define CRC_DATA_DATA                  BITS(0, 31)                     /*!< CRC calculation result bits */

/* CRC_FDATA */
#define CRC_FDATA_FDATA                BITS(0, 7)                      /*!< CRC free data bits */

/* CRC_CTL */
#define CRC_CTL_RST                    BIT(0)                          /*!< CRC reset CRC_DATA register bit */


////////////////////////////////////////////////////////////////////////
/* DACx(x=0,1) definitions */
#define DAC                     DAC_BASE
#define DAC0                    (0U)
#define DAC1                    (1U)

/* registers definitions */
#define DAC_CTL                 REG32(DAC + 0x00U)   /*!< DAC control register */
#define DAC_SWT                 REG32(DAC + 0x04U)   /*!< DAC software trigger register */
#define DAC0_R12DH              REG32(DAC + 0x08U)   /*!< DAC0 12-bit right-aligned data holding register */
#define DAC0_L12DH              REG32(DAC + 0x0CU)   /*!< DAC0 12-bit left-aligned data holding register */
#define DAC0_R8DH               REG32(DAC + 0x10U)   /*!< DAC0 8-bit right-aligned data holding register */
#define DAC1_R12DH              REG32(DAC + 0x14U)   /*!< DAC1 12-bit right-aligned data holding register */
#define DAC1_L12DH              REG32(DAC + 0x18U)   /*!< DAC1 12-bit left-aligned data holding register */
#define DAC1_R8DH               REG32(DAC + 0x1CU)   /*!< DAC1 8-bit right-aligned data holding register */
#define DACC_R12DH              REG32(DAC + 0x20U)   /*!< DAC concurrent mode 12-bit right-aligned data holding register */
#define DACC_L12DH              REG32(DAC + 0x24U)   /*!< DAC concurrent mode 12-bit left-aligned data holding register */
#define DACC_R8DH               REG32(DAC + 0x28U)   /*!< DAC concurrent mode 8-bit right-aligned data holding register */
#define DAC0_DO                 REG32(DAC + 0x2CU)   /*!< DAC0 data output register */
#define DAC1_DO                 REG32(DAC + 0x30U)   /*!< DAC1 data output register */

/* bits definitions */
/* DAC_CTL */
#define DAC_CTL_DEN0            BIT(0)               /*!< DAC0 enable/disable bit */
#define DAC_CTL_DBOFF0          BIT(1)               /*!< DAC0 output buffer turn on/turn off bit */
#define DAC_CTL_DTEN0           BIT(2)               /*!< DAC0 trigger enable/disable bit */
#define DAC_CTL_DTSEL0          BITS(3,5)            /*!< DAC0 trigger source selection enable/disable bits */
#define DAC_CTL_DWM0            BITS(6,7)            /*!< DAC0 noise wave mode */
#define DAC_CTL_DWBW0           BITS(8,11)           /*!< DAC0 noise wave bit width */
#define DAC_CTL_DDMAEN0         BIT(12)              /*!< DAC0 DMA enable/disable bit */
#define DAC_CTL_DEN1            BIT(16)              /*!< DAC1 enable/disable bit */ 
#define DAC_CTL_DBOFF1          BIT(17)              /*!< DAC1 output buffer turn on/turn off bit */
#define DAC_CTL_DTEN1           BIT(18)              /*!< DAC1 trigger enable/disable bit */
#define DAC_CTL_DTSEL1          BITS(19,21)          /*!< DAC1 trigger source selection enable/disable bits */
#define DAC_CTL_DWM1            BITS(22,23)          /*!< DAC1 noise wave mode */
#define DAC_CTL_DWBW1           BITS(24,27)          /*!< DAC1 noise wave bit width */
#define DAC_CTL_DDMAEN1         BIT(28)              /*!< DAC1 DMA enable/disable bit */

/* DAC_SWT */
#define DAC_SWT_SWTR0           BIT(0)               /*!< DAC0 software trigger bit, cleared by hardware */
#define DAC_SWT_SWTR1           BIT(1)               /*!< DAC1 software trigger bit, cleared by hardware */

/* DAC0_R12DH */
#define DAC0_R12DH_DAC0_DH      BITS(0,11)           /*!< DAC0 12-bit right-aligned data bits */

/* DAC0_L12DH */
#define DAC0_L12DH_DAC0_DH      BITS(4,15)           /*!< DAC0 12-bit left-aligned data bits */

/* DAC0_R8DH */
#define DAC0_R8DH_DAC0_DH       BITS(0,7)            /*!< DAC0 8-bit right-aligned data bits */

/* DAC1_R12DH */
#define DAC1_R12DH_DAC1_DH      BITS(0,11)           /*!< DAC1 12-bit right-aligned data bits */

/* DAC1_L12DH */
#define DAC1_L12DH_DAC1_DH      BITS(4,15)           /*!< DAC1 12-bit left-aligned data bits */

/* DAC1_R8DH */
#define DAC1_R8DH_DAC1_DH       BITS(0,7)            /*!< DAC1 8-bit right-aligned data bits */

/* DACC_R12DH */
#define DACC_R12DH_DAC0_DH      BITS(0,11)           /*!< DAC concurrent mode DAC0 12-bit right-aligned data bits */
#define DACC_R12DH_DAC1_DH      BITS(16,27)          /*!< DAC concurrent mode DAC1 12-bit right-aligned data bits */

/* DACC_L12DH */
#define DACC_L12DH_DAC0_DH      BITS(4,15)           /*!< DAC concurrent mode DAC0 12-bit left-aligned data bits */
#define DACC_L12DH_DAC1_DH      BITS(20,31)          /*!< DAC concurrent mode DAC1 12-bit left-aligned data bits */

/* DACC_R8DH */
#define DACC_R8DH_DAC0_DH       BITS(0,7)            /*!< DAC concurrent mode DAC0 8-bit right-aligned data bits */
#define DACC_R8DH_DAC1_DH       BITS(8,15)           /*!< DAC concurrent mode DAC1 8-bit right-aligned data bits */

/* DAC0_DO */
#define DAC0_DO_DAC0_DO         BITS(0,11)           /*!< DAC0 12-bit output data bits */

/* DAC1_DO */
#define DAC1_DO_DAC1_DO         BITS(0,11)           /*!< DAC1 12-bit output data bits */

/* constants definitions */
/* DAC trigger source */
#define CTL_DTSEL(regval)       (BITS(3,5) & ((uint32_t)(regval) << 3))
#define DAC_TRIGGER_T5_TRGO     CTL_DTSEL(0)         /*!< TIMER5 TRGO */
#define DAC_TRIGGER_T2_TRGO     CTL_DTSEL(1)         /*!< TIMER2 TRGO */
#define DAC_TRIGGER_T6_TRGO     CTL_DTSEL(2)         /*!< TIMER6 TRGO */
#define DAC_TRIGGER_T4_TRGO     CTL_DTSEL(3)         /*!< TIMER4 TRGO */
#define DAC_TRIGGER_T1_TRGO     CTL_DTSEL(4)         /*!< TIMER1 TRGO */
#define DAC_TRIGGER_T3_TRGO     CTL_DTSEL(5)         /*!< TIMER3 TRGO */
#define DAC_TRIGGER_EXTI_9      CTL_DTSEL(6)         /*!< EXTI interrupt line9 event */
#define DAC_TRIGGER_SOFTWARE    CTL_DTSEL(7)         /*!< software trigger */

/* DAC noise wave mode */
#define CTL_DWM(regval)         (BITS(6,7) & ((uint32_t)(regval) << 6))
#define DAC_WAVE_DISABLE        CTL_DWM(0)           /*!< wave disable */
#define DAC_WAVE_MODE_LFSR      CTL_DWM(1)           /*!< LFSR noise mode */
#define DAC_WAVE_MODE_TRIANGLE  CTL_DWM(2)           /*!< triangle noise mode */

/* DAC noise wave bit width */
#define DWBW(regval)            (BITS(8,11) & ((uint32_t)(regval) << 8))
#define DAC_WAVE_BIT_WIDTH_1    DWBW(0)              /*!< bit width of the wave signal is 1 */
#define DAC_WAVE_BIT_WIDTH_2    DWBW(1)              /*!< bit width of the wave signal is 2 */
#define DAC_WAVE_BIT_WIDTH_3    DWBW(2)              /*!< bit width of the wave signal is 3 */
#define DAC_WAVE_BIT_WIDTH_4    DWBW(3)              /*!< bit width of the wave signal is 4 */
#define DAC_WAVE_BIT_WIDTH_5    DWBW(4)              /*!< bit width of the wave signal is 5 */
#define DAC_WAVE_BIT_WIDTH_6    DWBW(5)              /*!< bit width of the wave signal is 6 */
#define DAC_WAVE_BIT_WIDTH_7    DWBW(6)              /*!< bit width of the wave signal is 7 */
#define DAC_WAVE_BIT_WIDTH_8    DWBW(7)              /*!< bit width of the wave signal is 8 */
#define DAC_WAVE_BIT_WIDTH_9    DWBW(8)              /*!< bit width of the wave signal is 9 */
#define DAC_WAVE_BIT_WIDTH_10   DWBW(9)              /*!< bit width of the wave signal is 10 */
#define DAC_WAVE_BIT_WIDTH_11   DWBW(10)             /*!< bit width of the wave signal is 11 */
#define DAC_WAVE_BIT_WIDTH_12   DWBW(11)             /*!< bit width of the wave signal is 12 */

/* unmask LFSR bits in DAC LFSR noise mode */
#define DAC_LFSR_BIT0           DAC_WAVE_BIT_WIDTH_1  /*!< unmask the LFSR bit0 */
#define DAC_LFSR_BITS1_0        DAC_WAVE_BIT_WIDTH_2  /*!< unmask the LFSR bits[1:0] */
#define DAC_LFSR_BITS2_0        DAC_WAVE_BIT_WIDTH_3  /*!< unmask the LFSR bits[2:0] */
#define DAC_LFSR_BITS3_0        DAC_WAVE_BIT_WIDTH_4  /*!< unmask the LFSR bits[3:0] */
#define DAC_LFSR_BITS4_0        DAC_WAVE_BIT_WIDTH_5  /*!< unmask the LFSR bits[4:0] */
#define DAC_LFSR_BITS5_0        DAC_WAVE_BIT_WIDTH_6  /*!< unmask the LFSR bits[5:0] */
#define DAC_LFSR_BITS6_0        DAC_WAVE_BIT_WIDTH_7  /*!< unmask the LFSR bits[6:0] */
#define DAC_LFSR_BITS7_0        DAC_WAVE_BIT_WIDTH_8  /*!< unmask the LFSR bits[7:0] */
#define DAC_LFSR_BITS8_0        DAC_WAVE_BIT_WIDTH_9  /*!< unmask the LFSR bits[8:0] */
#define DAC_LFSR_BITS9_0        DAC_WAVE_BIT_WIDTH_10 /*!< unmask the LFSR bits[9:0] */
#define DAC_LFSR_BITS10_0       DAC_WAVE_BIT_WIDTH_11 /*!< unmask the LFSR bits[10:0] */
#define DAC_LFSR_BITS11_0       DAC_WAVE_BIT_WIDTH_12 /*!< unmask the LFSR bits[11:0] */

/* DAC data alignment */
#define DATA_ALIGN(regval)      (BITS(0,1) & ((uint32_t)(regval) << 0))
#define DAC_ALIGN_12B_R         DATA_ALIGN(0)        /*!< data right 12b alignment */
#define DAC_ALIGN_12B_L         DATA_ALIGN(1)        /*!< data left  12b alignment */
#define DAC_ALIGN_8B_R          DATA_ALIGN(2)        /*!< data right  8b alignment */
/* triangle amplitude in DAC triangle noise mode */
#define DAC_TRIANGLE_AMPLITUDE_1    DAC_WAVE_BIT_WIDTH_1  /*!< triangle amplitude is 1 */
#define DAC_TRIANGLE_AMPLITUDE_3    DAC_WAVE_BIT_WIDTH_2  /*!< triangle amplitude is 3 */
#define DAC_TRIANGLE_AMPLITUDE_7    DAC_WAVE_BIT_WIDTH_3  /*!< triangle amplitude is 7 */
#define DAC_TRIANGLE_AMPLITUDE_15   DAC_WAVE_BIT_WIDTH_4  /*!< triangle amplitude is 15 */
#define DAC_TRIANGLE_AMPLITUDE_31   DAC_WAVE_BIT_WIDTH_5  /*!< triangle amplitude is 31 */
#define DAC_TRIANGLE_AMPLITUDE_63   DAC_WAVE_BIT_WIDTH_6  /*!< triangle amplitude is 63 */
#define DAC_TRIANGLE_AMPLITUDE_127  DAC_WAVE_BIT_WIDTH_7  /*!< triangle amplitude is 127 */
#define DAC_TRIANGLE_AMPLITUDE_255  DAC_WAVE_BIT_WIDTH_8  /*!< triangle amplitude is 255 */
#define DAC_TRIANGLE_AMPLITUDE_511  DAC_WAVE_BIT_WIDTH_9  /*!< triangle amplitude is 511 */
#define DAC_TRIANGLE_AMPLITUDE_1023 DAC_WAVE_BIT_WIDTH_10 /*!< triangle amplitude is 1023 */
#define DAC_TRIANGLE_AMPLITUDE_2047 DAC_WAVE_BIT_WIDTH_11 /*!< triangle amplitude is 2047 */
#define DAC_TRIANGLE_AMPLITUDE_4095 DAC_WAVE_BIT_WIDTH_12 /*!< triangle amplitude is 4095 */


////////////////////////////////////////////////////////////////////////
/* DBG definitions */
#define DBG                      DBG_BASE

/* registers definitions */
#define DBG_ID                   REG32(DBG + 0x00U)         /*!< DBG_ID code register */
#define DBG_CTL                  REG32(DBG + 0x04U)         /*!< DBG control register */

/* bits definitions */
/* DBG_ID */
#define DBG_ID_ID_CODE           BITS(0,31)                 /*!< DBG ID code values */

/* DBG_CTL */
#define DBG_CTL_SLP_HOLD         BIT(0)                     /*!< keep debugger connection during sleep mode */
#define DBG_CTL_DSLP_HOLD        BIT(1)                     /*!< keep debugger connection during deepsleep mode */
#define DBG_CTL_STB_HOLD         BIT(2)                     /*!< keep debugger connection during standby mode */
#define DBG_CTL_FWDGT_HOLD       BIT(8)                     /*!< debug FWDGT kept when core is halted */
#define DBG_CTL_WWDGT_HOLD       BIT(9)                     /*!< debug WWDGT kept when core is halted */
#define DBG_CTL_TIMER0_HOLD      BIT(10)                    /*!< hold TIMER0 counter when core is halted */
#define DBG_CTL_TIMER1_HOLD      BIT(11)                    /*!< hold TIMER1 counter when core is halted */
#define DBG_CTL_TIMER2_HOLD      BIT(12)                    /*!< hold TIMER2 counter when core is halted */
#define DBG_CTL_TIMER3_HOLD      BIT(13)                    /*!< hold TIMER3 counter when core is halted */
#define DBG_CTL_CAN0_HOLD        BIT(14)                    /*!< debug CAN0 kept when core is halted */
#define DBG_CTL_I2C0_HOLD        BIT(15)                    /*!< hold I2C0 smbus when core is halted */
#define DBG_CTL_I2C1_HOLD        BIT(16)                    /*!< hold I2C1 smbus when core is halted */
#define DBG_CTL_TIMER4_HOLD      BIT(18)                    /*!< hold TIMER4 counter when core is halted */
#define DBG_CTL_TIMER5_HOLD      BIT(19)                    /*!< hold TIMER5 counter when core is halted */
#define DBG_CTL_TIMER6_HOLD      BIT(20)                    /*!< hold TIMER6 counter when core is halted */
#define DBG_CTL_CAN1_HOLD        BIT(21)                    /*!< debug CAN1 kept when core is halted */

/* constants definitions */
/* debug hold when core is halted */
typedef enum
{
    DBG_FWDGT_HOLD             = BIT(8),                    /*!< debug FWDGT kept when core is halted */
    DBG_WWDGT_HOLD             = BIT(9),                    /*!< debug WWDGT kept when core is halted */
    DBG_TIMER0_HOLD            = BIT(10),                   /*!< hold TIMER0 counter when core is halted */
    DBG_TIMER1_HOLD            = BIT(11),                   /*!< hold TIMER1 counter when core is halted */
    DBG_TIMER2_HOLD            = BIT(12),                   /*!< hold TIMER2 counter when core is halted */
    DBG_TIMER3_HOLD            = BIT(13),                   /*!< hold TIMER3 counter when core is halted */
    DBG_CAN0_HOLD              = BIT(14),                   /*!< debug CAN0 kept when core is halted */
    DBG_I2C0_HOLD              = BIT(15),                   /*!< hold I2C0 smbus when core is halted */
    DBG_I2C1_HOLD              = BIT(16),                   /*!< hold I2C1 smbus when core is halted */
    DBG_TIMER4_HOLD            = BIT(17),                   /*!< hold TIMER4 counter when core is halted */
    DBG_TIMER5_HOLD            = BIT(18),                   /*!< hold TIMER5 counter when core is halted */
    DBG_TIMER6_HOLD            = BIT(19),                   /*!< hold TIMER6 counter when core is halted */
    DBG_CAN1_HOLD              = BIT(21),                   /*!< debug CAN1 kept when core is halted */
}dbg_periph_enum;

/* DBG low power mode configurations */
#define DBG_LOW_POWER_SLEEP      DBG_CTL_SLP_HOLD           /*!< keep debugger connection during sleep mode */
#define DBG_LOW_POWER_DEEPSLEEP  DBG_CTL_DSLP_HOLD          /*!< keep debugger connection during deepsleep mode */
#define DBG_LOW_POWER_STANDBY    DBG_CTL_STB_HOLD           /*!< keep debugger connection during standby mode */


////////////////////////////////////////////////////////////////////////
/* DMA definitions */
#define DMA0                            (DMA_BASE)               /*!< DMA0 base address */
#define DMA1                            (DMA_BASE + 0x0400U)     /*!< DMA1 base address */

/* registers definitions */
#define DMA_INTF(dmax)                  REG32((dmax) + 0x00U)    /*!< DMA interrupt flag register */
#define DMA_INTC(dmax)                  REG32((dmax) + 0x04U)    /*!< DMA interrupt flag clear register */

#define DMA_CH0CTL(dmax)                REG32((dmax) + 0x08U)    /*!< DMA channel 0 control register */
#define DMA_CH0CNT(dmax)                REG32((dmax) + 0x0CU)    /*!< DMA channel 0 counter register */
#define DMA_CH0PADDR(dmax)              REG32((dmax) + 0x10U)    /*!< DMA channel 0 peripheral base address register */
#define DMA_CH0MADDR(dmax)              REG32((dmax) + 0x14U)    /*!< DMA channel 0 memory base address register */

#define DMA_CH1CTL(dmax)                REG32((dmax) + 0x1CU)    /*!< DMA channel 1 control register */
#define DMA_CH1CNT(dmax)                REG32((dmax) + 0x20U)    /*!< DMA channel 1 counter register */
#define DMA_CH1PADDR(dmax)              REG32((dmax) + 0x24U)    /*!< DMA channel 1 peripheral base address register */
#define DMA_CH1MADDR(dmax)              REG32((dmax) + 0x28U)    /*!< DMA channel 1 memory base address register */

#define DMA_CH2CTL(dmax)                REG32((dmax) + 0x30U)    /*!< DMA channel 2 control register */
#define DMA_CH2CNT(dmax)                REG32((dmax) + 0x34U)    /*!< DMA channel 2 counter register */
#define DMA_CH2PADDR(dmax)              REG32((dmax) + 0x38U)    /*!< DMA channel 2 peripheral base address register */
#define DMA_CH2MADDR(dmax)              REG32((dmax) + 0x3CU)    /*!< DMA channel 2 memory base address register */

#define DMA_CH3CTL(dmax)                REG32((dmax) + 0x44U)    /*!< DMA channel 3 control register */
#define DMA_CH3CNT(dmax)                REG32((dmax) + 0x48U)    /*!< DMA channel 3 counter register */
#define DMA_CH3PADDR(dmax)              REG32((dmax) + 0x4CU)    /*!< DMA channel 3 peripheral base address register */
#define DMA_CH3MADDR(dmax)              REG32((dmax) + 0x50U)    /*!< DMA channel 3 memory base address register */

#define DMA_CH4CTL(dmax)                REG32((dmax) + 0x58U)    /*!< DMA channel 4 control register */
#define DMA_CH4CNT(dmax)                REG32((dmax) + 0x5CU)    /*!< DMA channel 4 counter register */
#define DMA_CH4PADDR(dmax)              REG32((dmax) + 0x60U)    /*!< DMA channel 4 peripheral base address register */
#define DMA_CH4MADDR(dmax)              REG32((dmax) + 0x64U)    /*!< DMA channel 4 memory base address register */

#define DMA_CH5CTL(dmax)                REG32((dmax) + 0x6CU)    /*!< DMA channel 5 control register */
#define DMA_CH5CNT(dmax)                REG32((dmax) + 0x70U)    /*!< DMA channel 5 counter register */
#define DMA_CH5PADDR(dmax)              REG32((dmax) + 0x74U)    /*!< DMA channel 5 peripheral base address register */
#define DMA_CH5MADDR(dmax)              REG32((dmax) + 0x78U)    /*!< DMA channel 5 memory base address register */

#define DMA_CH6CTL(dmax)                REG32((dmax) + 0x80U)    /*!< DMA channel 6 control register */
#define DMA_CH6CNT(dmax)                REG32((dmax) + 0x84U)    /*!< DMA channel 6 counter register */
#define DMA_CH6PADDR(dmax)              REG32((dmax) + 0x88U)    /*!< DMA channel 6 peripheral base address register */
#define DMA_CH6MADDR(dmax)              REG32((dmax) + 0x8CU)    /*!< DMA channel 6 memory base address register */

/* bits definitions */
/* DMA_INTF */
#define DMA_INTF_GIF                    BIT(0)                  /*!< global interrupt flag of channel */
#define DMA_INTF_FTFIF                  BIT(1)                  /*!< full transfer finish flag of channel */
#define DMA_INTF_HTFIF                  BIT(2)                  /*!< half transfer finish flag of channel */
#define DMA_INTF_ERRIF                  BIT(3)                  /*!< error flag of channel */

/* DMA_INTC */
#define DMA_INTC_GIFC                   BIT(0)                  /*!< clear global interrupt flag of channel */
#define DMA_INTC_FTFIFC                 BIT(1)                  /*!< clear transfer finish flag of channel */
#define DMA_INTC_HTFIFC                 BIT(2)                  /*!< clear half transfer finish flag of channel */
#define DMA_INTC_ERRIFC                 BIT(3)                  /*!< clear error flag of channel */

/* DMA_CHxCTL, x=0..6 */
#define DMA_CHXCTL_CHEN                 BIT(0)                  /*!< channel enable */
#define DMA_CHXCTL_FTFIE                BIT(1)                  /*!< enable bit for channel full transfer finish interrupt */
#define DMA_CHXCTL_HTFIE                BIT(2)                  /*!< enable bit for channel half transfer finish interrupt */
#define DMA_CHXCTL_ERRIE                BIT(3)                  /*!< enable bit for channel error interrupt */
#define DMA_CHXCTL_DIR                  BIT(4)                  /*!< transfer direction */
#define DMA_CHXCTL_CMEN                 BIT(5)                  /*!< circular mode enable */
#define DMA_CHXCTL_PNAGA                BIT(6)                  /*!< next address generation algorithm of peripheral */
#define DMA_CHXCTL_MNAGA                BIT(7)                  /*!< next address generation algorithm of memory */
#define DMA_CHXCTL_PWIDTH               BITS(8,9)               /*!< transfer data width of peripheral */
#define DMA_CHXCTL_MWIDTH               BITS(10,11)             /*!< transfer data width of memory */
#define DMA_CHXCTL_PRIO                 BITS(12,13)             /*!< priority level */
#define DMA_CHXCTL_M2M                  BIT(14)                 /*!< memory to memory mode */

/* DMA_CHxCNT, x=0..6 */
#define DMA_CHXCNT_CNT                  BITS(0,15)              /*!< transfer counter */

/* DMA_CHxPADDR, x=0..6 */
#define DMA_CHXPADDR_PADDR              BITS(0,31)              /*!< peripheral base address */

/* DMA_CHxMADDR, x=0..6 */
#define DMA_CHXMADDR_MADDR              BITS(0,31)              /*!< memory base address */

/* constants definitions */
/* DMA channel select */
typedef enum
{
    DMA_CH0 = 0,                /*!< DMA Channel0 */
    DMA_CH1,                    /*!< DMA Channel1 */
    DMA_CH2,                    /*!< DMA Channel2 */
    DMA_CH3,                    /*!< DMA Channel3 */
    DMA_CH4,                    /*!< DMA Channel4 */
    DMA_CH5,                    /*!< DMA Channel5 */
    DMA_CH6                     /*!< DMA Channel6 */
} dma_channel_enum;

/* DMA initialize struct */
typedef struct
{
    uint32_t periph_addr;       /*!< peripheral base address */
    uint32_t periph_width;      /*!< transfer data size of peripheral */
    uint32_t memory_addr;       /*!< memory base address */
    uint32_t memory_width;      /*!< transfer data size of memory */
    uint32_t number;            /*!< channel transfer number */
    uint32_t priority;          /*!< channel priority level */
    uint8_t periph_inc;         /*!< peripheral increasing mode */
    uint8_t memory_inc;         /*!< memory increasing mode */
    uint8_t direction;          /*!< channel data transfer direction */

} dma_parameter_struct;

#define DMA_FLAG_ADD(flag, shift)           ((flag) << ((shift) * 4U))                      /*!< DMA channel flag shift */

/* DMA_register address */
#define DMA_CHCTL(dma, channel)             REG32(((dma) + 0x08U) + 0x14U * (uint32_t)(channel))      /*!< the address of DMA channel CHXCTL register */
#define DMA_CHCNT(dma, channel)             REG32(((dma) + 0x0CU) + 0x14U * (uint32_t)(channel))      /*!< the address of DMA channel CHXCNT register */
#define DMA_CHPADDR(dma, channel)           REG32(((dma) + 0x10U) + 0x14U * (uint32_t)(channel))      /*!< the address of DMA channel CHXPADDR register */
#define DMA_CHMADDR(dma, channel)           REG32(((dma) + 0x14U) + 0x14U * (uint32_t)(channel))      /*!< the address of DMA channel CHXMADDR register */

/* DMA reset value */
#define DMA_CHCTL_RESET_VALUE               ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXCTL register */
#define DMA_CHCNT_RESET_VALUE               ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXCNT register */
#define DMA_CHPADDR_RESET_VALUE             ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXPADDR register */
#define DMA_CHMADDR_RESET_VALUE             ((uint32_t)0x00000000U)                         /*!< the reset value of DMA channel CHXMADDR register */
#define DMA_CHINTF_RESET_VALUE              (DMA_INTF_GIF | DMA_INTF_FTFIF | \
                                             DMA_INTF_HTFIF | DMA_INTF_ERRIF)               /*!< clear DMA channel DMA_INTF register */

/* DMA_INTF register */
/* interrupt flag bits */
#define DMA_INT_FLAG_G                      DMA_INTF_GIF                                    /*!< global interrupt flag of channel */
#define DMA_INT_FLAG_FTF                    DMA_INTF_FTFIF                                  /*!< full transfer finish interrupt flag of channel */
#define DMA_INT_FLAG_HTF                    DMA_INTF_HTFIF                                  /*!< half transfer finish interrupt flag of channel */
#define DMA_INT_FLAG_ERR                    DMA_INTF_ERRIF                                  /*!< error interrupt flag of channel */

/* flag bits */
#define DMA_FLAG_G                          DMA_INTF_GIF                                    /*!< global interrupt flag of channel */
#define DMA_FLAG_FTF                        DMA_INTF_FTFIF                                  /*!< full transfer finish flag of channel */
#define DMA_FLAG_HTF                        DMA_INTF_HTFIF                                  /*!< half transfer finish flag of channel */
#define DMA_FLAG_ERR                        DMA_INTF_ERRIF                                  /*!< error flag of channel */

/* DMA_CHxCTL register */
/* interrupt enable bits */
#define DMA_INT_FTF                         DMA_CHXCTL_FTFIE                                /*!< enable bit for channel full transfer finish interrupt */
#define DMA_INT_HTF                         DMA_CHXCTL_HTFIE                                /*!< enable bit for channel half transfer finish interrupt */
#define DMA_INT_ERR                         DMA_CHXCTL_ERRIE                                /*!< enable bit for channel error interrupt */

/* transfer direction */
#define DMA_PERIPHERAL_TO_MEMORY            ((uint8_t)0x00U)                         		/*!< read from peripheral and write to memory */
#define DMA_MEMORY_TO_PERIPHERAL            ((uint8_t)0x01U)                         		/*!< read from memory and write to peripheral */

/* peripheral increasing mode */
#define DMA_PERIPH_INCREASE_DISABLE         ((uint8_t)0x00U)                                /*!< next address of peripheral is fixed address mode */
#define DMA_PERIPH_INCREASE_ENABLE          ((uint8_t)0x01U)                                /*!< next address of peripheral is increasing address mode */

/* memory increasing mode */
#define DMA_MEMORY_INCREASE_DISABLE         ((uint8_t)0x00U)                                /*!< next address of memory is fixed address mode */
#define DMA_MEMORY_INCREASE_ENABLE          ((uint8_t)0x01U)                                /*!< next address of memory is increasing address mode */

/* transfer data size of peripheral */
#define CHCTL_PWIDTH(regval)                (BITS(8,9) & ((uint32_t)(regval) << 8))          /*!< transfer data size of peripheral */
#define DMA_PERIPHERAL_WIDTH_8BIT           CHCTL_PWIDTH(0U)                                 /*!< transfer data size of peripheral is 8-bit */
#define DMA_PERIPHERAL_WIDTH_16BIT          CHCTL_PWIDTH(1U)                                 /*!< transfer data size of peripheral is 16-bit */
#define DMA_PERIPHERAL_WIDTH_32BIT          CHCTL_PWIDTH(2U)                                 /*!< transfer data size of peripheral is 32-bit */

/* transfer data size of memory */
#define CHCTL_MWIDTH(regval)                (BITS(10,11) & ((uint32_t)(regval) << 10))       /*!< transfer data size of memory */
#define DMA_MEMORY_WIDTH_8BIT               CHCTL_MWIDTH(0U)                                 /*!< transfer data size of memory is 8-bit */
#define DMA_MEMORY_WIDTH_16BIT              CHCTL_MWIDTH(1U)                                 /*!< transfer data size of memory is 16-bit */
#define DMA_MEMORY_WIDTH_32BIT              CHCTL_MWIDTH(2U)                                 /*!< transfer data size of memory is 32-bit */

/* channel priority level */
#define CHCTL_PRIO(regval)                  (BITS(12,13) & ((uint32_t)(regval) << 12))       /*!< DMA channel priority level */
#define DMA_PRIORITY_LOW                    CHCTL_PRIO(0U)                                   /*!< low priority */
#define DMA_PRIORITY_MEDIUM                 CHCTL_PRIO(1U)                                   /*!< medium priority */
#define DMA_PRIORITY_HIGH                   CHCTL_PRIO(2U)                                   /*!< high priority */
#define DMA_PRIORITY_ULTRA_HIGH             CHCTL_PRIO(3U)                                   /*!< ultra high priority */

/* memory to memory mode */
#define DMA_MEMORY_TO_MEMORY_DISABLE        ((uint32_t)0x00000000U)                         /*!< disable memory to memory mode */
#define DMA_MEMORY_TO_MEMORY_ENABLE         ((uint32_t)0x00000001U)                         /*!< enable memory to memory mode */

/* DMA_CHxCNT register */
/* transfer counter */
#define DMA_CHANNEL_CNT_MASK                DMA_CHXCNT_CNT                                  /*!< transfer counter mask */


////////////////////////////////////////////////////////////////////////
/* ECLIC constants definitions */
#define ECLIC_PRIGROUP_LEVEL0_PRIO4        0    /*!< 0 bits for level 4 bits for priority */
#define ECLIC_PRIGROUP_LEVEL1_PRIO3        1    /*!< 1 bits for level 3 bits for priority */
#define ECLIC_PRIGROUP_LEVEL2_PRIO2        2    /*!< 2 bits for level 2 bits for priority */
#define ECLIC_PRIGROUP_LEVEL3_PRIO1        3    /*!< 3 bits for level 1 bits for priority */
#define ECLIC_PRIGROUP_LEVEL4_PRIO0        4    /*!< 4 bits for level 0 bits for priority */


////////////////////////////////////////////////////////////////////////
/* EXMC definitions */
#define EXMC                              (EXMC_BASE)                   /*!< EXMC register base address */

/* registers definitions */
/* NOR/PSRAM */
#define EXMC_SNCTL0                       REG32(EXMC + 0x00U)           /*!< EXMC SRAM/NOR flash control register 0 */
#define EXMC_SNTCFG0                      REG32(EXMC + 0x04U)           /*!< EXMC SRAM/NOR flash timing configuration register 0 */
#define EXMC_SNWTCFG0                     REG32(EXMC + 0x104U)          /*!< EXMC SRAM/NOR flash write timing configuration register 0 */

/* bits definitions */
/* NOR/PSRAM */
/* EXMC_SNCTLx, x=0 */
#define EXMC_SNCTL_NRBKEN                 BIT(0)                        /*!< NOR bank enable */
#define EXMC_SNCTL_NRMUX                  BIT(1)                        /*!< NOR bank memory address/data multiplexing */
#define EXMC_SNCTL_NRTP                   BITS(2,3)                     /*!< NOR bank memory type */
#define EXMC_SNCTL_NRW                    BITS(4,5)                     /*!< NOR bank memory data bus width */
#define EXMC_SNCTL_NREN                   BIT(6)                        /*!< NOR flash access enable */
#define EXMC_SNCTL_NRWTPOL                BIT(9)                        /*!< NWAIT signal polarity */
#define EXMC_SNCTL_WREN                   BIT(12)                       /*!< write enable */
#define EXMC_SNCTL_NRWTEN                 BIT(13)                       /*!< NWAIT signal enable */
#define EXMC_SNCTL_ASYNCWAIT              BIT(15)                       /*!< asynchronous wait */

/* EXMC_SNTCFGx, x=0 */
#define EXMC_SNTCFG_ASET                  BITS(0,3)                     /*!< address setup time */
#define EXMC_SNTCFG_AHLD                  BITS(4,7)                     /*!< address hold time */
#define EXMC_SNTCFG_DSET                  BITS(8,15)                    /*!< data setup time */
#define EXMC_SNTCFG_BUSLAT                BITS(16,19)                   /*!< bus latency */

/* constants definitions */
/* EXMC NOR/SRAM timing initialize struct */
typedef struct
{
    uint32_t bus_latency;                                               /*!< configure the bus latency */
    uint32_t asyn_data_setuptime;                                       /*!< configure the data setup time,asynchronous access mode valid */
    uint32_t asyn_address_holdtime;                                     /*!< configure the address hold time,asynchronous access mode valid */
    uint32_t asyn_address_setuptime;                                    /*!< configure the data setup time,asynchronous access mode valid */
}exmc_norsram_timing_parameter_struct;

/* EXMC NOR/SRAM initialize struct */
typedef struct
{
    uint32_t norsram_region;                                            /*!< select the region of EXMC NOR/SRAM bank */
    ControlStatus asyn_wait;                                            /*!< enable or disable the asynchronous wait function */
    ControlStatus nwait_signal;                                         /*!< enable or disable the NWAIT signal */
    ControlStatus memory_write;                                         /*!< enable or disable the write operation */
    uint32_t nwait_polarity;                                            /*!< specifies the polarity of NWAIT signal from memory */
    uint32_t databus_width;                                             /*!< specifies the databus width of external memory */
    uint32_t memory_type;                                               /*!< specifies the type of external memory */
    ControlStatus address_data_mux;                                     /*!< specifies whether the data bus and address bus are multiplexed */
    exmc_norsram_timing_parameter_struct* read_write_timing;            /*!< timing parameters for read and write */
}exmc_norsram_parameter_struct;

/* EXMC register address */
#define EXMC_SNCTL(region)                REG32(EXMC + 0x08U * (region))                  /*!< EXMC SRAM/NOR flash control register */
#define EXMC_SNTCFG(region)               REG32(EXMC + 0x04U + 0x08U * (region))          /*!< EXMC SRAM/NOR flash timing configuration register */

/* NOR bank memory data bus width */
#define SNCTL_NRW(regval)                 (BITS(4,5) & ((uint32_t)(regval) << 4))
#define EXMC_NOR_DATABUS_WIDTH_8B         SNCTL_NRW(0)                  /*!< NOR data width 8 bits */
#define EXMC_NOR_DATABUS_WIDTH_16B        SNCTL_NRW(1)                  /*!< NOR data width 16 bits */

/* NOR bank memory type */
#define SNCTL_NRTP(regval)                (BITS(2,3) & ((uint32_t)(regval) << 2))
#define EXMC_MEMORY_TYPE_SRAM             SNCTL_NRTP(0)                 /*!< SRAM,ROM */
#define EXMC_MEMORY_TYPE_PSRAM            SNCTL_NRTP(1)                 /*!< PSRAM,CRAM */
#define EXMC_MEMORY_TYPE_NOR              SNCTL_NRTP(2)                 /*!< NOR flash */

/* EXMC NOR/SRAM bank region definition */
#define EXMC_BANK0_NORSRAM_REGION0        ((uint32_t)0x00000000U)       /*!< bank0 NOR/SRAM region0 */

/* EXMC NWAIT signal polarity configuration */
#define EXMC_NWAIT_POLARITY_LOW           ((uint32_t)0x00000000U)       /*!< low level is active of NWAIT */
#define EXMC_NWAIT_POLARITY_HIGH          ((uint32_t)0x00000200U)       /*!< high level is active of NWAIT */


////////////////////////////////////////////////////////////////////////
/* EXTI definitions */
#define EXTI                         EXTI_BASE

/* registers definitions */
#define EXTI_INTEN                   REG32(EXTI + 0x00U)      /*!< interrupt enable register */
#define EXTI_EVEN                    REG32(EXTI + 0x04U)      /*!< event enable register */
#define EXTI_RTEN                    REG32(EXTI + 0x08U)      /*!< rising edge trigger enable register */
#define EXTI_FTEN                    REG32(EXTI + 0x0CU)      /*!< falling trigger enable register */
#define EXTI_SWIEV                   REG32(EXTI + 0x10U)      /*!< software interrupt event register */
#define EXTI_PD                      REG32(EXTI + 0x14U)      /*!< pending register */

/* bits definitions */
/* EXTI_INTEN */
#define EXTI_INTEN_INTEN0            BIT(0)                   /*!< interrupt from line 0 */
#define EXTI_INTEN_INTEN1            BIT(1)                   /*!< interrupt from line 1 */
#define EXTI_INTEN_INTEN2            BIT(2)                   /*!< interrupt from line 2 */
#define EXTI_INTEN_INTEN3            BIT(3)                   /*!< interrupt from line 3 */
#define EXTI_INTEN_INTEN4            BIT(4)                   /*!< interrupt from line 4 */
#define EXTI_INTEN_INTEN5            BIT(5)                   /*!< interrupt from line 5 */
#define EXTI_INTEN_INTEN6            BIT(6)                   /*!< interrupt from line 6 */
#define EXTI_INTEN_INTEN7            BIT(7)                   /*!< interrupt from line 7 */
#define EXTI_INTEN_INTEN8            BIT(8)                   /*!< interrupt from line 8 */
#define EXTI_INTEN_INTEN9            BIT(9)                   /*!< interrupt from line 9 */
#define EXTI_INTEN_INTEN10           BIT(10)                  /*!< interrupt from line 10 */
#define EXTI_INTEN_INTEN11           BIT(11)                  /*!< interrupt from line 11 */
#define EXTI_INTEN_INTEN12           BIT(12)                  /*!< interrupt from line 12 */
#define EXTI_INTEN_INTEN13           BIT(13)                  /*!< interrupt from line 13 */
#define EXTI_INTEN_INTEN14           BIT(14)                  /*!< interrupt from line 14 */
#define EXTI_INTEN_INTEN15           BIT(15)                  /*!< interrupt from line 15 */
#define EXTI_INTEN_INTEN16           BIT(16)                  /*!< interrupt from line 16 */
#define EXTI_INTEN_INTEN17           BIT(17)                  /*!< interrupt from line 17 */
#define EXTI_INTEN_INTEN18           BIT(18)                  /*!< interrupt from line 18 */

/* EXTI_EVEN */
#define EXTI_EVEN_EVEN0              BIT(0)                   /*!< event from line 0 */
#define EXTI_EVEN_EVEN1              BIT(1)                   /*!< event from line 1 */
#define EXTI_EVEN_EVEN2              BIT(2)                   /*!< event from line 2 */
#define EXTI_EVEN_EVEN3              BIT(3)                   /*!< event from line 3 */
#define EXTI_EVEN_EVEN4              BIT(4)                   /*!< event from line 4 */
#define EXTI_EVEN_EVEN5              BIT(5)                   /*!< event from line 5 */
#define EXTI_EVEN_EVEN6              BIT(6)                   /*!< event from line 6 */
#define EXTI_EVEN_EVEN7              BIT(7)                   /*!< event from line 7 */
#define EXTI_EVEN_EVEN8              BIT(8)                   /*!< event from line 8 */
#define EXTI_EVEN_EVEN9              BIT(9)                   /*!< event from line 9 */
#define EXTI_EVEN_EVEN10             BIT(10)                  /*!< event from line 10 */
#define EXTI_EVEN_EVEN11             BIT(11)                  /*!< event from line 11 */
#define EXTI_EVEN_EVEN12             BIT(12)                  /*!< event from line 12 */
#define EXTI_EVEN_EVEN13             BIT(13)                  /*!< event from line 13 */
#define EXTI_EVEN_EVEN14             BIT(14)                  /*!< event from line 14 */
#define EXTI_EVEN_EVEN15             BIT(15)                  /*!< event from line 15 */
#define EXTI_EVEN_EVEN16             BIT(16)                  /*!< event from line 16 */
#define EXTI_EVEN_EVEN17             BIT(17)                  /*!< event from line 17 */
#define EXTI_EVEN_EVEN18             BIT(18)                  /*!< event from line 18 */

/* EXTI_RTEN */
#define EXTI_RTEN_RTEN0              BIT(0)                   /*!< rising edge from line 0 */
#define EXTI_RTEN_RTEN1              BIT(1)                   /*!< rising edge from line 1 */
#define EXTI_RTEN_RTEN2              BIT(2)                   /*!< rising edge from line 2 */
#define EXTI_RTEN_RTEN3              BIT(3)                   /*!< rising edge from line 3 */
#define EXTI_RTEN_RTEN4              BIT(4)                   /*!< rising edge from line 4 */
#define EXTI_RTEN_RTEN5              BIT(5)                   /*!< rising edge from line 5 */
#define EXTI_RTEN_RTEN6              BIT(6)                   /*!< rising edge from line 6 */
#define EXTI_RTEN_RTEN7              BIT(7)                   /*!< rising edge from line 7 */
#define EXTI_RTEN_RTEN8              BIT(8)                   /*!< rising edge from line 8 */
#define EXTI_RTEN_RTEN9              BIT(9)                   /*!< rising edge from line 9 */
#define EXTI_RTEN_RTEN10             BIT(10)                  /*!< rising edge from line 10 */
#define EXTI_RTEN_RTEN11             BIT(11)                  /*!< rising edge from line 11 */
#define EXTI_RTEN_RTEN12             BIT(12)                  /*!< rising edge from line 12 */
#define EXTI_RTEN_RTEN13             BIT(13)                  /*!< rising edge from line 13 */
#define EXTI_RTEN_RTEN14             BIT(14)                  /*!< rising edge from line 14 */
#define EXTI_RTEN_RTEN15             BIT(15)                  /*!< rising edge from line 15 */
#define EXTI_RTEN_RTEN16             BIT(16)                  /*!< rising edge from line 16 */
#define EXTI_RTEN_RTEN17             BIT(17)                  /*!< rising edge from line 17 */
#define EXTI_RTEN_RTEN18             BIT(18)                  /*!< rising edge from line 18 */

/* EXTI_FTEN */
#define EXTI_FTEN_FTEN0              BIT(0)                   /*!< falling edge from line 0 */
#define EXTI_FTEN_FTEN1              BIT(1)                   /*!< falling edge from line 1 */
#define EXTI_FTEN_FTEN2              BIT(2)                   /*!< falling edge from line 2 */
#define EXTI_FTEN_FTEN3              BIT(3)                   /*!< falling edge from line 3 */
#define EXTI_FTEN_FTEN4              BIT(4)                   /*!< falling edge from line 4 */
#define EXTI_FTEN_FTEN5              BIT(5)                   /*!< falling edge from line 5 */
#define EXTI_FTEN_FTEN6              BIT(6)                   /*!< falling edge from line 6 */
#define EXTI_FTEN_FTEN7              BIT(7)                   /*!< falling edge from line 7 */
#define EXTI_FTEN_FTEN8              BIT(8)                   /*!< falling edge from line 8 */
#define EXTI_FTEN_FTEN9              BIT(9)                   /*!< falling edge from line 9 */
#define EXTI_FTEN_FTEN10             BIT(10)                  /*!< falling edge from line 10 */
#define EXTI_FTEN_FTEN11             BIT(11)                  /*!< falling edge from line 11 */
#define EXTI_FTEN_FTEN12             BIT(12)                  /*!< falling edge from line 12 */
#define EXTI_FTEN_FTEN13             BIT(13)                  /*!< falling edge from line 13 */
#define EXTI_FTEN_FTEN14             BIT(14)                  /*!< falling edge from line 14 */
#define EXTI_FTEN_FTEN15             BIT(15)                  /*!< falling edge from line 15 */
#define EXTI_FTEN_FTEN16             BIT(16)                  /*!< falling edge from line 16 */
#define EXTI_FTEN_FTEN17             BIT(17)                  /*!< falling edge from line 17 */
#define EXTI_FTEN_FTEN18             BIT(18)                  /*!< falling edge from line 18 */

/* EXTI_SWIEV */
#define EXTI_SWIEV_SWIEV0            BIT(0)                   /*!< software interrupt/event request from line 0 */
#define EXTI_SWIEV_SWIEV1            BIT(1)                   /*!< software interrupt/event request from line 1 */
#define EXTI_SWIEV_SWIEV2            BIT(2)                   /*!< software interrupt/event request from line 2 */
#define EXTI_SWIEV_SWIEV3            BIT(3)                   /*!< software interrupt/event request from line 3 */
#define EXTI_SWIEV_SWIEV4            BIT(4)                   /*!< software interrupt/event request from line 4 */
#define EXTI_SWIEV_SWIEV5            BIT(5)                   /*!< software interrupt/event request from line 5 */
#define EXTI_SWIEV_SWIEV6            BIT(6)                   /*!< software interrupt/event request from line 6 */
#define EXTI_SWIEV_SWIEV7            BIT(7)                   /*!< software interrupt/event request from line 7 */
#define EXTI_SWIEV_SWIEV8            BIT(8)                   /*!< software interrupt/event request from line 8 */
#define EXTI_SWIEV_SWIEV9            BIT(9)                   /*!< software interrupt/event request from line 9 */
#define EXTI_SWIEV_SWIEV10           BIT(10)                  /*!< software interrupt/event request from line 10 */
#define EXTI_SWIEV_SWIEV11           BIT(11)                  /*!< software interrupt/event request from line 11 */
#define EXTI_SWIEV_SWIEV12           BIT(12)                  /*!< software interrupt/event request from line 12 */
#define EXTI_SWIEV_SWIEV13           BIT(13)                  /*!< software interrupt/event request from line 13 */
#define EXTI_SWIEV_SWIEV14           BIT(14)                  /*!< software interrupt/event request from line 14 */
#define EXTI_SWIEV_SWIEV15           BIT(15)                  /*!< software interrupt/event request from line 15 */
#define EXTI_SWIEV_SWIEV16           BIT(16)                  /*!< software interrupt/event request from line 16 */
#define EXTI_SWIEV_SWIEV17           BIT(17)                  /*!< software interrupt/event request from line 17 */
#define EXTI_SWIEV_SWIEV18           BIT(18)                  /*!< software interrupt/event request from line 18 */

/* EXTI_PD */
#define EXTI_PD_PD0                  BIT(0)                   /*!< interrupt/event pending status from line 0 */
#define EXTI_PD_PD1                  BIT(1)                   /*!< interrupt/event pending status from line 1 */
#define EXTI_PD_PD2                  BIT(2)                   /*!< interrupt/event pending status from line 2 */
#define EXTI_PD_PD3                  BIT(3)                   /*!< interrupt/event pending status from line 3 */
#define EXTI_PD_PD4                  BIT(4)                   /*!< interrupt/event pending status from line 4 */
#define EXTI_PD_PD5                  BIT(5)                   /*!< interrupt/event pending status from line 5 */
#define EXTI_PD_PD6                  BIT(6)                   /*!< interrupt/event pending status from line 6 */
#define EXTI_PD_PD7                  BIT(7)                   /*!< interrupt/event pending status from line 7 */
#define EXTI_PD_PD8                  BIT(8)                   /*!< interrupt/event pending status from line 8 */
#define EXTI_PD_PD9                  BIT(9)                   /*!< interrupt/event pending status from line 9 */
#define EXTI_PD_PD10                 BIT(10)                  /*!< interrupt/event pending status from line 10 */
#define EXTI_PD_PD11                 BIT(11)                  /*!< interrupt/event pending status from line 11 */
#define EXTI_PD_PD12                 BIT(12)                  /*!< interrupt/event pending status from line 12 */
#define EXTI_PD_PD13                 BIT(13)                  /*!< interrupt/event pending status from line 13 */
#define EXTI_PD_PD14                 BIT(14)                  /*!< interrupt/event pending status from line 14 */
#define EXTI_PD_PD15                 BIT(15)                  /*!< interrupt/event pending status from line 15 */
#define EXTI_PD_PD16                 BIT(16)                  /*!< interrupt/event pending status from line 16 */
#define EXTI_PD_PD17                 BIT(17)                  /*!< interrupt/event pending status from line 17 */
#define EXTI_PD_PD18                 BIT(18)                  /*!< interrupt/event pending status from line 18 */

/* constants definitions */
/* EXTI line number */
typedef enum {
    EXTI_0 = BIT(0),                                          /*!< EXTI line 0 */
    EXTI_1 = BIT(1),                                          /*!< EXTI line 1 */
    EXTI_2 = BIT(2),                                          /*!< EXTI line 2 */
    EXTI_3 = BIT(3),                                          /*!< EXTI line 3 */
    EXTI_4 = BIT(4),                                          /*!< EXTI line 4 */
    EXTI_5 = BIT(5),                                          /*!< EXTI line 5 */
    EXTI_6 = BIT(6),                                          /*!< EXTI line 6 */
    EXTI_7 = BIT(7),                                          /*!< EXTI line 7 */
    EXTI_8 = BIT(8),                                          /*!< EXTI line 8 */
    EXTI_9 = BIT(9),                                          /*!< EXTI line 9 */
    EXTI_10 = BIT(10),                                        /*!< EXTI line 10 */
    EXTI_11 = BIT(11),                                        /*!< EXTI line 11 */
    EXTI_12 = BIT(12),                                        /*!< EXTI line 12 */
    EXTI_13 = BIT(13),                                        /*!< EXTI line 13 */
    EXTI_14 = BIT(14),                                        /*!< EXTI line 14 */
    EXTI_15 = BIT(15),                                        /*!< EXTI line 15 */
    EXTI_16 = BIT(16),                                        /*!< EXTI line 16 */
    EXTI_17 = BIT(17),                                        /*!< EXTI line 17 */
    EXTI_18 = BIT(18),                                        /*!< EXTI line 18 */
} exti_line_enum;

/* external interrupt and event  */
typedef enum {
    EXTI_INTERRUPT = 0,                                       /*!< EXTI interrupt mode */
    EXTI_EVENT                                                /*!< EXTI event mode */
} exti_mode_enum;

/* interrupt trigger mode */
typedef enum {
    EXTI_TRIG_RISING = 0,                                     /*!< EXTI rising edge trigger */
    EXTI_TRIG_FALLING,                                        /*!< EXTI falling edge trigger */
    EXTI_TRIG_BOTH,                                           /*!< EXTI rising edge and falling edge trigger */
    EXTI_TRIG_NONE                                            /*!< without rising edge or falling edge trigger */
} exti_trig_type_enum;


////////////////////////////////////////////////////////////////////////
/* FMC and option byte definition */
#define FMC                        FMC_BASE                        /*!< FMC register base address */
#define OB                         OB_BASE                         /*!< option bytes base address */

/* registers definitions */
#define FMC_WS                     REG32((FMC) + 0x00U)            /*!< FMC wait state register */
#define FMC_KEY                    REG32((FMC) + 0x04U)            /*!< FMC unlock key register */
#define FMC_OBKEY                  REG32((FMC) + 0x08U)            /*!< FMC option bytes unlock key register */
#define FMC_STAT                   REG32((FMC) + 0x0CU)            /*!< FMC status register */
#define FMC_CTL                    REG32((FMC) + 0x10U)            /*!< FMC control register */
#define FMC_ADDR                   REG32((FMC) + 0x14U)            /*!< FMC address register */
#define FMC_OBSTAT                 REG32((FMC) + 0x1CU)            /*!< FMC option bytes status register */
#define FMC_WP                     REG32((FMC) + 0x20U)            /*!< FMC erase/program protection register */
#define FMC_PID                    REG32((FMC) + 0x100U)           /*!< FMC product ID register */

#define OB_SPC                     REG16((OB) + 0x00U)             /*!< option byte security protection value */
#define OB_USER                    REG16((OB) + 0x02U)             /*!< option byte user value*/
#define OB_WP0                     REG16((OB) + 0x08U)             /*!< option byte write protection 0 */
#define OB_WP1                     REG16((OB) + 0x0AU)             /*!< option byte write protection 1 */
#define OB_WP2                     REG16((OB) + 0x0CU)             /*!< option byte write protection 2 */
#define OB_WP3                     REG16((OB) + 0x0EU)             /*!< option byte write protection 3 */

/* bits definitions */
/* FMC_WS */
#define FMC_WS_WSCNT               BITS(0,2)                       /*!< wait state counter */

/* FMC_KEY */
#define FMC_KEY_KEY                BITS(0,31)                      /*!< FMC_CTL unlock key bits */

/* FMC_OBKEY */
#define FMC_OBKEY_OBKEY            BITS(0,31)                      /*!< option bytes unlock key bits */

/* FMC_STAT */
#define FMC_STAT_BUSY              BIT(0)                          /*!< flash busy flag bit */
#define FMC_STAT_PGERR             BIT(2)                          /*!< flash program error flag bit */
#define FMC_STAT_WPERR             BIT(4)                          /*!< erase/program protection error flag bit */
#define FMC_STAT_ENDF              BIT(5)                          /*!< end of operation flag bit */

/* FMC_CTL */
#define FMC_CTL_PG                 BIT(0)                          /*!< main flash program command bit */
#define FMC_CTL_PER                BIT(1)                          /*!< main flash page erase command bit */
#define FMC_CTL_MER                BIT(2)                          /*!< main flash mass erase command bit */
#define FMC_CTL_OBPG               BIT(4)                          /*!< option bytes program command bit */
#define FMC_CTL_OBER               BIT(5)                          /*!< option bytes erase command bit */
#define FMC_CTL_START              BIT(6)                          /*!< send erase command to FMC bit */
#define FMC_CTL_LK                 BIT(7)                          /*!< FMC_CTL lock bit */
#define FMC_CTL_OBWEN              BIT(9)                          /*!< option bytes erase/program enable bit */
#define FMC_CTL_ERRIE              BIT(10)                         /*!< error interrupt enable bit */
#define FMC_CTL_ENDIE              BIT(12)                         /*!< end of operation interrupt enable bit */

/* FMC_ADDR */
#define FMC_ADDR0_ADDR             BITS(0,31)                      /*!< Flash erase/program command address bits */

/* FMC_OBSTAT */
#define FMC_OBSTAT_OBERR           BIT(0)                          /*!< option bytes read error bit. */
#define FMC_OBSTAT_SPC             BIT(1)                          /*!< option bytes security protection code */
#define FMC_OBSTAT_USER            BITS(2,9)                       /*!< store USER of option bytes block after system reset */
#define FMC_OBSTAT_DATA            BITS(10,25)                     /*!< store DATA of option bytes block after system reset. */

/* FMC_WP */
#define FMC_WP_WP                  BITS(0,31)                      /*!< store WP of option bytes block after system reset */

/* FMC_WSEN */
#define FMC_WSEN_WSEN              BIT(0)                          /*!< FMC wait state enable bit */

/* FMC_PID */
#define FMC_PID_PID                BITS(0,31)                      /*!< product ID bits */

/* constants definitions */
/* define the FMC bit position and its register index offset */
#define FMC_REGIDX_BIT(regidx, bitpos)              (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define FMC_REG_VAL(offset)                         (REG32(FMC + ((uint32_t)(offset) >> 6)))
#define FMC_BIT_POS(val)                            ((uint32_t)(val) & 0x1FU)
#define FMC_REGIDX_BITS(regidx, bitpos0, bitpos1)   (((uint32_t)(regidx) << 12) | ((uint32_t)(bitpos0) << 6) | (uint32_t)(bitpos1))
#define FMC_REG_VALS(offset)                        (REG32(FMC + ((uint32_t)(offset) >> 12)))
#define FMC_BIT_POS0(val)                           (((uint32_t)(val) >> 6) & 0x1FU)
#define FMC_BIT_POS1(val)                           ((uint32_t)(val) & 0x1FU)
#define FMC_REG_OFFSET_GET(flag)                    ((uint32_t)(flag) >> 12)

/* configuration register */
#define FMC_STAT_REG_OFFSET        0x0CU                          /*!< status register offset */
#define FMC_CTL_REG_OFFSET         0x10U                          /*!< control register offset */
#define FMC_OBSTAT_REG_OFFSET      0x1CU                          /*!< option byte status register offset */

/* fmc state */
typedef enum
{
    FMC_READY,                                                    /*!< the operation has been completed */
    FMC_BUSY,                                                     /*!< the operation is in progress */
    FMC_PGERR,                                                    /*!< program error */
    FMC_WPERR,                                                    /*!< erase/program protection error */
    FMC_TOERR,                                                    /*!< timeout error */
}fmc_state_enum;

/* FMC interrupt enable */
typedef enum
{
    FMC_INT_END     = FMC_CTL_ENDIE,                              /*!< enable FMC end of program interrupt */
    FMC_INT_ERR     = FMC_CTL_ERRIE,                              /*!< enable FMC error interrupt */
}fmc_int_enum;

/* FMC flags */
typedef enum
{
    FMC_FLAG_BUSY   = FMC_STAT_BUSY,                              /*!< FMC busy flag */
    FMC_FLAG_PGERR  = FMC_STAT_PGERR,                             /*!< FMC operation error flag */
    FMC_FLAG_WPERR  = FMC_STAT_WPERR,                             /*!< FMC erase/program protection error flag */
    FMC_FLAG_END    = FMC_STAT_ENDF,                              /*!< FMC end of operation flag */
}fmc_flag_enum;

/* FMC interrupt flags */
typedef enum
{
    FMC_INT_FLAG_PGERR  = FMC_STAT_PGERR,                         /*!< FMC operation error interrupt flag */
    FMC_INT_FLAG_WPERR  = FMC_STAT_WPERR,                         /*!< FMC erase/program protection error interrupt flag */
    FMC_INT_FLAG_END    = FMC_STAT_ENDF,                          /*!< FMC end of operation interrupt flag */
}fmc_interrupt_flag_enum;

/* unlock key */
#define UNLOCK_KEY0                ((uint32_t)0x45670123U)        /*!< unlock key 0 */
#define UNLOCK_KEY1                ((uint32_t)0xCDEF89ABU)        /*!< unlock key 1 */

/* FMC wait state counter */
#define WS_WSCNT(regval)           (BITS(0,2) & ((uint32_t)(regval)))
#define WS_WSCNT_0                 WS_WSCNT(0)                    /*!< FMC 0 wait */
#define WS_WSCNT_1                 WS_WSCNT(1)                    /*!< FMC 1 wait */
#define WS_WSCNT_2                 WS_WSCNT(2)                    /*!< FMC 2 wait */

/* option bytes software/hardware free watch dog timer */  
#define OB_FWDGT_SW                ((uint8_t)0x01U)               /*!< software free watchdog */
#define OB_FWDGT_HW                ((uint8_t)0x00U)               /*!< hardware free watchdog */

/* option bytes reset or not entering deep sleep mode */
#define OB_DEEPSLEEP_NRST          ((uint8_t)0x02U)               /*!< no reset when entering deepsleep mode */
#define OB_DEEPSLEEP_RST           ((uint8_t)0x00U)               /*!< generate a reset instead of entering deepsleep mode */

/* option bytes reset or not entering standby mode */
#define OB_STDBY_NRST              ((uint8_t)0x04U)               /*!< no reset when entering deepsleep mode */
#define OB_STDBY_RST               ((uint8_t)0x00U)               /*!< generate a reset instead of entering standby mode */

/* option bytes boot bank value */
#define OB_BOOT_B0                 ((uint8_t)0x08U)               /*!< boot from bank0 */

#define OB_USER_MASK               ((uint8_t)0xF0U)               /*!< MASK value */

/* read protect configure */
#define FMC_NSPC                   ((uint8_t)0xA5U)               /*!< no security protection */
#define FMC_USPC                   ((uint8_t)0xBBU)               /*!< under security protection */

/* OB_SPC */
#define OB_SPC_SPC                 ((uint32_t)0x000000FFU)        /*!< option byte security protection value */
#define OB_SPC_SPC_N               ((uint32_t)0x0000FF00U)        /*!< option byte security protection complement value */

/* OB_USER */
#define OB_USER_USER               ((uint32_t)0x00FF0000U)        /*!< user option value */
#define OB_USER_USER_N             ((uint32_t)0xFF000000U)        /*!< user option complement value */

/* OB_WP0 */
#define OB_WP0_WP0                 ((uint32_t)0x000000FFU)        /*!< FMC write protection option value */

/* OB_WP1 */
#define OB_WP1_WP1                 ((uint32_t)0x0000FF00U)        /*!< FMC write protection option complement value */

/* OB_WP2 */
#define OB_WP2_WP2                 ((uint32_t)0x00FF0000U)        /*!< FMC write protection option value */

/* OB_WP3 */
#define OB_WP3_WP3                 ((uint32_t)0xFF000000U)        /*!< FMC write protection option complement value */

/* option bytes write protection */
#define OB_WP_0                    ((uint32_t)0x00000001U)        /*!< erase/program protection of sector 0  */
#define OB_WP_1                    ((uint32_t)0x00000002U)        /*!< erase/program protection of sector 1  */
#define OB_WP_2                    ((uint32_t)0x00000004U)        /*!< erase/program protection of sector 2  */
#define OB_WP_3                    ((uint32_t)0x00000008U)        /*!< erase/program protection of sector 3  */
#define OB_WP_4                    ((uint32_t)0x00000010U)        /*!< erase/program protection of sector 4  */
#define OB_WP_5                    ((uint32_t)0x00000020U)        /*!< erase/program protection of sector 5  */
#define OB_WP_6                    ((uint32_t)0x00000040U)        /*!< erase/program protection of sector 6  */
#define OB_WP_7                    ((uint32_t)0x00000080U)        /*!< erase/program protection of sector 7  */
#define OB_WP_8                    ((uint32_t)0x00000100U)        /*!< erase/program protection of sector 8  */
#define OB_WP_9                    ((uint32_t)0x00000200U)        /*!< erase/program protection of sector 9  */
#define OB_WP_10                   ((uint32_t)0x00000400U)        /*!< erase/program protection of sector 10 */
#define OB_WP_11                   ((uint32_t)0x00000800U)        /*!< erase/program protection of sector 11 */
#define OB_WP_12                   ((uint32_t)0x00001000U)        /*!< erase/program protection of sector 12 */
#define OB_WP_13                   ((uint32_t)0x00002000U)        /*!< erase/program protection of sector 13 */
#define OB_WP_14                   ((uint32_t)0x00004000U)        /*!< erase/program protection of sector 14 */
#define OB_WP_15                   ((uint32_t)0x00008000U)        /*!< erase/program protection of sector 15 */
#define OB_WP_16                   ((uint32_t)0x00010000U)        /*!< erase/program protection of sector 16 */
#define OB_WP_17                   ((uint32_t)0x00020000U)        /*!< erase/program protection of sector 17 */
#define OB_WP_18                   ((uint32_t)0x00040000U)        /*!< erase/program protection of sector 18 */
#define OB_WP_19                   ((uint32_t)0x00080000U)        /*!< erase/program protection of sector 19 */
#define OB_WP_20                   ((uint32_t)0x00100000U)        /*!< erase/program protection of sector 20 */
#define OB_WP_21                   ((uint32_t)0x00200000U)        /*!< erase/program protection of sector 21 */
#define OB_WP_22                   ((uint32_t)0x00400000U)        /*!< erase/program protection of sector 22 */
#define OB_WP_23                   ((uint32_t)0x00800000U)        /*!< erase/program protection of sector 23 */
#define OB_WP_24                   ((uint32_t)0x01000000U)        /*!< erase/program protection of sector 24 */
#define OB_WP_25                   ((uint32_t)0x02000000U)        /*!< erase/program protection of sector 25 */
#define OB_WP_26                   ((uint32_t)0x04000000U)        /*!< erase/program protection of sector 26 */
#define OB_WP_27                   ((uint32_t)0x08000000U)        /*!< erase/program protection of sector 27 */
#define OB_WP_28                   ((uint32_t)0x10000000U)        /*!< erase/program protection of sector 28 */
#define OB_WP_29                   ((uint32_t)0x20000000U)        /*!< erase/program protection of sector 29 */
#define OB_WP_30                   ((uint32_t)0x40000000U)        /*!< erase/program protection of sector 30 */
#define OB_WP_31                   ((uint32_t)0x80000000U)        /*!< erase/program protection of sector 31 */
#define OB_WP_ALL                  ((uint32_t)0xFFFFFFFFU)        /*!< erase/program protection of all sectors */

/* FMC timeout */
#define FMC_TIMEOUT_COUNT          ((uint32_t)0x000F0000U)        /*!< FMC timeout count value */

/* FMC BANK address */
#define FMC_SIZE                   (*(uint16_t *)0x1FFFF7E0U)     /*!< FMC size */
#define SRAM_SIZE                  (*(uint16_t *)0x1FFFF7E2U)     /*!< SRAM size*/


////////////////////////////////////////////////////////////////////////
/* FWDGT definitions */
#define FWDGT                       FWDGT_BASE                                /*!< FWDGT base address */

/* registers definitions */
#define FWDGT_CTL                   REG32((FWDGT) + 0x00000000U)              /*!< FWDGT control register */
#define FWDGT_PSC                   REG32((FWDGT) + 0x00000004U)              /*!< FWDGT prescaler register */
#define FWDGT_RLD                   REG32((FWDGT) + 0x00000008U)              /*!< FWDGT reload register */
#define FWDGT_STAT                  REG32((FWDGT) + 0x0000000CU)              /*!< FWDGT status register */

/* bits definitions */
/* FWDGT_CTL */
#define FWDGT_CTL_CMD               BITS(0,15)                                /*!< FWDGT command value */

/* FWDGT_PSC */
#define FWDGT_PSC_PSC               BITS(0,2)                                 /*!< FWDGT prescaler divider value */

/* FWDGT_RLD */
#define FWDGT_RLD_RLD               BITS(0,11)                                /*!< FWDGT counter reload value */

/* FWDGT_STAT */
#define FWDGT_STAT_PUD              BIT(0)                                    /*!< FWDGT prescaler divider value update */
#define FWDGT_STAT_RUD              BIT(1)                                    /*!< FWDGT counter reload value update */

/* constants definitions */
/* psc register value */
#define PSC_PSC(regval)             (BITS(0,2) & ((uint32_t)(regval) << 0))
#define FWDGT_PSC_DIV4              ((uint8_t)PSC_PSC(0))                     /*!< FWDGT prescaler set to 4 */
#define FWDGT_PSC_DIV8              ((uint8_t)PSC_PSC(1))                     /*!< FWDGT prescaler set to 8 */
#define FWDGT_PSC_DIV16             ((uint8_t)PSC_PSC(2))                     /*!< FWDGT prescaler set to 16 */
#define FWDGT_PSC_DIV32             ((uint8_t)PSC_PSC(3))                     /*!< FWDGT prescaler set to 32 */
#define FWDGT_PSC_DIV64             ((uint8_t)PSC_PSC(4))                     /*!< FWDGT prescaler set to 64 */
#define FWDGT_PSC_DIV128            ((uint8_t)PSC_PSC(5))                     /*!< FWDGT prescaler set to 128 */
#define FWDGT_PSC_DIV256            ((uint8_t)PSC_PSC(6))                     /*!< FWDGT prescaler set to 256 */

/* control value */
#define FWDGT_WRITEACCESS_ENABLE    ((uint16_t)0x5555U)                       /*!< FWDGT_CTL bits write access enable value */
#define FWDGT_WRITEACCESS_DISABLE   ((uint16_t)0x0000U)                       /*!< FWDGT_CTL bits write access disable value */
#define FWDGT_KEY_RELOAD            ((uint16_t)0xAAAAU)                       /*!< FWDGT_CTL bits fwdgt counter reload value */
#define FWDGT_KEY_ENABLE            ((uint16_t)0xCCCCU)                       /*!< FWDGT_CTL bits fwdgt counter enable value */

/* FWDGT timeout value */
#define FWDGT_PSC_TIMEOUT           ((uint32_t)0x000FFFFFU)                   /*!< FWDGT_PSC register write operation state flag timeout */
#define FWDGT_RLD_TIMEOUT           ((uint32_t)0x000FFFFFU)                   /*!< FWDGT_RLD register write operation state flag timeout */

/* FWDGT flag definitions */
#define FWDGT_FLAG_PUD              FWDGT_STAT_PUD                            /*!< FWDGT prescaler divider value update flag */
#define FWDGT_FLAG_RUD              FWDGT_STAT_RUD                            /*!< FWDGT counter reload value update flag */

/* write value to FWDGT_RLD_RLD bit field */
#define RLD_RLD(regval)             (BITS(0,11) & ((uint32_t)(regval) << 0))


////////////////////////////////////////////////////////////////////////
/* GPIOx(x=A,B,C,D,E) definitions */
#define GPIOA                      (GPIO_BASE + 0x00000000U)
#define GPIOB                      (GPIO_BASE + 0x00000400U)
#define GPIOC                      (GPIO_BASE + 0x00000800U)
#define GPIOD                      (GPIO_BASE + 0x00000C00U)
#define GPIOE                      (GPIO_BASE + 0x00001000U)

/* AFIO definitions */
#define AFIO                       AFIO_BASE

/* registers definitions */

/* GPIO registers definitions */
#define GPIO_CTL0(gpiox)           REG32((gpiox) + 0x00U)    /*!< GPIO port control register 0 */
#define GPIO_CTL1(gpiox)           REG32((gpiox) + 0x04U)    /*!< GPIO port control register 1 */
#define GPIO_ISTAT(gpiox)          REG32((gpiox) + 0x08U)    /*!< GPIO port input status register */
#define GPIO_OCTL(gpiox)           REG32((gpiox) + 0x0CU)    /*!< GPIO port output control register */
#define GPIO_BOP(gpiox)            REG32((gpiox) + 0x10U)    /*!< GPIO port bit operation register */
#define GPIO_BC(gpiox)             REG32((gpiox) + 0x14U)    /*!< GPIO bit clear register */
#define GPIO_LOCK(gpiox)           REG32((gpiox) + 0x18U)    /*!< GPIO port configuration lock register */

/* AFIO registers definitions */
#define AFIO_EC                    REG32(AFIO + 0x00U)       /*!< AFIO event control register */
#define AFIO_PCF0                  REG32(AFIO + 0x04U)       /*!< AFIO port configuration register 0 */
#define AFIO_EXTISS0               REG32(AFIO + 0x08U)       /*!< AFIO port EXTI sources selection register 0 */
#define AFIO_EXTISS1               REG32(AFIO + 0x0CU)       /*!< AFIO port EXTI sources selection register 1 */
#define AFIO_EXTISS2               REG32(AFIO + 0x10U)       /*!< AFIO port EXTI sources selection register 2 */
#define AFIO_EXTISS3               REG32(AFIO + 0x14U)       /*!< AFIO port EXTI sources selection register 3 */
#define AFIO_PCF1                  REG32(AFIO + 0x1CU)       /*!< AFIO port configuration register 1 */

/* bits definitions */
/* GPIO_CTL0 */
#define GPIO_CTL0_MD0              BITS(0, 1)                /*!< port 0 mode bits */
#define GPIO_CTL0_CTL0             BITS(2, 3)                /*!< pin 0 configuration bits */
#define GPIO_CTL0_MD1              BITS(4, 5)                /*!< port 1 mode bits */
#define GPIO_CTL0_CTL1             BITS(6, 7)                /*!< pin 1 configuration bits */
#define GPIO_CTL0_MD2              BITS(8, 9)                /*!< port 2 mode bits */
#define GPIO_CTL0_CTL2             BITS(10, 11)              /*!< pin 2 configuration bits */
#define GPIO_CTL0_MD3              BITS(12, 13)              /*!< port 3 mode bits */
#define GPIO_CTL0_CTL3             BITS(14, 15)              /*!< pin 3 configuration bits */
#define GPIO_CTL0_MD4              BITS(16, 17)              /*!< port 4 mode bits */
#define GPIO_CTL0_CTL4             BITS(18, 19)              /*!< pin 4 configuration bits */
#define GPIO_CTL0_MD5              BITS(20, 21)              /*!< port 5 mode bits */
#define GPIO_CTL0_CTL5             BITS(22, 23)              /*!< pin 5 configuration bits */
#define GPIO_CTL0_MD6              BITS(24, 25)              /*!< port 6 mode bits */
#define GPIO_CTL0_CTL6             BITS(26, 27)              /*!< pin 6 configuration bits */
#define GPIO_CTL0_MD7              BITS(28, 29)              /*!< port 7 mode bits */
#define GPIO_CTL0_CTL7             BITS(30, 31)              /*!< pin 7 configuration bits */

/* GPIO_CTL1 */
#define GPIO_CTL1_MD8              BITS(0, 1)                /*!< port 8 mode bits */
#define GPIO_CTL1_CTL8             BITS(2, 3)                /*!< pin 8 configuration bits */
#define GPIO_CTL1_MD9              BITS(4, 5)                /*!< port 9 mode bits */
#define GPIO_CTL1_CTL9             BITS(6, 7)                /*!< pin 9 configuration bits */
#define GPIO_CTL1_MD10             BITS(8, 9)                /*!< port 10 mode bits */
#define GPIO_CTL1_CTL10            BITS(10, 11)              /*!< pin 10 configuration bits */
#define GPIO_CTL1_MD11             BITS(12, 13)              /*!< port 11 mode bits */
#define GPIO_CTL1_CTL11            BITS(14, 15)              /*!< pin 11 configuration bits */
#define GPIO_CTL1_MD12             BITS(16, 17)              /*!< port 12 mode bits */
#define GPIO_CTL1_CTL12            BITS(18, 19)              /*!< pin 12 configuration bits */
#define GPIO_CTL1_MD13             BITS(20, 21)              /*!< port 13 mode bits */
#define GPIO_CTL1_CTL13            BITS(22, 23)              /*!< pin 13 configuration bits */
#define GPIO_CTL1_MD14             BITS(24, 25)              /*!< port 14 mode bits */
#define GPIO_CTL1_CTL14            BITS(26, 27)              /*!< pin 14 configuration bits */
#define GPIO_CTL1_MD15             BITS(28, 29)              /*!< port 15 mode bits */
#define GPIO_CTL1_CTL15            BITS(30, 31)              /*!< pin 15 configuration bits */

/* GPIO_ISTAT */
#define GPIO_ISTAT_ISTAT0          BIT(0)                    /*!< pin 0 input status */
#define GPIO_ISTAT_ISTAT1          BIT(1)                    /*!< pin 1 input status */
#define GPIO_ISTAT_ISTAT2          BIT(2)                    /*!< pin 2 input status */
#define GPIO_ISTAT_ISTAT3          BIT(3)                    /*!< pin 3 input status */
#define GPIO_ISTAT_ISTAT4          BIT(4)                    /*!< pin 4 input status */
#define GPIO_ISTAT_ISTAT5          BIT(5)                    /*!< pin 5 input status */
#define GPIO_ISTAT_ISTAT6          BIT(6)                    /*!< pin 6 input status */
#define GPIO_ISTAT_ISTAT7          BIT(7)                    /*!< pin 7 input status */
#define GPIO_ISTAT_ISTAT8          BIT(8)                    /*!< pin 8 input status */
#define GPIO_ISTAT_ISTAT9          BIT(9)                    /*!< pin 9 input status */
#define GPIO_ISTAT_ISTAT10         BIT(10)                   /*!< pin 10 input status */
#define GPIO_ISTAT_ISTAT11         BIT(11)                   /*!< pin 11 input status */
#define GPIO_ISTAT_ISTAT12         BIT(12)                   /*!< pin 12 input status */
#define GPIO_ISTAT_ISTAT13         BIT(13)                   /*!< pin 13 input status */
#define GPIO_ISTAT_ISTAT14         BIT(14)                   /*!< pin 14 input status */
#define GPIO_ISTAT_ISTAT15         BIT(15)                   /*!< pin 15 input status */

/* GPIO_OCTL */
#define GPIO_OCTL_OCTL0            BIT(0)                    /*!< pin 0 output bit */
#define GPIO_OCTL_OCTL1            BIT(1)                    /*!< pin 1 output bit */
#define GPIO_OCTL_OCTL2            BIT(2)                    /*!< pin 2 output bit */
#define GPIO_OCTL_OCTL3            BIT(3)                    /*!< pin 3 output bit */
#define GPIO_OCTL_OCTL4            BIT(4)                    /*!< pin 4 output bit */
#define GPIO_OCTL_OCTL5            BIT(5)                    /*!< pin 5 output bit */
#define GPIO_OCTL_OCTL6            BIT(6)                    /*!< pin 6 output bit */
#define GPIO_OCTL_OCTL7            BIT(7)                    /*!< pin 7 output bit */
#define GPIO_OCTL_OCTL8            BIT(8)                    /*!< pin 8 output bit */
#define GPIO_OCTL_OCTL9            BIT(9)                    /*!< pin 9 output bit */
#define GPIO_OCTL_OCTL10           BIT(10)                   /*!< pin 10 output bit */
#define GPIO_OCTL_OCTL11           BIT(11)                   /*!< pin 11 output bit */
#define GPIO_OCTL_OCTL12           BIT(12)                   /*!< pin 12 output bit */
#define GPIO_OCTL_OCTL13           BIT(13)                   /*!< pin 13 output bit */
#define GPIO_OCTL_OCTL14           BIT(14)                   /*!< pin 14 output bit */
#define GPIO_OCTL_OCTL15           BIT(15)                   /*!< pin 15 output bit */

/* GPIO_BOP */
#define GPIO_BOP_BOP0              BIT(0)                    /*!< pin 0 set bit */
#define GPIO_BOP_BOP1              BIT(1)                    /*!< pin 1 set bit */
#define GPIO_BOP_BOP2              BIT(2)                    /*!< pin 2 set bit */
#define GPIO_BOP_BOP3              BIT(3)                    /*!< pin 3 set bit */
#define GPIO_BOP_BOP4              BIT(4)                    /*!< pin 4 set bit */
#define GPIO_BOP_BOP5              BIT(5)                    /*!< pin 5 set bit */
#define GPIO_BOP_BOP6              BIT(6)                    /*!< pin 6 set bit */
#define GPIO_BOP_BOP7              BIT(7)                    /*!< pin 7 set bit */
#define GPIO_BOP_BOP8              BIT(8)                    /*!< pin 8 set bit */
#define GPIO_BOP_BOP9              BIT(9)                    /*!< pin 9 set bit */
#define GPIO_BOP_BOP10             BIT(10)                   /*!< pin 10 set bit */
#define GPIO_BOP_BOP11             BIT(11)                   /*!< pin 11 set bit */
#define GPIO_BOP_BOP12             BIT(12)                   /*!< pin 12 set bit */
#define GPIO_BOP_BOP13             BIT(13)                   /*!< pin 13 set bit */
#define GPIO_BOP_BOP14             BIT(14)                   /*!< pin 14 set bit */
#define GPIO_BOP_BOP15             BIT(15)                   /*!< pin 15 set bit */
#define GPIO_BOP_CR0               BIT(16)                   /*!< pin 0 clear bit */
#define GPIO_BOP_CR1               BIT(17)                   /*!< pin 1 clear bit */
#define GPIO_BOP_CR2               BIT(18)                   /*!< pin 2 clear bit */
#define GPIO_BOP_CR3               BIT(19)                   /*!< pin 3 clear bit */
#define GPIO_BOP_CR4               BIT(20)                   /*!< pin 4 clear bit */
#define GPIO_BOP_CR5               BIT(21)                   /*!< pin 5 clear bit */
#define GPIO_BOP_CR6               BIT(22)                   /*!< pin 6 clear bit */
#define GPIO_BOP_CR7               BIT(23)                   /*!< pin 7 clear bit */
#define GPIO_BOP_CR8               BIT(24)                   /*!< pin 8 clear bit */
#define GPIO_BOP_CR9               BIT(25)                   /*!< pin 9 clear bit */
#define GPIO_BOP_CR10              BIT(26)                   /*!< pin 10 clear bit */
#define GPIO_BOP_CR11              BIT(27)                   /*!< pin 11 clear bit */
#define GPIO_BOP_CR12              BIT(28)                   /*!< pin 12 clear bit */
#define GPIO_BOP_CR13              BIT(29)                   /*!< pin 13 clear bit */
#define GPIO_BOP_CR14              BIT(30)                   /*!< pin 14 clear bit */
#define GPIO_BOP_CR15              BIT(31)                   /*!< pin 15 clear bit */

/* GPIO_BC */
#define GPIO_BC_CR0                BIT(0)                    /*!< pin 0 clear bit */
#define GPIO_BC_CR1                BIT(1)                    /*!< pin 1 clear bit */
#define GPIO_BC_CR2                BIT(2)                    /*!< pin 2 clear bit */
#define GPIO_BC_CR3                BIT(3)                    /*!< pin 3 clear bit */
#define GPIO_BC_CR4                BIT(4)                    /*!< pin 4 clear bit */
#define GPIO_BC_CR5                BIT(5)                    /*!< pin 5 clear bit */
#define GPIO_BC_CR6                BIT(6)                    /*!< pin 6 clear bit */
#define GPIO_BC_CR7                BIT(7)                    /*!< pin 7 clear bit */
#define GPIO_BC_CR8                BIT(8)                    /*!< pin 8 clear bit */
#define GPIO_BC_CR9                BIT(9)                    /*!< pin 9 clear bit */
#define GPIO_BC_CR10               BIT(10)                   /*!< pin 10 clear bit */
#define GPIO_BC_CR11               BIT(11)                   /*!< pin 11 clear bit */
#define GPIO_BC_CR12               BIT(12)                   /*!< pin 12 clear bit */
#define GPIO_BC_CR13               BIT(13)                   /*!< pin 13 clear bit */
#define GPIO_BC_CR14               BIT(14)                   /*!< pin 14 clear bit */
#define GPIO_BC_CR15               BIT(15)                   /*!< pin 15 clear bit */

/* GPIO_LOCK */
#define GPIO_LOCK_LK0              BIT(0)                    /*!< pin 0 lock bit */
#define GPIO_LOCK_LK1              BIT(1)                    /*!< pin 1 lock bit */
#define GPIO_LOCK_LK2              BIT(2)                    /*!< pin 2 lock bit */
#define GPIO_LOCK_LK3              BIT(3)                    /*!< pin 3 lock bit */
#define GPIO_LOCK_LK4              BIT(4)                    /*!< pin 4 lock bit */
#define GPIO_LOCK_LK5              BIT(5)                    /*!< pin 5 lock bit */
#define GPIO_LOCK_LK6              BIT(6)                    /*!< pin 6 lock bit */
#define GPIO_LOCK_LK7              BIT(7)                    /*!< pin 7 lock bit */
#define GPIO_LOCK_LK8              BIT(8)                    /*!< pin 8 lock bit */
#define GPIO_LOCK_LK9              BIT(9)                    /*!< pin 9 lock bit */
#define GPIO_LOCK_LK10             BIT(10)                   /*!< pin 10 lock bit */
#define GPIO_LOCK_LK11             BIT(11)                   /*!< pin 11 lock bit */
#define GPIO_LOCK_LK12             BIT(12)                   /*!< pin 12 lock bit */
#define GPIO_LOCK_LK13             BIT(13)                   /*!< pin 13 lock bit */
#define GPIO_LOCK_LK14             BIT(14)                   /*!< pin 14 lock bit */
#define GPIO_LOCK_LK15             BIT(15)                   /*!< pin 15 lock bit */
#define GPIO_LOCK_LKK              BIT(16)                   /*!< pin sequence lock key */

/* AFIO_EC */
#define AFIO_EC_PIN                BITS(0, 3)                /*!< event output pin selection */
#define AFIO_EC_PORT               BITS(4, 6)                /*!< event output port selection */
#define AFIO_EC_EOE                BIT(7)                    /*!< event output enable */

/* AFIO_PCF0 */
#define AFIO_PCF0_SPI0_REMAP             BIT(0)              /*!< SPI0 remapping */
#define AFIO_PCF0_I2C0_REMAP             BIT(1)              /*!< I2C0 remapping */
#define AFIO_PCF0_USART0_REMAP           BIT(2)              /*!< USART0 remapping */
#define AFIO_PCF0_USART1_REMAP           BIT(3)              /*!< USART1 remapping */
#define AFIO_PCF0_USART2_REMAP           BITS(4, 5)          /*!< USART2 remapping */
#define AFIO_PCF0_TIMER0_REMAP           BITS(6, 7)          /*!< TIMER0 remapping */
#define AFIO_PCF0_TIMER1_REMAP           BITS(8, 9)          /*!< TIMER1 remapping */
#define AFIO_PCF0_TIMER2_REMAP           BITS(10, 11)        /*!< TIMER2 remapping */
#define AFIO_PCF0_TIMER3_REMAP           BIT(12)             /*!< TIMER3 remapping */
#define AFIO_PCF0_CAN_REMAP              BITS(13, 14)        /*!< CAN remapping */
#define AFIO_PCF0_PD01_REMAP             BIT(15)             /*!< port D0/port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_PCF0_TIMER4CH3_IREMAP       BIT(16)             /*!< TIMER3 channel3 internal remapping */
#define AFIO_PCF0_SWJ_CFG                BITS(24, 26)        /*!< serial wire JTAG configuration */
#define AFIO_PCF0_SPI2_REMAP             BIT(28)             /*!< SPI2/I2S2 remapping */
#define AFIO_PCF0_TIMER1_ITI1_REMAP      BIT(29)             /*!< TIMER1 internal trigger 1 remapping */

/* AFIO_EXTISS0 */
#define AFIO_EXTI0_SS                    BITS(0, 3)          /*!< EXTI 0 sources selection */
#define AFIO_EXTI1_SS                    BITS(4, 7)          /*!< EXTI 1 sources selection */
#define AFIO_EXTI2_SS                    BITS(8, 11)         /*!< EXTI 2 sources selection */
#define AFIO_EXTI3_SS                    BITS(12, 15)        /*!< EXTI 3 sources selection */

/* AFIO_EXTISS1 */
#define AFIO_EXTI4_SS                    BITS(0, 3)          /*!< EXTI 4 sources selection */
#define AFIO_EXTI5_SS                    BITS(4, 7)          /*!< EXTI 5 sources selection */
#define AFIO_EXTI6_SS                    BITS(8, 11)         /*!< EXTI 6 sources selection */
#define AFIO_EXTI7_SS                    BITS(12, 15)        /*!< EXTI 7 sources selection */

/* AFIO_EXTISS2 */
#define AFIO_EXTI8_SS                    BITS(0, 3)          /*!< EXTI 8 sources selection */
#define AFIO_EXTI9_SS                    BITS(4, 7)          /*!< EXTI 9 sources selection */
#define AFIO_EXTI10_SS                   BITS(8, 11)         /*!< EXTI 10 sources selection */
#define AFIO_EXTI11_SS                   BITS(12, 15)        /*!< EXTI 11 sources selection */

/* AFIO_EXTISS3 */
#define AFIO_EXTI12_SS                   BITS(0, 3)          /*!< EXTI 12 sources selection */
#define AFIO_EXTI13_SS                   BITS(4, 7)          /*!< EXTI 13 sources selection */
#define AFIO_EXTI14_SS                   BITS(8, 11)         /*!< EXTI 14 sources selection */
#define AFIO_EXTI15_SS                   BITS(12, 15)        /*!< EXTI 15 sources selection */

/* AFIO_PCF1 */
#define AFIO_PCF1_EXMC_NADV              BIT(10)             /*!< EXMC_NADV connect/disconnect */

/* constants definitions */
typedef FlagStatus bit_status;

/* GPIO mode values set */
#define GPIO_MODE_SET(n, mode)           ((uint32_t)((uint32_t)(mode) << (4U * (n))))
#define GPIO_MODE_MASK(n)                (0xFU << (4U * (n)))

/* GPIO mode definitions */
#define GPIO_MODE_AIN                    ((uint8_t)0x00U)          /*!< analog input mode */
#define GPIO_MODE_IN_FLOATING            ((uint8_t)0x04U)          /*!< floating input mode */
#define GPIO_MODE_IPD                    ((uint8_t)0x28U)          /*!< pull-down input mode */
#define GPIO_MODE_IPU                    ((uint8_t)0x48U)          /*!< pull-up input mode */
#define GPIO_MODE_OUT_OD                 ((uint8_t)0x14U)          /*!< GPIO output with open-drain */
#define GPIO_MODE_OUT_PP                 ((uint8_t)0x10U)          /*!< GPIO output with push-pull */
#define GPIO_MODE_AF_OD                  ((uint8_t)0x1CU)          /*!< AFIO output with open-drain */
#define GPIO_MODE_AF_PP                  ((uint8_t)0x18U)          /*!< AFIO output with push-pull */

/* GPIO output max speed value */
#define GPIO_OSPEED_10MHZ                ((uint8_t)0x01U)          /*!< output max speed 10MHz */
#define GPIO_OSPEED_2MHZ                 ((uint8_t)0x02U)          /*!< output max speed 2MHz */
#define GPIO_OSPEED_50MHZ                ((uint8_t)0x03U)          /*!< output max speed 50MHz */

/* GPIO event output port definitions */
#define GPIO_EVENT_PORT_GPIOA            ((uint8_t)0x00U)          /*!< event output port A */
#define GPIO_EVENT_PORT_GPIOB            ((uint8_t)0x01U)          /*!< event output port B */
#define GPIO_EVENT_PORT_GPIOC            ((uint8_t)0x02U)          /*!< event output port C */
#define GPIO_EVENT_PORT_GPIOD            ((uint8_t)0x03U)          /*!< event output port D */
#define GPIO_EVENT_PORT_GPIOE            ((uint8_t)0x04U)          /*!< event output port E */

/* GPIO output port source definitions */
#define GPIO_PORT_SOURCE_GPIOA           ((uint8_t)0x00U)          /*!< output port source A */
#define GPIO_PORT_SOURCE_GPIOB           ((uint8_t)0x01U)          /*!< output port source B */
#define GPIO_PORT_SOURCE_GPIOC           ((uint8_t)0x02U)          /*!< output port source C */
#define GPIO_PORT_SOURCE_GPIOD           ((uint8_t)0x03U)          /*!< output port source D */
#define GPIO_PORT_SOURCE_GPIOE           ((uint8_t)0x04U)          /*!< output port source E */

/* GPIO event output pin definitions */
#define GPIO_EVENT_PIN_0                 ((uint8_t)0x00U)          /*!< GPIO event pin 0 */
#define GPIO_EVENT_PIN_1                 ((uint8_t)0x01U)          /*!< GPIO event pin 1 */
#define GPIO_EVENT_PIN_2                 ((uint8_t)0x02U)          /*!< GPIO event pin 2 */
#define GPIO_EVENT_PIN_3                 ((uint8_t)0x03U)          /*!< GPIO event pin 3 */
#define GPIO_EVENT_PIN_4                 ((uint8_t)0x04U)          /*!< GPIO event pin 4 */
#define GPIO_EVENT_PIN_5                 ((uint8_t)0x05U)          /*!< GPIO event pin 5 */
#define GPIO_EVENT_PIN_6                 ((uint8_t)0x06U)          /*!< GPIO event pin 6 */
#define GPIO_EVENT_PIN_7                 ((uint8_t)0x07U)          /*!< GPIO event pin 7 */
#define GPIO_EVENT_PIN_8                 ((uint8_t)0x08U)          /*!< GPIO event pin 8 */
#define GPIO_EVENT_PIN_9                 ((uint8_t)0x09U)          /*!< GPIO event pin 9 */
#define GPIO_EVENT_PIN_10                ((uint8_t)0x0AU)          /*!< GPIO event pin 10 */
#define GPIO_EVENT_PIN_11                ((uint8_t)0x0BU)          /*!< GPIO event pin 11 */
#define GPIO_EVENT_PIN_12                ((uint8_t)0x0CU)          /*!< GPIO event pin 12 */
#define GPIO_EVENT_PIN_13                ((uint8_t)0x0DU)          /*!< GPIO event pin 13 */
#define GPIO_EVENT_PIN_14                ((uint8_t)0x0EU)          /*!< GPIO event pin 14 */
#define GPIO_EVENT_PIN_15                ((uint8_t)0x0FU)          /*!< GPIO event pin 15 */

/* GPIO output pin source definitions */
#define GPIO_PIN_SOURCE_0                ((uint8_t)0x00U)          /*!< GPIO pin source 0 */
#define GPIO_PIN_SOURCE_1                ((uint8_t)0x01U)          /*!< GPIO pin source 1 */
#define GPIO_PIN_SOURCE_2                ((uint8_t)0x02U)          /*!< GPIO pin source 2 */
#define GPIO_PIN_SOURCE_3                ((uint8_t)0x03U)          /*!< GPIO pin source 3 */
#define GPIO_PIN_SOURCE_4                ((uint8_t)0x04U)          /*!< GPIO pin source 4 */
#define GPIO_PIN_SOURCE_5                ((uint8_t)0x05U)          /*!< GPIO pin source 5 */
#define GPIO_PIN_SOURCE_6                ((uint8_t)0x06U)          /*!< GPIO pin source 6 */
#define GPIO_PIN_SOURCE_7                ((uint8_t)0x07U)          /*!< GPIO pin source 7 */
#define GPIO_PIN_SOURCE_8                ((uint8_t)0x08U)          /*!< GPIO pin source 8 */
#define GPIO_PIN_SOURCE_9                ((uint8_t)0x09U)          /*!< GPIO pin source 9 */
#define GPIO_PIN_SOURCE_10               ((uint8_t)0x0AU)          /*!< GPIO pin source 10 */
#define GPIO_PIN_SOURCE_11               ((uint8_t)0x0BU)          /*!< GPIO pin source 11 */
#define GPIO_PIN_SOURCE_12               ((uint8_t)0x0CU)          /*!< GPIO pin source 12 */
#define GPIO_PIN_SOURCE_13               ((uint8_t)0x0DU)          /*!< GPIO pin source 13 */
#define GPIO_PIN_SOURCE_14               ((uint8_t)0x0EU)          /*!< GPIO pin source 14 */
#define GPIO_PIN_SOURCE_15               ((uint8_t)0x0FU)          /*!< GPIO pin source 15 */

/* GPIO pin definitions */
#define GPIO_PIN_0                       BIT(0)                    /*!< GPIO pin 0 */
#define GPIO_PIN_1                       BIT(1)                    /*!< GPIO pin 1 */
#define GPIO_PIN_2                       BIT(2)                    /*!< GPIO pin 2 */
#define GPIO_PIN_3                       BIT(3)                    /*!< GPIO pin 3 */
#define GPIO_PIN_4                       BIT(4)                    /*!< GPIO pin 4 */
#define GPIO_PIN_5                       BIT(5)                    /*!< GPIO pin 5 */
#define GPIO_PIN_6                       BIT(6)                    /*!< GPIO pin 6 */
#define GPIO_PIN_7                       BIT(7)                    /*!< GPIO pin 7 */
#define GPIO_PIN_8                       BIT(8)                    /*!< GPIO pin 8 */
#define GPIO_PIN_9                       BIT(9)                    /*!< GPIO pin 9 */
#define GPIO_PIN_10                      BIT(10)                   /*!< GPIO pin 10 */
#define GPIO_PIN_11                      BIT(11)                   /*!< GPIO pin 11 */
#define GPIO_PIN_12                      BIT(12)                   /*!< GPIO pin 12 */
#define GPIO_PIN_13                      BIT(13)                   /*!< GPIO pin 13 */
#define GPIO_PIN_14                      BIT(14)                   /*!< GPIO pin 14 */
#define GPIO_PIN_15                      BIT(15)                   /*!< GPIO pin 15 */
#define GPIO_PIN_ALL                     BITS(0, 15)               /*!< GPIO pin all */

/* GPIO remap definitions */
#define GPIO_SPI0_REMAP                  ((uint32_t)0x00000001U)   /*!< SPI0 remapping */
#define GPIO_I2C0_REMAP                  ((uint32_t)0x00000002U)   /*!< I2C0 remapping */
#define GPIO_USART0_REMAP                ((uint32_t)0x00000004U)   /*!< USART0 remapping */
#define GPIO_USART1_REMAP                ((uint32_t)0x00000008U)   /*!< USART1 remapping */
#define GPIO_USART2_PARTIAL_REMAP        ((uint32_t)0x00140010U)   /*!< USART2 partial remapping */
#define GPIO_USART2_FULL_REMAP           ((uint32_t)0x00140030U)   /*!< USART2 full remapping */
#define GPIO_TIMER0_PARTIAL_REMAP        ((uint32_t)0x00160040U)   /*!< TIMER0 partial remapping */
#define GPIO_TIMER0_FULL_REMAP           ((uint32_t)0x001600C0U)   /*!< TIMER0 full remapping */
#define GPIO_TIMER1_PARTIAL_REMAP0       ((uint32_t)0x00180100U)   /*!< TIMER1 partial remapping */
#define GPIO_TIMER1_PARTIAL_REMAP1       ((uint32_t)0x00180200U)   /*!< TIMER1 partial remapping */
#define GPIO_TIMER1_FULL_REMAP           ((uint32_t)0x00180300U)   /*!< TIMER1 full remapping */
#define GPIO_TIMER2_PARTIAL_REMAP        ((uint32_t)0x001A0800U)   /*!< TIMER2 partial remapping */
#define GPIO_TIMER2_FULL_REMAP           ((uint32_t)0x001A0C00U)   /*!< TIMER2 full remapping */
#define GPIO_TIMER3_REMAP                ((uint32_t)0x00001000U)   /*!< TIMER3 remapping */
#define GPIO_CAN0_PARTIAL_REMAP          ((uint32_t)0x001D4000U)   /*!< CAN0 partial remapping */
#define GPIO_CAN0_FULL_REMAP             ((uint32_t)0x001D6000U)   /*!< CAN0 full remapping */
#define GPIO_PD01_REMAP                  ((uint32_t)0x00008000U)   /*!< PD01 remapping */
#define GPIO_TIMER4CH3_IREMAP            ((uint32_t)0x00200001U)   /*!< TIMER4 channel3 internal remapping */
#define GPIO_CAN1_REMAP                  ((uint32_t)0x00200040U)   /*!< CAN1 remapping */
#define GPIO_SWJ_NONJTRST_REMAP          ((uint32_t)0x00300100U)   /*!< JTAG-DP,but without NJTRST */
#define GPIO_SWJ_DISABLE_REMAP           ((uint32_t)0x00300200U)   /*!< JTAG-DP disabled */
#define GPIO_SPI2_REMAP                  ((uint32_t)0x00201100U)   /*!< SPI2 remapping */
#define GPIO_TIMER1ITI1_REMAP            ((uint32_t)0x00202000U)   /*!< TIMER1 internal trigger 1 remapping */
#define GPIO_EXMC_NADV_REMAP             ((uint32_t)0x80000400U)   /*!< EXMC_NADV connect/disconnect */


////////////////////////////////////////////////////////////////////////
/* I2Cx(x=0,1) definitions */
#define I2C0                          I2C_BASE                   /*!< I2C0 base address */
#define I2C1                          (I2C_BASE + 0x00000400U)   /*!< I2C1 base address */

/* registers definitions */
#define I2C_CTL0(i2cx)                REG32((i2cx) + 0x00000000U)      /*!< I2C control register 0 */
#define I2C_CTL1(i2cx)                REG32((i2cx) + 0x00000004U)      /*!< I2C control register 1 */
#define I2C_SADDR0(i2cx)              REG32((i2cx) + 0x00000008U)      /*!< I2C slave address register 0*/
#define I2C_SADDR1(i2cx)              REG32((i2cx) + 0x0000000CU)      /*!< I2C slave address register */
#define I2C_DATA(i2cx)                REG32((i2cx) + 0x00000010U)      /*!< I2C transfer buffer register */
#define I2C_STAT0(i2cx)               REG32((i2cx) + 0x00000014U)      /*!< I2C transfer status register 0 */
#define I2C_STAT1(i2cx)               REG32((i2cx) + 0x00000018U)      /*!< I2C transfer status register */
#define I2C_CKCFG(i2cx)               REG32((i2cx) + 0x0000001CU)      /*!< I2C clock configure register */
#define I2C_RT(i2cx)                  REG32((i2cx) + 0x00000020U)      /*!< I2C rise time register */
#define I2C_FMPCFG(i2cx)              REG32((i2cx) + 0x00000090U)      /*!< I2C fast-mode-plus configure register */
/* bits definitions */
/* I2Cx_CTL0 */
#define I2C_CTL0_I2CEN                BIT(0)        /*!< peripheral enable */
#define I2C_CTL0_SMBEN                BIT(1)        /*!< SMBus mode */
#define I2C_CTL0_SMBSEL               BIT(3)        /*!< SMBus type */
#define I2C_CTL0_ARPEN                BIT(4)        /*!< ARP enable */
#define I2C_CTL0_PECEN                BIT(5)        /*!< PEC enable */
#define I2C_CTL0_GCEN                 BIT(6)        /*!< general call enable */
#define I2C_CTL0_SS                   BIT(7)        /*!< clock stretching disable (slave mode) */
#define I2C_CTL0_START                BIT(8)        /*!< start generation */
#define I2C_CTL0_STOP                 BIT(9)        /*!< stop generation */
#define I2C_CTL0_ACKEN                BIT(10)       /*!< acknowledge enable */
#define I2C_CTL0_POAP                 BIT(11)       /*!< acknowledge/PEC position (for data reception) */
#define I2C_CTL0_PECTRANS             BIT(12)       /*!< packet error checking */
#define I2C_CTL0_SALT                 BIT(13)       /*!< SMBus alert */
#define I2C_CTL0_SRESET               BIT(15)       /*!< software reset */

/* I2Cx_CTL1 */
#define I2C_CTL1_I2CCLK               BITS(0,5)     /*!< I2CCLK[5:0] bits (peripheral clock frequency) */
#define I2C_CTL1_ERRIE                BIT(8)        /*!< error interrupt enable */
#define I2C_CTL1_EVIE                 BIT(9)        /*!< event interrupt enable */
#define I2C_CTL1_BUFIE                BIT(10)       /*!< buffer interrupt enable */
#define I2C_CTL1_DMAON                BIT(11)       /*!< DMA requests enable */
#define I2C_CTL1_DMALST               BIT(12)       /*!< DMA last transfer */

/* I2Cx_SADDR0 */
#define I2C_SADDR0_ADDRESS0           BIT(0)        /*!< bit 0 of a 10-bit address */
#define I2C_SADDR0_ADDRESS            BITS(1,7)     /*!< 7-bit address or bits 7:1 of a 10-bit address */
#define I2C_SADDR0_ADDRESS_H          BITS(8,9)     /*!< highest two bits of a 10-bit address */
#define I2C_SADDR0_ADDFORMAT          BIT(15)       /*!< address mode for the I2C slave */

/* I2Cx_SADDR1 */
#define I2C_SADDR1_DUADEN             BIT(0)        /*!< aual-address mode switch */
#define I2C_SADDR1_ADDRESS2           BITS(1,7)     /*!< second I2C address for the slave in dual-address mode */

/* I2Cx_DATA */
#define I2C_DATA_TRB                  BITS(0,7)     /*!< 8-bit data register */

/* I2Cx_STAT0 */
#define I2C_STAT0_SBSEND              BIT(0)        /*!< start bit (master mode) */
#define I2C_STAT0_ADDSEND             BIT(1)        /*!< address sent (master mode)/matched (slave mode) */
#define I2C_STAT0_BTC                 BIT(2)        /*!< byte transfer finished */
#define I2C_STAT0_ADD10SEND           BIT(3)        /*!< 10-bit header sent (master mode) */
#define I2C_STAT0_STPDET              BIT(4)        /*!< stop detection (slave mode) */
#define I2C_STAT0_RBNE                BIT(6)        /*!< data register not empty (receivers) */
#define I2C_STAT0_TBE                 BIT(7)        /*!< data register empty (transmitters) */
#define I2C_STAT0_BERR                BIT(8)        /*!< bus error */
#define I2C_STAT0_LOSTARB             BIT(9)        /*!< arbitration lost (master mode) */
#define I2C_STAT0_AERR                BIT(10)       /*!< acknowledge failure */
#define I2C_STAT0_OUERR               BIT(11)       /*!< overrun/underrun */
#define I2C_STAT0_PECERR              BIT(12)       /*!< PEC error in reception */
#define I2C_STAT0_SMBTO               BIT(14)       /*!< timeout signal in SMBus mode */
#define I2C_STAT0_SMBALT              BIT(15)       /*!< SMBus alert status */

/* I2Cx_STAT1 */
#define I2C_STAT1_MASTER              BIT(0)        /*!< master/slave */
#define I2C_STAT1_I2CBSY              BIT(1)        /*!< bus busy */
#define I2C_STAT1_TR                  BIT(2)        /*!< transmitter/receiver */
#define I2C_STAT1_RXGC                BIT(4)        /*!< general call address (slave mode) */
#define I2C_STAT1_DEFSMB              BIT(5)        /*!< SMBus device default address (slave mode) */
#define I2C_STAT1_HSTSMB              BIT(6)        /*!< SMBus host header (slave mode) */
#define I2C_STAT1_DUMODF              BIT(7)        /*!< dual flag (slave mode) */
#define I2C_STAT1_PECV                BITS(8,15)    /*!< packet error checking value */

/* I2Cx_CKCFG */
#define I2C_CKCFG_CLKC                BITS(0,11)    /*!< clock control register in fast/standard mode (master mode) */
#define I2C_CKCFG_DTCY                BIT(14)       /*!< fast mode duty cycle */
#define I2C_CKCFG_FAST                BIT(15)       /*!< I2C speed selection in master mode */

/* I2Cx_RT */
#define I2C_RT_RISETIME               BITS(0,5)     /*!< maximum rise time in fast/standard mode (Master mode) */

/* I2Cx_FMPCFG */
#define I2C_FMPCFG_FMPEN              BIT(0)        /*!< fast mode plus enable bit */

/* constants definitions */
/* define the I2C bit position and its register index offset */
#define I2C_REGIDX_BIT(regidx, bitpos)  (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define I2C_REG_VAL(i2cx, offset)       (REG32((i2cx) + (((uint32_t)(offset) & 0xFFFFU) >> 6)))
#define I2C_BIT_POS(val)                ((uint32_t)(val) & 0x1FU)
#define I2C_REGIDX_BIT2(regidx, bitpos, regidx2, bitpos2)   (((uint32_t)(regidx2) << 22) | (uint32_t)((bitpos2) << 16)\
                                                              | (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos)))
#define I2C_REG_VAL2(i2cx, offset)      (REG32((i2cx) + ((uint32_t)(offset) >> 22)))
#define I2C_BIT_POS2(val)               (((uint32_t)(val) & 0x1F0000U) >> 16)

/* register offset */
#define I2C_CTL1_REG_OFFSET           0x04U         /*!< CTL1 register offset */
#define I2C_STAT0_REG_OFFSET          0x14U         /*!< STAT0 register offset */
#define I2C_STAT1_REG_OFFSET          0x18U         /*!< STAT1 register offset */

/* I2C flags */
typedef enum {
    /* flags in STAT0 register */
    I2C_FLAG_SBSEND = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 0U), /*!< start condition sent out in master mode */
    I2C_FLAG_ADDSEND = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 1U), /*!< address is sent in master mode or received and matches in slave mode */
    I2C_FLAG_BTC = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 2U), /*!< byte transmission finishes */
    I2C_FLAG_ADD10SEND = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 3U), /*!< header of 10-bit address is sent in master mode */
    I2C_FLAG_STPDET = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 4U), /*!< stop condition detected in slave mode */
    I2C_FLAG_RBNE = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 6U), /*!< I2C_DATA is not Empty during receiving */
    I2C_FLAG_TBE = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 7U), /*!< I2C_DATA is empty during transmitting */
    I2C_FLAG_BERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 8U), /*!< a bus error occurs indication a unexpected start or stop condition on I2C bus */
    I2C_FLAG_LOSTARB = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 9U), /*!< arbitration lost in master mode */
    I2C_FLAG_AERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 10U), /*!< acknowledge error */
    I2C_FLAG_OUERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 11U), /*!< over-run or under-run situation occurs in slave mode */
    I2C_FLAG_PECERR = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 12U), /*!< PEC error when receiving data */
    I2C_FLAG_SMBTO = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 14U), /*!< timeout signal in SMBus mode */
    I2C_FLAG_SMBALT = I2C_REGIDX_BIT(I2C_STAT0_REG_OFFSET, 15U), /*!< SMBus alert status */
    /* flags in STAT1 register */
    I2C_FLAG_MASTER = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 0U), /*!< a flag indicating whether I2C block is in master or slave mode */
    I2C_FLAG_I2CBSY = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 1U), /*!< busy flag */
    I2C_FLAG_TR = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 2U), /*!< whether the I2C is a transmitter or a receiver */
    I2C_FLAG_RXGC = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 4U), /*!< general call address (00h) received */
    I2C_FLAG_DEFSMB = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 5U), /*!< default address of SMBus device */
    I2C_FLAG_HSTSMB = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 6U), /*!< SMBus host header detected in slave mode */
    I2C_FLAG_DUMODF = I2C_REGIDX_BIT(I2C_STAT1_REG_OFFSET, 7U), /*!< dual flag in slave mode indicating which address is matched in dual-address mode */
} i2c_flag_enum;

/* I2C interrupt flags */
typedef enum {
    /* interrupt flags in CTL1 register */
    I2C_INT_FLAG_SBSEND = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 0U), /*!< start condition sent out in master mode interrupt flag */
    I2C_INT_FLAG_ADDSEND = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 1U), /*!< address is sent in master mode or received and matches in slave mode interrupt flag */
    I2C_INT_FLAG_BTC = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 2U), /*!< byte transmission finishes */
    I2C_INT_FLAG_ADD10SEND = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 3U), /*!< header of 10-bit address is sent in master mode interrupt flag */
    I2C_INT_FLAG_STPDET = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 4U), /*!< stop condition detected in slave mode interrupt flag */
    I2C_INT_FLAG_RBNE = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 6U), /*!< I2C_DATA is not Empty during receiving interrupt flag */
    I2C_INT_FLAG_TBE = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 9U, I2C_STAT0_REG_OFFSET, 7U), /*!< I2C_DATA is empty during transmitting interrupt flag */
    I2C_INT_FLAG_BERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 8U), /*!< a bus error occurs indication a unexpected start or stop condition on I2C bus interrupt flag */
    I2C_INT_FLAG_LOSTARB = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 9U), /*!< arbitration lost in master mode interrupt flag */
    I2C_INT_FLAG_AERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 10U), /*!< acknowledge error interrupt flag */
    I2C_INT_FLAG_OUERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 11U), /*!< over-run or under-run situation occurs in slave mode interrupt flag */
    I2C_INT_FLAG_PECERR = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 12U), /*!< PEC error when receiving data interrupt flag */
    I2C_INT_FLAG_SMBTO = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 14U), /*!< timeout signal in SMBus mode interrupt flag */
    I2C_INT_FLAG_SMBALT = I2C_REGIDX_BIT2(I2C_CTL1_REG_OFFSET, 8U, I2C_STAT0_REG_OFFSET, 15U), /*!< SMBus Alert status interrupt flag */
} i2c_interrupt_flag_enum;

/* I2C interrupt enable or disable */
typedef enum {
    /* interrupt in CTL1 register */
    I2C_INT_ERR = I2C_REGIDX_BIT(I2C_CTL1_REG_OFFSET, 8U), /*!< error interrupt enable */
    I2C_INT_EV = I2C_REGIDX_BIT(I2C_CTL1_REG_OFFSET, 9U), /*!< event interrupt enable */
    I2C_INT_BUF = I2C_REGIDX_BIT(I2C_CTL1_REG_OFFSET, 10U), /*!< buffer interrupt enable */
} i2c_interrupt_enum;

/* SMBus/I2C mode switch and SMBus type selection */
#define I2C_I2CMODE_ENABLE            ((uint32_t)0x00000000U)                  /*!< I2C mode */
#define I2C_SMBUSMODE_ENABLE          I2C_CTL0_SMBEN                           /*!< SMBus mode */

/* SMBus/I2C mode switch and SMBus type selection */
#define I2C_SMBUS_DEVICE              ((uint32_t)0x00000000U)                  /*!< SMBus mode device type */
#define I2C_SMBUS_HOST                I2C_CTL0_SMBSEL                          /*!< SMBus mode host type */

/* I2C transfer direction */
#define I2C_RECEIVER                  ((uint32_t)0x00000001U)                  /*!< receiver */
#define I2C_TRANSMITTER               ((uint32_t)0xFFFFFFFEU)                  /*!< transmitter */

/* whether or not to send an ACK */
#define I2C_ACK_DISABLE               ((uint32_t)0x00000000U)                  /*!< ACK will be not sent */
#define I2C_ACK_ENABLE                I2C_CTL0_ACKEN                           /*!< ACK will be sent */

/* I2C POAP position*/
#define I2C_ACKPOS_CURRENT            ((uint32_t)0x00000000U)                  /*!< ACKEN bit decides whether or not to send ACK or not for the current byte */
#define I2C_ACKPOS_NEXT               I2C_CTL0_POAP                            /*!< ACKEN bit decides whether or not to send ACK for the next byte */

/* I2C dual-address mode switch */
#define I2C_DUADEN_DISABLE            ((uint32_t)0x00000000U)                  /*!< dual-address mode disabled */
#define I2C_DUADEN_ENABLE             ((uint32_t)0x00000001U)                  /*!< dual-address mode enabled */

/* whether or not to stretch SCL low */
#define I2C_SCLSTRETCH_ENABLE         ((uint32_t)0x00000000U)                  /*!< SCL stretching is enabled */
#define I2C_SCLSTRETCH_DISABLE        I2C_CTL0_SS                              /*!< SCL stretching is disabled */

/* whether or not to response to a general call */
#define I2C_GCEN_ENABLE               I2C_CTL0_GCEN                            /*!< slave will response to a general call */
#define I2C_GCEN_DISABLE              ((uint32_t)0x00000000U)                  /*!< slave will not response to a general call */

/* software reset I2C */
#define I2C_SRESET_SET                I2C_CTL0_SRESET                          /*!< I2C is under reset */
#define I2C_SRESET_RESET              ((uint32_t)0x00000000U)                  /*!< I2C is not under reset */

/* I2C DMA mode configure */
/* DMA mode switch */
#define I2C_DMA_ON                    I2C_CTL1_DMAON                           /*!< DMA mode enabled */
#define I2C_DMA_OFF                   ((uint32_t)0x00000000U)                  /*!< DMA mode disabled */

/* flag indicating DMA last transfer */
#define I2C_DMALST_ON                 I2C_CTL1_DMALST                          /*!< next DMA EOT is the last transfer */
#define I2C_DMALST_OFF                ((uint32_t)0x00000000U)                  /*!< next DMA EOT is not the last transfer */

/* I2C PEC configure */
/* PEC enable */
#define I2C_PEC_ENABLE                I2C_CTL0_PECEN                           /*!< PEC calculation on */
#define I2C_PEC_DISABLE               ((uint32_t)0x00000000U)                   /*!< PEC calculation off */

/* PEC transfer */
#define I2C_PECTRANS_ENABLE           I2C_CTL0_PECTRANS                        /*!< transfer PEC */
#define I2C_PECTRANS_DISABLE          ((uint32_t)0x00000000U)                  /*!< not transfer PEC value */

/* I2C SMBus configure */
/* issue or not alert through SMBA pin */
#define I2C_SALTSEND_ENABLE           I2C_CTL0_SALT                            /*!< issue alert through SMBA pin */
#define I2C_SALTSEND_DISABLE          ((uint32_t)0x00000000U)                  /*!< not issue alert through SMBA */

/* ARP protocol in SMBus switch */
#define I2C_ARP_ENABLE                I2C_CTL0_ARPEN                           /*!< ARP enable */
#define I2C_ARP_DISABLE               ((uint32_t)0x00000000U)                  /*!< ARP disable */

/* transmit I2C data */
#define DATA_TRANS(regval)            (BITS(0,7) & ((uint32_t)(regval) << 0))

/* receive I2C data */
#define DATA_RECV(regval)             GET_BITS((uint32_t)(regval), 0, 7)

/* I2C duty cycle in fast mode */
#define I2C_DTCY_2                    ((uint32_t)0x00000000U)                  /*!< I2C fast mode Tlow/Thigh = 2 */
#define I2C_DTCY_16_9                 I2C_CKCFG_DTCY                           /*!< I2C fast mode Tlow/Thigh = 16/9 */

/* address mode for the I2C slave */
#define I2C_ADDFORMAT_7BITS           ((uint32_t)0x00000000U)                  /*!< address:7 bits */
#define I2C_ADDFORMAT_10BITS          I2C_SADDR0_ADDFORMAT                     /*!< address:10 bits */


////////////////////////////////////////////////////////////////////////
/* PMU definitions */
#define PMU                           PMU_BASE                 /*!< PMU base address */

/* registers definitions */
#define PMU_CTL                       REG32((PMU) + 0x00U)     /*!< PMU control register */
#define PMU_CS                        REG32((PMU) + 0x04U)     /*!< PMU control and status register */

/* bits definitions */
/* PMU_CTL */
#define PMU_CTL_LDOLP                 BIT(0)                   /*!< LDO low power mode */
#define PMU_CTL_STBMOD                BIT(1)                   /*!< standby mode */
#define PMU_CTL_WURST                 BIT(2)                   /*!< wakeup flag reset */
#define PMU_CTL_STBRST                BIT(3)                   /*!< standby flag reset */
#define PMU_CTL_LVDEN                 BIT(4)                   /*!< low voltage detector enable */
#define PMU_CTL_LVDT                  BITS(5,7)                /*!< low voltage detector threshold */
#define PMU_CTL_BKPWEN                BIT(8)                   /*!< backup domain write enable */

/* PMU_CS */
#define PMU_CS_WUF                    BIT(0)                   /*!< wakeup flag */
#define PMU_CS_STBF                   BIT(1)                   /*!< standby flag */
#define PMU_CS_LVDF                   BIT(2)                   /*!< low voltage detector status flag */
#define PMU_CS_WUPEN                  BIT(8)                   /*!< wakeup pin enable */

/* constants definitions */
/* PMU low voltage detector threshold definitions */
#define CTL_LVDT(regval)              (BITS(5,7)&((uint32_t)(regval) << 5))
#define PMU_LVDT_0                    CTL_LVDT(0)              /*!< voltage threshold is 2.2V */
#define PMU_LVDT_1                    CTL_LVDT(1)              /*!< voltage threshold is 2.3V */
#define PMU_LVDT_2                    CTL_LVDT(2)              /*!< voltage threshold is 2.4V */
#define PMU_LVDT_3                    CTL_LVDT(3)              /*!< voltage threshold is 2.5V */
#define PMU_LVDT_4                    CTL_LVDT(4)              /*!< voltage threshold is 2.6V */
#define PMU_LVDT_5                    CTL_LVDT(5)              /*!< voltage threshold is 2.7V */
#define PMU_LVDT_6                    CTL_LVDT(6)              /*!< voltage threshold is 2.8V */
#define PMU_LVDT_7                    CTL_LVDT(7)              /*!< voltage threshold is 2.9V */

/* PMU flag definitions */
#define PMU_FLAG_WAKEUP               PMU_CS_WUF               /*!< wakeup flag status */
#define PMU_FLAG_STANDBY              PMU_CS_STBF              /*!< standby flag status */
#define PMU_FLAG_LVD                  PMU_CS_LVDF              /*!< lvd flag status */

/* PMU ldo definitions */
#define PMU_LDO_NORMAL                ((uint32_t)0x00000000U)  /*!< LDO normal work when PMU enter deepsleep mode */
#define PMU_LDO_LOWPOWER              PMU_CTL_LDOLP            /*!< LDO work at low power status when PMU enter deepsleep mode */

/* PMU flag reset definitions */
#define PMU_FLAG_RESET_WAKEUP         ((uint8_t)0x00U)         /*!< wakeup flag reset */
#define PMU_FLAG_RESET_STANDBY        ((uint8_t)0x01U)         /*!< standby flag reset */

/* PMU command constants definitions */
#define WFI_CMD                       ((uint8_t)0x00U)         /*!< use WFI command */
#define WFE_CMD                       ((uint8_t)0x01U)         /*!< use WFE command */


////////////////////////////////////////////////////////////////////////
/* RCU definitions */
#define RCU                             RCU_BASE

/* registers definitions */

#define RCU_CTL                         REG32(RCU + 0x00U)        /*!< control register */
#define RCU_CFG0                        REG32(RCU + 0x04U)        /*!< clock configuration register 0 */
#define RCU_INT                         REG32(RCU + 0x08U)        /*!< clock interrupt register */
#define RCU_APB2RST                     REG32(RCU + 0x0CU)        /*!< APB2 reset register */
#define RCU_APB1RST                     REG32(RCU + 0x10U)        /*!< APB1 reset register */
#define RCU_AHBEN                       REG32(RCU + 0x14U)        /*!< AHB1 enable register */
#define RCU_APB2EN                      REG32(RCU + 0x18U)        /*!< APB2 enable register */
#define RCU_APB1EN                      REG32(RCU + 0x1CU)        /*!< APB1 enable register */
#define RCU_BDCTL                       REG32(RCU + 0x20U)        /*!< backup domain control register */
#define RCU_RSTSCK                      REG32(RCU + 0x24U)        /*!< reset source / clock register */
#define RCU_AHBRST                      REG32(RCU + 0x28U)        /*!< AHB reset register */
#define RCU_CFG1                        REG32(RCU + 0x2CU)        /*!< clock configuration register 1 */
#define RCU_DSV                         REG32(RCU + 0x34U)        /*!< deep-sleep mode voltage register */


/* bits definitions */
/* RCU_CTL */
#define RCU_CTL_IRC8MEN                 BIT(0)                    /*!< internal high speed oscillator enable */
#define RCU_CTL_IRC8MSTB                BIT(1)                    /*!< IRC8M high speed internal oscillator stabilization flag */
#define RCU_CTL_IRC8MADJ                BITS(3,7)                 /*!< high speed internal oscillator clock trim adjust value */
#define RCU_CTL_IRC8MCALIB              BITS(8,15)                /*!< high speed internal oscillator calibration value register */
#define RCU_CTL_HXTALEN                 BIT(16)                   /*!< external high speed oscillator enable */
#define RCU_CTL_HXTALSTB                BIT(17)                   /*!< external crystal oscillator clock stabilization flag */
#define RCU_CTL_HXTALBPS                BIT(18)                   /*!< external crystal oscillator clock bypass mode enable */
#define RCU_CTL_CKMEN                   BIT(19)                   /*!< HXTAL clock monitor enable */
#define RCU_CTL_PLLEN                   BIT(24)                   /*!< PLL enable */
#define RCU_CTL_PLLSTB                  BIT(25)                   /*!< PLL clock stabilization flag */
#define RCU_CTL_PLL1EN                  BIT(26)                   /*!< PLL1 enable */
#define RCU_CTL_PLL1STB                 BIT(27)                   /*!< PLL1 clock stabilization flag */
#define RCU_CTL_PLL2EN                  BIT(28)                   /*!< PLL2 enable */
#define RCU_CTL_PLL2STB                 BIT(29)                   /*!< PLL2 clock stabilization flag */


#define RCU_CFG0_SCS                    BITS(0,1)                 /*!< system clock switch */
#define RCU_CFG0_SCSS                   BITS(2,3)                 /*!< system clock switch status */
#define RCU_CFG0_AHBPSC                 BITS(4,7)                 /*!< AHB prescaler selection */
#define RCU_CFG0_APB1PSC                BITS(8,10)                /*!< APB1 prescaler selection */
#define RCU_CFG0_APB2PSC                BITS(11,13)               /*!< APB2 prescaler selection */
#define RCU_CFG0_ADCPSC                 BITS(14,15)               /*!< ADC prescaler selection */
#define RCU_CFG0_PLLSEL                 BIT(16)                   /*!< PLL clock source selection */
#define RCU_CFG0_PREDV0_LSB             BIT(17)                   /*!< the LSB of PREDV0 division factor */
#define RCU_CFG0_PLLMF                  BITS(18,21)               /*!< PLL clock multiplication factor */
#define RCU_CFG0_USBFSPSC               BITS(22,23)               /*!< USBFS clock prescaler selection */
#define RCU_CFG0_CKOUT0SEL              BITS(24,27)               /*!< CKOUT0 clock source selection */
#define RCU_CFG0_ADCPSC_2               BIT(28)                   /*!< bit 2 of ADCPSC */
#define RCU_CFG0_PLLMF_4                BIT(29)                   /*!< bit 4 of PLLMF */

/* RCU_INT */
#define RCU_INT_IRC40KSTBIF             BIT(0)                    /*!< IRC40K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF              BIT(1)                    /*!< LXTAL stabilization interrupt flag */
#define RCU_INT_IRC8MSTBIF              BIT(2)                    /*!< IRC8M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF              BIT(3)                    /*!< HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF                BIT(4)                    /*!< PLL stabilization interrupt flag */
#define RCU_INT_PLL1STBIF               BIT(5)                    /*!< PLL1 stabilization interrupt flag */
#define RCU_INT_PLL2STBIF               BIT(6)                    /*!< PLL2 stabilization interrupt flag */
#define RCU_INT_CKMIF                   BIT(7)                    /*!< HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC40KSTBIE             BIT(8)                    /*!< IRC40K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE              BIT(9)                    /*!< LXTAL stabilization interrupt enable */
#define RCU_INT_IRC8MSTBIE              BIT(10)                   /*!< IRC8M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE              BIT(11)                   /*!< HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE                BIT(12)                   /*!< PLL stabilization interrupt enable */
#define RCU_INT_PLL1STBIE               BIT(13)                   /*!< PLL1 stabilization interrupt enable */
#define RCU_INT_PLL2STBIE               BIT(14)                   /*!< PLL2 stabilization interrupt enable */
#define RCU_INT_IRC40KSTBIC             BIT(16)                   /*!< IRC40K stabilization interrupt clear */
#define RCU_INT_LXTALSTBIC              BIT(17)                   /*!< LXTAL stabilization interrupt clear */
#define RCU_INT_IRC8MSTBIC              BIT(18)                   /*!< IRC8M stabilization interrupt clear */
#define RCU_INT_HXTALSTBIC              BIT(19)                   /*!< HXTAL stabilization interrupt clear */
#define RCU_INT_PLLSTBIC                BIT(20)                   /*!< PLL stabilization interrupt clear */
#define RCU_INT_PLL1STBIC               BIT(21)                   /*!< PLL1 stabilization interrupt clear */
#define RCU_INT_PLL2STBIC               BIT(22)                   /*!< PLL2 stabilization interrupt clear */
#define RCU_INT_CKMIC                   BIT(23)                   /*!< HXTAL clock stuck interrupt clear */

/* RCU_APB2RST */
#define RCU_APB2RST_AFRST               BIT(0)                    /*!< alternate function I/O reset */
#define RCU_APB2RST_PARST               BIT(2)                    /*!< GPIO port A reset */
#define RCU_APB2RST_PBRST               BIT(3)                    /*!< GPIO port B reset */
#define RCU_APB2RST_PCRST               BIT(4)                    /*!< GPIO port C reset */
#define RCU_APB2RST_PDRST               BIT(5)                    /*!< GPIO port D reset */
#define RCU_APB2RST_PERST               BIT(6)                    /*!< GPIO port E reset */
#define RCU_APB2RST_ADC0RST             BIT(9)                    /*!< ADC0 reset */
#define RCU_APB2RST_ADC1RST             BIT(10)                   /*!< ADC1 reset */
#define RCU_APB2RST_TIMER0RST           BIT(11)                   /*!< TIMER0 reset */
#define RCU_APB2RST_SPI0RST             BIT(12)                   /*!< SPI0 reset */
#define RCU_APB2RST_USART0RST           BIT(14)                   /*!< USART0 reset */

/* RCU_APB1RST */
#define RCU_APB1RST_TIMER1RST           BIT(0)                    /*!< TIMER1 reset */
#define RCU_APB1RST_TIMER2RST           BIT(1)                    /*!< TIMER2 reset */
#define RCU_APB1RST_TIMER3RST           BIT(2)                    /*!< TIMER3 reset */
#define RCU_APB1RST_TIMER4RST           BIT(3)                    /*!< TIMER4 reset */
#define RCU_APB1RST_TIMER5RST           BIT(4)                    /*!< TIMER5 reset */
#define RCU_APB1RST_TIMER6RST           BIT(5)                    /*!< TIMER6 reset */

#define RCU_APB1RST_WWDGTRST            BIT(11)                   /*!< WWDGT reset */
#define RCU_APB1RST_SPI1RST             BIT(14)                   /*!< SPI1 reset */
#define RCU_APB1RST_SPI2RST             BIT(15)                   /*!< SPI2 reset */
#define RCU_APB1RST_USART1RST           BIT(17)                   /*!< USART1 reset */
#define RCU_APB1RST_USART2RST           BIT(18)                   /*!< USART2 reset */
#define RCU_APB1RST_UART3RST            BIT(19)                   /*!< UART3 reset */
#define RCU_APB1RST_UART4RST            BIT(20)                   /*!< UART4 reset */
#define RCU_APB1RST_I2C0RST             BIT(21)                   /*!< I2C0 reset */
#define RCU_APB1RST_I2C1RST             BIT(22)                   /*!< I2C1 reset */
#define RCU_APB1RST_CAN0RST             BIT(25)                   /*!< CAN0 reset */
#define RCU_APB1RST_CAN1RST             BIT(26)                   /*!< CAN1 reset */
#define RCU_APB1RST_BKPIRST             BIT(27)                   /*!< backup interface reset */
#define RCU_APB1RST_PMURST              BIT(28)                   /*!< PMU reset */
#define RCU_APB1RST_DACRST              BIT(29)                   /*!< DAC reset */

/* RCU_AHBEN */
#define RCU_AHBEN_DMA0EN                BIT(0)                    /*!< DMA0 clock enable */
#define RCU_AHBEN_DMA1EN                BIT(1)                    /*!< DMA1 clock enable */
#define RCU_AHBEN_SRAMSPEN              BIT(2)                    /*!< SRAM clock enable when sleep mode */
#define RCU_AHBEN_FMCSPEN               BIT(4)                    /*!< FMC clock enable when sleep mode */
#define RCU_AHBEN_CRCEN                 BIT(6)                    /*!< CRC clock enable */
#define RCU_AHBEN_EXMCEN                BIT(8)                    /*!< EXMC clock enable */
#define RCU_AHBEN_USBFSEN               BIT(12)                   /*!< USBFS clock enable */

/* RCU_APB2EN */
#define RCU_APB2EN_AFEN                 BIT(0)                    /*!< alternate function IO clock enable */
#define RCU_APB2EN_PAEN                 BIT(2)                    /*!< GPIO port A clock enable */
#define RCU_APB2EN_PBEN                 BIT(3)                    /*!< GPIO port B clock enable */
#define RCU_APB2EN_PCEN                 BIT(4)                    /*!< GPIO port C clock enable */
#define RCU_APB2EN_PDEN                 BIT(5)                    /*!< GPIO port D clock enable */
#define RCU_APB2EN_PEEN                 BIT(6)                    /*!< GPIO port E clock enable */
#define RCU_APB2EN_ADC0EN               BIT(9)                    /*!< ADC0 clock enable */
#define RCU_APB2EN_ADC1EN               BIT(10)                   /*!< ADC1 clock enable */
#define RCU_APB2EN_TIMER0EN             BIT(11)                   /*!< TIMER0 clock enable */
#define RCU_APB2EN_SPI0EN               BIT(12)                   /*!< SPI0 clock enable */
#define RCU_APB2EN_USART0EN             BIT(14)                   /*!< USART0 clock enable */

/* RCU_APB1EN */
#define RCU_APB1EN_TIMER1EN             BIT(0)                    /*!< TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN             BIT(1)                    /*!< TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN             BIT(2)                    /*!< TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN             BIT(3)                    /*!< TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN             BIT(4)                    /*!< TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN             BIT(5)                    /*!< TIMER6 clock enable */
#define RCU_APB1EN_WWDGTEN              BIT(11)                   /*!< WWDGT clock enable */
#define RCU_APB1EN_SPI1EN               BIT(14)                   /*!< SPI1 clock enable */
#define RCU_APB1EN_SPI2EN               BIT(15)                   /*!< SPI2 clock enable */
#define RCU_APB1EN_USART1EN             BIT(17)                   /*!< USART1 clock enable */
#define RCU_APB1EN_USART2EN             BIT(18)                   /*!< USART2 clock enable */
#define RCU_APB1EN_UART3EN              BIT(19)                   /*!< UART3 clock enable */
#define RCU_APB1EN_UART4EN              BIT(20)                   /*!< UART4 clock enable */
#define RCU_APB1EN_I2C0EN               BIT(21)                   /*!< I2C0 clock enable */
#define RCU_APB1EN_I2C1EN               BIT(22)                   /*!< I2C1 clock enable */
#define RCU_APB1EN_CAN0EN               BIT(25)                   /*!< CAN0 clock enable */
#define RCU_APB1EN_CAN1EN               BIT(26)                   /*!< CAN1 clock enable */
#define RCU_APB1EN_BKPIEN               BIT(27)                   /*!< backup interface clock enable */
#define RCU_APB1EN_PMUEN                BIT(28)                   /*!< PMU clock enable */
#define RCU_APB1EN_DACEN                BIT(29)                   /*!< DAC clock enable */

/* RCU_BDCTL */
#define RCU_BDCTL_LXTALEN               BIT(0)                    /*!< LXTAL enable */
#define RCU_BDCTL_LXTALSTB              BIT(1)                    /*!< low speed crystal oscillator stabilization flag */
#define RCU_BDCTL_LXTALBPS              BIT(2)                    /*!< LXTAL bypass mode enable */
#define RCU_BDCTL_RTCSRC                BITS(8,9)                 /*!< RTC clock entry selection */
#define RCU_BDCTL_RTCEN                 BIT(15)                   /*!< RTC clock enable */
#define RCU_BDCTL_BKPRST                BIT(16)                   /*!< backup domain reset */

/* RCU_RSTSCK */
#define RCU_RSTSCK_IRC40KEN             BIT(0)                    /*!< IRC40K enable */
#define RCU_RSTSCK_IRC40KSTB            BIT(1)                    /*!< IRC40K stabilization flag */
#define RCU_RSTSCK_RSTFC                BIT(24)                   /*!< reset flag clear */
#define RCU_RSTSCK_EPRSTF               BIT(26)                   /*!< external pin reset flag */
#define RCU_RSTSCK_PORRSTF              BIT(27)                   /*!< power reset flag */
#define RCU_RSTSCK_SWRSTF               BIT(28)                   /*!< software reset flag */
#define RCU_RSTSCK_FWDGTRSTF            BIT(29)                   /*!< free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF            BIT(30)                   /*!< window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF               BIT(31)                   /*!< low-power reset flag */

/* RCU_AHBRST */
#define RCU_AHBRST_USBFSRST             BIT(12)                   /*!< USBFS reset */

/* RCU_CFG1 */
#define RCU_CFG1_PREDV0                 BITS(0,3)                 /*!< PREDV0 division factor */
#define RCU_CFG1_PREDV1                 BITS(4,7)                 /*!< PREDV1 division factor */
#define RCU_CFG1_PLL1MF                 BITS(8,11)                /*!< PLL1 clock multiplication factor */
#define RCU_CFG1_PLL2MF                 BITS(12,15)               /*!< PLL2 clock multiplication factor */
#define RCU_CFG1_PREDV0SEL              BIT(16)                   /*!< PREDV0 input clock source selection */
#define RCU_CFG1_I2S1SEL                BIT(17)                   /*!< I2S1 clock source selection */
#define RCU_CFG1_I2S2SEL                BIT(18)                   /*!< I2S2 clock source selection  */

/* RCU_DSV */
#define RCU_DSV_DSLPVS                  BITS(0,1)                 /*!< deep-sleep mode voltage select */

/* constants definitions */
/* define the peripheral clock enable bit position and its register index offset */
#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define RCU_REG_VAL(periph)                 (REG32(RCU + ((uint32_t)(periph) >> 6)))
#define RCU_BIT_POS(val)                    ((uint32_t)(val) & 0x1FU)

/* register offset */
/* peripherals enable */
#define AHBEN_REG_OFFSET                0x14U                     /*!< AHB enable register offset */
#define APB1EN_REG_OFFSET               0x1CU                     /*!< APB1 enable register offset */
#define APB2EN_REG_OFFSET               0x18U                     /*!< APB2 enable register offset */

/* peripherals reset */
#define AHBRST_REG_OFFSET               0x28U                     /*!< AHB reset register offset */
#define APB1RST_REG_OFFSET              0x10U                     /*!< APB1 reset register offset */
#define APB2RST_REG_OFFSET              0x0CU                     /*!< APB2 reset register offset */
#define RSTSCK_REG_OFFSET               0x24U                     /*!< reset source/clock register offset */

/* clock control */
#define CTL_REG_OFFSET                  0x00U                     /*!< control register offset */
#define BDCTL_REG_OFFSET                0x20U                     /*!< backup domain control register offset */

/* clock stabilization and stuck interrupt */
#define INT_REG_OFFSET                  0x08U                     /*!< clock interrupt register offset */

/* configuration register */
#define CFG0_REG_OFFSET                 0x04U                     /*!< clock configuration register 0 offset */
#define CFG1_REG_OFFSET                 0x2CU                     /*!< clock configuration register 1 offset */

/* peripheral clock enable */
typedef enum {
    /* AHB peripherals */
    RCU_DMA0 = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 0U),              /*!< DMA0 clock */
    RCU_DMA1 = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 1U),              /*!< DMA1 clock */
    RCU_CRC = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 6U),               /*!< CRC clock */
    RCU_EXMC = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 8U),              /*!< EXMC clock */
    RCU_USBFS = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 12U),            /*!< USBFS clock */
    /* APB1 peripherals */
    RCU_TIMER1 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 0U),           /*!< TIMER1 clock */
    RCU_TIMER2 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 1U),           /*!< TIMER2 clock */
    RCU_TIMER3 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 2U),           /*!< TIMER3 clock */
    RCU_TIMER4 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 3U),           /*!< TIMER4 clock */
    RCU_TIMER5 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 4U),           /*!< TIMER5 clock */
    RCU_TIMER6 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 5U),           /*!< TIMER6 clock */
    RCU_WWDGT = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 11U),           /*!< WWDGT clock */
    RCU_SPI1 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 14U),            /*!< SPI1 clock */
    RCU_SPI2 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 15U),            /*!< SPI2 clock */
    RCU_USART1 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 17U),          /*!< USART1 clock */
    RCU_USART2 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 18U),          /*!< USART2 clock */
    RCU_UART3 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 19U),           /*!< UART3 clock */
    RCU_UART4 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 20U),           /*!< UART4 clock */
    RCU_I2C0 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 21U),            /*!< I2C0 clock */
    RCU_I2C1 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 22U),            /*!< I2C1 clock */
    RCU_CAN0 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 25U),            /*!< CAN0 clock */
    RCU_CAN1 = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 26U),            /*!< CAN1 clock */
    RCU_BKPI = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 27U),            /*!< BKPI clock */
    RCU_PMU = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 28U),             /*!< PMU clock */
    RCU_DAC = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 29U),             /*!< DAC clock */
    RCU_RTC = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 15U),              /*!< RTC clock */
    /* APB2 peripherals */
    RCU_AF = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 0U),               /*!< alternate function clock */
    RCU_GPIOA = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 2U),            /*!< GPIOA clock */
    RCU_GPIOB = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 3U),            /*!< GPIOB clock */
    RCU_GPIOC = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 4U),            /*!< GPIOC clock */
    RCU_GPIOD = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 5U),            /*!< GPIOD clock */
    RCU_GPIOE = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 6U),            /*!< GPIOE clock */
    RCU_ADC0 = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 9U),             /*!< ADC0 clock */
    RCU_ADC1 = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 10U),            /*!< ADC1 clock */
    RCU_TIMER0 = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 11U),          /*!< TIMER0 clock */
    RCU_SPI0 = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 12U),            /*!< SPI0 clock */
    RCU_USART0 = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 14U),          /*!< USART0 clock */
} rcu_periph_enum;

/* peripheral clock enable when sleep mode*/
typedef enum {
/* AHB peripherals */
    RCU_SRAM_SLP = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 2U),          /*!< SRAM clock */
    RCU_FMC_SLP = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 4U),           /*!< FMC clock */
} rcu_periph_sleep_enum;

/* peripherals reset */
typedef enum {
    /* AHB peripherals */
    RCU_USBFSRST = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 12U),        /*!< USBFS clock reset */
    /* APB1 peripherals */
    RCU_TIMER1RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 0U),       /*!< TIMER1 clock reset */
    RCU_TIMER2RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 1U),       /*!< TIMER2 clock reset */
    RCU_TIMER3RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 2U),       /*!< TIMER3 clock reset */
    RCU_TIMER4RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 3U),       /*!< TIMER4 clock reset */
    RCU_TIMER5RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 4U),       /*!< TIMER5 clock reset */
    RCU_TIMER6RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 5U),       /*!< TIMER6 clock reset */
    RCU_WWDGTRST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 11U),       /*!< WWDGT clock reset */
    RCU_SPI1RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 14U),        /*!< SPI1 clock reset */
    RCU_SPI2RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 15U),        /*!< SPI2 clock reset */
    RCU_USART1RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 17U),      /*!< USART1 clock reset */
    RCU_USART2RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 18U),      /*!< USART2 clock reset */
    RCU_UART3RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 19U),       /*!< UART3 clock reset */
    RCU_UART4RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 20U),       /*!< UART4 clock reset */
    RCU_I2C0RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 21U),        /*!< I2C0 clock reset */
    RCU_I2C1RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 22U),        /*!< I2C1 clock reset */
    RCU_CAN0RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 25U),        /*!< CAN0 clock reset */
    RCU_CAN1RST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 26U),        /*!< CAN1 clock reset */
    RCU_BKPIRST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 27U),        /*!< BKPI clock reset */
    RCU_PMURST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 28U),         /*!< PMU clock reset */
    RCU_DACRST = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 29U),         /*!< DAC clock reset */
    /* APB2 peripherals */
    RCU_AFRST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 0U),           /*!< alternate function clock reset */
    RCU_GPIOARST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 2U),        /*!< GPIOA clock reset */
    RCU_GPIOBRST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 3U),        /*!< GPIOB clock reset */
    RCU_GPIOCRST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 4U),        /*!< GPIOC clock reset */
    RCU_GPIODRST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 5U),        /*!< GPIOD clock reset */
    RCU_GPIOERST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 6U),        /*!< GPIOE clock reset */
    RCU_ADC0RST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 9U),         /*!< ADC0 clock reset */
    RCU_ADC1RST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 10U),        /*!< ADC1 clock reset */
    RCU_TIMER0RST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 11U),      /*!< TIMER0 clock reset */
    RCU_SPI0RST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 12U),        /*!< SPI0 clock reset */
    RCU_USART0RST = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 14U),      /*!< USART0 clock reset */
} rcu_periph_reset_enum;

/* clock stabilization and peripheral reset flags */
typedef enum {
    /* clock stabilization flags */
    RCU_FLAG_IRC8MSTB = RCU_REGIDX_BIT(CTL_REG_OFFSET, 1U),       /*!< IRC8M stabilization flags */
    RCU_FLAG_HXTALSTB = RCU_REGIDX_BIT(CTL_REG_OFFSET, 17U),      /*!< HXTAL stabilization flags */
    RCU_FLAG_PLLSTB = RCU_REGIDX_BIT(CTL_REG_OFFSET, 25U),        /*!< PLL stabilization flags */
    RCU_FLAG_PLL1STB = RCU_REGIDX_BIT(CTL_REG_OFFSET, 27U),       /*!< PLL1 stabilization flags */
    RCU_FLAG_PLL2STB = RCU_REGIDX_BIT(CTL_REG_OFFSET, 29U),       /*!< PLL2 stabilization flags */
    RCU_FLAG_LXTALSTB = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 1U),     /*!< LXTAL stabilization flags */
    RCU_FLAG_IRC40KSTB = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 1U),   /*!< IRC40K stabilization flags */
    /* reset source flags */
    RCU_FLAG_EPRST = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 26U),      /*!< external PIN reset flags */
    RCU_FLAG_PORRST = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 27U),     /*!< power reset flags */
    RCU_FLAG_SWRST = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 28U),      /*!< software reset flags */
    RCU_FLAG_FWDGTRST = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 29U),   /*!< FWDGT reset flags */
    RCU_FLAG_WWDGTRST = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 30U),   /*!< WWDGT reset flags */
    RCU_FLAG_LPRST = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 31U),      /*!< low-power reset flags */
} rcu_flag_enum;

/* clock stabilization and ckm interrupt flags */
typedef enum {
    RCU_INT_FLAG_IRC40KSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 0U),  /*!< IRC40K stabilization interrupt flag */
    RCU_INT_FLAG_LXTALSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 1U),   /*!< LXTAL stabilization interrupt flag */
    RCU_INT_FLAG_IRC8MSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 2U),   /*!< IRC8M stabilization interrupt flag */
    RCU_INT_FLAG_HXTALSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 3U),   /*!< HXTAL stabilization interrupt flag */
    RCU_INT_FLAG_PLLSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 4U),     /*!< PLL stabilization interrupt flag */
    RCU_INT_FLAG_PLL1STB = RCU_REGIDX_BIT(INT_REG_OFFSET, 5U),    /*!< PLL1 stabilization interrupt flag */
    RCU_INT_FLAG_PLL2STB = RCU_REGIDX_BIT(INT_REG_OFFSET, 6U),    /*!< PLL2 stabilization interrupt flag */
    RCU_INT_FLAG_CKM = RCU_REGIDX_BIT(INT_REG_OFFSET, 7U),        /*!< HXTAL clock stuck interrupt flag */
} rcu_int_flag_enum;

/* clock stabilization and stuck interrupt flags clear */
typedef enum {
    RCU_INT_FLAG_IRC40KSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 16U), /*!< IRC40K stabilization interrupt flags clear */
    RCU_INT_FLAG_LXTALSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 17U),  /*!< LXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_IRC8MSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 18U),  /*!< IRC8M stabilization interrupt flags clear */
    RCU_INT_FLAG_HXTALSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 19U),  /*!< HXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 20U),    /*!< PLL stabilization interrupt flags clear */
    RCU_INT_FLAG_PLL1STB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 21U),   /*!< PLL1 stabilization interrupt flags clear */
    RCU_INT_FLAG_PLL2STB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 22U),   /*!< PLL2 stabilization interrupt flags clear */
    RCU_INT_FLAG_CKM_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 23U),       /*!< CKM interrupt flags clear */
} rcu_int_flag_clear_enum;

/* clock stabilization interrupt enable or disable */
typedef enum {
    RCU_INT_IRC40KSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 8U),  /*!< IRC40K stabilization interrupt */
    RCU_INT_LXTALSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 9U),   /*!< LXTAL stabilization interrupt */
    RCU_INT_IRC8MSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 10U),  /*!< IRC8M stabilization interrupt */
    RCU_INT_HXTALSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 11U),  /*!< HXTAL stabilization interrupt */
    RCU_INT_PLLSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 12U),    /*!< PLL stabilization interrupt */
    RCU_INT_PLL1STB = RCU_REGIDX_BIT(INT_REG_OFFSET, 13U),   /*!< PLL1 stabilization interrupt */
    RCU_INT_PLL2STB = RCU_REGIDX_BIT(INT_REG_OFFSET, 14U),   /*!< PLL2 stabilization interrupt */
} rcu_int_enum;

/* oscillator types */
typedef enum {
    RCU_HXTAL = RCU_REGIDX_BIT(CTL_REG_OFFSET, 16U),         /*!< HXTAL */
    RCU_LXTAL = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 0U),        /*!< LXTAL */
    RCU_IRC8M = RCU_REGIDX_BIT(CTL_REG_OFFSET, 0U),          /*!< IRC8M */
    RCU_IRC40K = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 0U),      /*!< IRC40K */
    RCU_PLL_CK = RCU_REGIDX_BIT(CTL_REG_OFFSET, 24U),        /*!< PLL */
    RCU_PLL1_CK = RCU_REGIDX_BIT(CTL_REG_OFFSET, 26U),       /*!< PLL1 */
    RCU_PLL2_CK = RCU_REGIDX_BIT(CTL_REG_OFFSET, 28U),       /*!< PLL2 */
} rcu_osci_type_enum;

/* rcu clock frequency */
typedef enum {
    CK_SYS = 0, /*!< system clock */
    CK_AHB, /*!< AHB clock */
    CK_APB1, /*!< APB1 clock */
    CK_APB2, /*!< APB2 clock */
} rcu_clock_freq_enum;

/* RCU_CFG0 register bit define */
/* system clock source select */
#define CFG0_SCS(regval)                (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_CKSYSSRC_IRC8M              CFG0_SCS(0)                         /*!< system clock source select IRC8M */
#define RCU_CKSYSSRC_HXTAL              CFG0_SCS(1)                         /*!< system clock source select HXTAL */
#define RCU_CKSYSSRC_PLL                CFG0_SCS(2)                         /*!< system clock source select PLL */

/* system clock source select status */
#define CFG0_SCSS(regval)               (BITS(2,3) & ((uint32_t)(regval) << 2))
#define RCU_SCSS_IRC8M                  CFG0_SCSS(0)                        /*!< system clock source select IRC8M */
#define RCU_SCSS_HXTAL                  CFG0_SCSS(1)                        /*!< system clock source select HXTAL */
#define RCU_SCSS_PLL                    CFG0_SCSS(2)                        /*!< system clock source select PLLP */
#define RCU_SCSS_MASK                   CFG0_SCSS(3)

/* AHB prescaler selection */
#define CFG0_AHBPSC(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))
#define RCU_AHB_CKSYS_DIV1              CFG0_AHBPSC(0)                      /*!< AHB prescaler select CK_SYS */
#define RCU_AHB_CKSYS_DIV2              CFG0_AHBPSC(8)                      /*!< AHB prescaler select CK_SYS/2 */
#define RCU_AHB_CKSYS_DIV4              CFG0_AHBPSC(9)                      /*!< AHB prescaler select CK_SYS/4 */
#define RCU_AHB_CKSYS_DIV8              CFG0_AHBPSC(10)                     /*!< AHB prescaler select CK_SYS/8 */
#define RCU_AHB_CKSYS_DIV16             CFG0_AHBPSC(11)                     /*!< AHB prescaler select CK_SYS/16 */
#define RCU_AHB_CKSYS_DIV64             CFG0_AHBPSC(12)                     /*!< AHB prescaler select CK_SYS/64 */
#define RCU_AHB_CKSYS_DIV128            CFG0_AHBPSC(13)                     /*!< AHB prescaler select CK_SYS/128 */
#define RCU_AHB_CKSYS_DIV256            CFG0_AHBPSC(14)                     /*!< AHB prescaler select CK_SYS/256 */
#define RCU_AHB_CKSYS_DIV512            CFG0_AHBPSC(15)                     /*!< AHB prescaler select CK_SYS/512 */

/* APB1 prescaler selection */
#define CFG0_APB1PSC(regval)            (BITS(8,10) & ((uint32_t)(regval) << 8))
#define RCU_APB1_CKAHB_DIV1             CFG0_APB1PSC(0)                     /*!< APB1 prescaler select CK_AHB */
#define RCU_APB1_CKAHB_DIV2             CFG0_APB1PSC(4)                     /*!< APB1 prescaler select CK_AHB/2 */
#define RCU_APB1_CKAHB_DIV4             CFG0_APB1PSC(5)                     /*!< APB1 prescaler select CK_AHB/4 */
#define RCU_APB1_CKAHB_DIV8             CFG0_APB1PSC(6)                     /*!< APB1 prescaler select CK_AHB/8 */
#define RCU_APB1_CKAHB_DIV16            CFG0_APB1PSC(7)                     /*!< APB1 prescaler select CK_AHB/16 */

/* APB2 prescaler selection */
#define CFG0_APB2PSC(regval)            (BITS(11,13) & ((uint32_t)(regval) << 11))
#define RCU_APB2_CKAHB_DIV1             CFG0_APB2PSC(0)                     /*!< APB2 prescaler select CK_AHB */
#define RCU_APB2_CKAHB_DIV2             CFG0_APB2PSC(4)                     /*!< APB2 prescaler select CK_AHB/2 */
#define RCU_APB2_CKAHB_DIV4             CFG0_APB2PSC(5)                     /*!< APB2 prescaler select CK_AHB/4 */
#define RCU_APB2_CKAHB_DIV8             CFG0_APB2PSC(6)                     /*!< APB2 prescaler select CK_AHB/8 */
#define RCU_APB2_CKAHB_DIV16            CFG0_APB2PSC(7)                     /*!< APB2 prescaler select CK_AHB/16 */

/* ADC prescaler select */
#define RCU_CKADC_CKAPB2_DIV2           ((uint32_t)0x00000000U)             /*!< ADC prescaler select CK_APB2/2 */
#define RCU_CKADC_CKAPB2_DIV4           ((uint32_t)0x00000001U)             /*!< ADC prescaler select CK_APB2/4 */
#define RCU_CKADC_CKAPB2_DIV6           ((uint32_t)0x00000002U)             /*!< ADC prescaler select CK_APB2/6 */
#define RCU_CKADC_CKAPB2_DIV8           ((uint32_t)0x00000003U)             /*!< ADC prescaler select CK_APB2/8 */
#define RCU_CKADC_CKAPB2_DIV12          ((uint32_t)0x00000005U)             /*!< ADC prescaler select CK_APB2/12 */
#define RCU_CKADC_CKAPB2_DIV16          ((uint32_t)0x00000007U)             /*!< ADC prescaler select CK_APB2/16 */

/* PLL clock source selection */
#define RCU_PLLSRC_IRC8M_DIV2           ((uint32_t)0x00000000U)             /*!< IRC8M/2 clock selected as source clock of PLL */
#define RCU_PLLSRC_HXTAL                RCU_CFG0_PLLSEL                     /*!< HXTAL clock selected as source clock of PLL */

/* PLL clock multiplication factor */
#define PLLMF_4                         RCU_CFG0_PLLMF_4                    /* bit 4 of PLLMF */

#define CFG0_PLLMF(regval)              (BITS(18,21) & ((uint32_t)(regval) << 18))
#define RCU_PLL_MUL2                    CFG0_PLLMF(0)                       /*!< PLL source clock multiply by 2 */
#define RCU_PLL_MUL3                    CFG0_PLLMF(1)                       /*!< PLL source clock multiply by 3 */
#define RCU_PLL_MUL4                    CFG0_PLLMF(2)                       /*!< PLL source clock multiply by 4 */
#define RCU_PLL_MUL5                    CFG0_PLLMF(3)                       /*!< PLL source clock multiply by 5 */
#define RCU_PLL_MUL6                    CFG0_PLLMF(4)                       /*!< PLL source clock multiply by 6 */
#define RCU_PLL_MUL7                    CFG0_PLLMF(5)                       /*!< PLL source clock multiply by 7 */
#define RCU_PLL_MUL8                    CFG0_PLLMF(6)                       /*!< PLL source clock multiply by 8 */
#define RCU_PLL_MUL9                    CFG0_PLLMF(7)                       /*!< PLL source clock multiply by 9 */
#define RCU_PLL_MUL10                   CFG0_PLLMF(8)                       /*!< PLL source clock multiply by 10 */
#define RCU_PLL_MUL11                   CFG0_PLLMF(9)                       /*!< PLL source clock multiply by 11 */
#define RCU_PLL_MUL12                   CFG0_PLLMF(10)                      /*!< PLL source clock multiply by 12 */
#define RCU_PLL_MUL13                   CFG0_PLLMF(11)                      /*!< PLL source clock multiply by 13 */
#define RCU_PLL_MUL14                   CFG0_PLLMF(12)                      /*!< PLL source clock multiply by 14 */
#define RCU_PLL_MUL6_5                  CFG0_PLLMF(13)                      /*!< PLL source clock multiply by 6.5 */
#define RCU_PLL_MUL16                   CFG0_PLLMF(14)                      /*!< PLL source clock multiply by 16 */
#define RCU_PLL_MUL17                   (PLLMF_4 | CFG0_PLLMF(0))           /*!< PLL source clock multiply by 17 */
#define RCU_PLL_MUL18                   (PLLMF_4 | CFG0_PLLMF(1))           /*!< PLL source clock multiply by 18 */
#define RCU_PLL_MUL19                   (PLLMF_4 | CFG0_PLLMF(2))           /*!< PLL source clock multiply by 19 */
#define RCU_PLL_MUL20                   (PLLMF_4 | CFG0_PLLMF(3))           /*!< PLL source clock multiply by 20 */
#define RCU_PLL_MUL21                   (PLLMF_4 | CFG0_PLLMF(4))           /*!< PLL source clock multiply by 21 */
#define RCU_PLL_MUL22                   (PLLMF_4 | CFG0_PLLMF(5))           /*!< PLL source clock multiply by 22 */
#define RCU_PLL_MUL23                   (PLLMF_4 | CFG0_PLLMF(6))           /*!< PLL source clock multiply by 23 */
#define RCU_PLL_MUL24                   (PLLMF_4 | CFG0_PLLMF(7))           /*!< PLL source clock multiply by 24 */
#define RCU_PLL_MUL25                   (PLLMF_4 | CFG0_PLLMF(8))           /*!< PLL source clock multiply by 25 */
#define RCU_PLL_MUL26                   (PLLMF_4 | CFG0_PLLMF(9))           /*!< PLL source clock multiply by 26 */
#define RCU_PLL_MUL27                   (PLLMF_4 | CFG0_PLLMF(10))          /*!< PLL source clock multiply by 27 */
#define RCU_PLL_MUL28                   (PLLMF_4 | CFG0_PLLMF(11))          /*!< PLL source clock multiply by 28 */
#define RCU_PLL_MUL29                   (PLLMF_4 | CFG0_PLLMF(12))          /*!< PLL source clock multiply by 29 */
#define RCU_PLL_MUL30                   (PLLMF_4 | CFG0_PLLMF(13))          /*!< PLL source clock multiply by 30 */
#define RCU_PLL_MUL31                   (PLLMF_4 | CFG0_PLLMF(14))          /*!< PLL source clock multiply by 31 */
#define RCU_PLL_MUL32                   (PLLMF_4 | CFG0_PLLMF(15))          /*!< PLL source clock multiply by 32 */

/* USBFS prescaler select */
#define CFG0_USBPSC(regval)             (BITS(22,23) & ((uint32_t)(regval) << 22))
#define RCU_CKUSB_CKPLL_DIV1_5          CFG0_USBPSC(0)                      /*!< USBFS prescaler select CK_PLL/1.5 */
#define RCU_CKUSB_CKPLL_DIV1            CFG0_USBPSC(1)                      /*!< USBFS prescaler select CK_PLL/1 */
#define RCU_CKUSB_CKPLL_DIV2_5          CFG0_USBPSC(2)                      /*!< USBFS prescaler select CK_PLL/2.5 */
#define RCU_CKUSB_CKPLL_DIV2            CFG0_USBPSC(3)                      /*!< USBFS prescaler select CK_PLL/2 */

/* CKOUT0 clock source selection */
#define CFG0_CKOUT0SEL(regval)          (BITS(24,27) & ((uint32_t)(regval) << 24))
#define RCU_CKOUT0SRC_NONE              CFG0_CKOUT0SEL(0)                   /*!< no clock selected */
#define RCU_CKOUT0SRC_CKSYS             CFG0_CKOUT0SEL(4)                   /*!< system clock selected */
#define RCU_CKOUT0SRC_IRC8M             CFG0_CKOUT0SEL(5)                   /*!< internal 8M RC oscillator clock selected */
#define RCU_CKOUT0SRC_HXTAL             CFG0_CKOUT0SEL(6)                   /*!< high speed crystal oscillator clock (HXTAL) selected */
#define RCU_CKOUT0SRC_CKPLL_DIV2        CFG0_CKOUT0SEL(7)                   /*!< CK_PLL/2 clock selected */
#define RCU_CKOUT0SRC_CKPLL1            CFG0_CKOUT0SEL(8)                   /*!< CK_PLL1 clock selected */
#define RCU_CKOUT0SRC_CKPLL2_DIV2       CFG0_CKOUT0SEL(9)                   /*!< CK_PLL2/2 clock selected */
#define RCU_CKOUT0SRC_EXT1              CFG0_CKOUT0SEL(10)                  /*!< EXT1 selected */
#define RCU_CKOUT0SRC_CKPLL2            CFG0_CKOUT0SEL(11)                  /*!< CK_PLL2 clock selected */

/* RTC clock entry selection */
#define BDCTL_RTCSRC(regval)            (BITS(8,9) & ((uint32_t)(regval) << 8))
#define RCU_RTCSRC_NONE                 BDCTL_RTCSRC(0)                     /*!< no clock selected */
#define RCU_RTCSRC_LXTAL                BDCTL_RTCSRC(1)                     /*!< RTC source clock select LXTAL  */
#define RCU_RTCSRC_IRC40K               BDCTL_RTCSRC(2)                     /*!< RTC source clock select IRC40K */
#define RCU_RTCSRC_HXTAL_DIV_128        BDCTL_RTCSRC(3)                     /*!< RTC source clock select HXTAL/128 */

/* PREDV0 division factor */
#define CFG1_PREDV0(regval)             (BITS(0,3) & ((uint32_t)(regval) << 0))
#define RCU_PREDV0_DIV1                CFG1_PREDV0(0)                       /*!< PREDV0 input source clock not divided */
#define RCU_PREDV0_DIV2                CFG1_PREDV0(1)                       /*!< PREDV0 input source clock divided by 2 */
#define RCU_PREDV0_DIV3                CFG1_PREDV0(2)                       /*!< PREDV0 input source clock divided by 3 */
#define RCU_PREDV0_DIV4                CFG1_PREDV0(3)                       /*!< PREDV0 input source clock divided by 4 */
#define RCU_PREDV0_DIV5                CFG1_PREDV0(4)                       /*!< PREDV0 input source clock divided by 5 */
#define RCU_PREDV0_DIV6                CFG1_PREDV0(5)                       /*!< PREDV0 input source clock divided by 6 */
#define RCU_PREDV0_DIV7                CFG1_PREDV0(6)                       /*!< PREDV0 input source clock divided by 7 */
#define RCU_PREDV0_DIV8                CFG1_PREDV0(7)                       /*!< PREDV0 input source clock divided by 8 */
#define RCU_PREDV0_DIV9                CFG1_PREDV0(8)                       /*!< PREDV0 input source clock divided by 9 */
#define RCU_PREDV0_DIV10               CFG1_PREDV0(9)                       /*!< PREDV0 input source clock divided by 10 */
#define RCU_PREDV0_DIV11               CFG1_PREDV0(10)                      /*!< PREDV0 input source clock divided by 11 */
#define RCU_PREDV0_DIV12               CFG1_PREDV0(11)                      /*!< PREDV0 input source clock divided by 12 */
#define RCU_PREDV0_DIV13               CFG1_PREDV0(12)                      /*!< PREDV0 input source clock divided by 13 */
#define RCU_PREDV0_DIV14               CFG1_PREDV0(13)                      /*!< PREDV0 input source clock divided by 14 */
#define RCU_PREDV0_DIV15               CFG1_PREDV0(14)                      /*!< PREDV0 input source clock divided by 15 */
#define RCU_PREDV0_DIV16               CFG1_PREDV0(15)                      /*!< PREDV0 input source clock divided by 16 */

/* PREDV1 division factor */
#define CFG1_PREDV1(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))
#define RCU_PREDV1_DIV1                CFG1_PREDV1(0)                       /*!< PREDV1 input source clock not divided */
#define RCU_PREDV1_DIV2                CFG1_PREDV1(1)                       /*!< PREDV1 input source clock divided by 2 */
#define RCU_PREDV1_DIV3                CFG1_PREDV1(2)                       /*!< PREDV1 input source clock divided by 3 */
#define RCU_PREDV1_DIV4                CFG1_PREDV1(3)                       /*!< PREDV1 input source clock divided by 4 */
#define RCU_PREDV1_DIV5                CFG1_PREDV1(4)                       /*!< PREDV1 input source clock divided by 5 */
#define RCU_PREDV1_DIV6                CFG1_PREDV1(5)                       /*!< PREDV1 input source clock divided by 6 */
#define RCU_PREDV1_DIV7                CFG1_PREDV1(6)                       /*!< PREDV1 input source clock divided by 7 */
#define RCU_PREDV1_DIV8                CFG1_PREDV1(7)                       /*!< PREDV1 input source clock divided by 8 */
#define RCU_PREDV1_DIV9                CFG1_PREDV1(8)                       /*!< PREDV1 input source clock divided by 9 */
#define RCU_PREDV1_DIV10               CFG1_PREDV1(9)                       /*!< PREDV1 input source clock divided by 10 */
#define RCU_PREDV1_DIV11               CFG1_PREDV1(10)                      /*!< PREDV1 input source clock divided by 11 */
#define RCU_PREDV1_DIV12               CFG1_PREDV1(11)                      /*!< PREDV1 input source clock divided by 12 */
#define RCU_PREDV1_DIV13               CFG1_PREDV1(12)                      /*!< PREDV1 input source clock divided by 13 */
#define RCU_PREDV1_DIV14               CFG1_PREDV1(13)                      /*!< PREDV1 input source clock divided by 14 */
#define RCU_PREDV1_DIV15               CFG1_PREDV1(14)                      /*!< PREDV1 input source clock divided by 15 */
#define RCU_PREDV1_DIV16               CFG1_PREDV1(15)                      /*!< PREDV1 input source clock divided by 16 */

/* PLL1 clock multiplication factor */
#define CFG1_PLL1MF(regval)             (BITS(8,11) & ((uint32_t)(regval) << 8))
#define RCU_PLL1_MUL8                   CFG1_PLL1MF(6)                      /*!< PLL1 source clock multiply by 8 */
#define RCU_PLL1_MUL9                   CFG1_PLL1MF(7)                      /*!< PLL1 source clock multiply by 9 */
#define RCU_PLL1_MUL10                  CFG1_PLL1MF(8)                      /*!< PLL1 source clock multiply by 10 */
#define RCU_PLL1_MUL11                  CFG1_PLL1MF(9)                      /*!< PLL1 source clock multiply by 11 */
#define RCU_PLL1_MUL12                  CFG1_PLL1MF(10)                     /*!< PLL1 source clock multiply by 12 */
#define RCU_PLL1_MUL13                  CFG1_PLL1MF(11)                     /*!< PLL1 source clock multiply by 13 */
#define RCU_PLL1_MUL14                  CFG1_PLL1MF(12)                     /*!< PLL1 source clock multiply by 14 */
#define RCU_PLL1_MUL15                  CFG1_PLL1MF(13)                     /*!< PLL1 source clock multiply by 15 */
#define RCU_PLL1_MUL16                  CFG1_PLL1MF(14)                     /*!< PLL1 source clock multiply by 16 */
#define RCU_PLL1_MUL20                  CFG1_PLL1MF(15)                     /*!< PLL1 source clock multiply by 20 */

/* PLL2 clock multiplication factor */
#define CFG1_PLL2MF(regval)             (BITS(12,15) & ((uint32_t)(regval) << 12))
#define RCU_PLL2_MUL8                   CFG1_PLL2MF(6)                      /*!< PLL2 source clock multiply by 8 */
#define RCU_PLL2_MUL9                   CFG1_PLL2MF(7)                      /*!< PLL2 source clock multiply by 9 */
#define RCU_PLL2_MUL10                  CFG1_PLL2MF(8)                      /*!< PLL2 source clock multiply by 10 */
#define RCU_PLL2_MUL11                  CFG1_PLL2MF(9)                      /*!< PLL2 source clock multiply by 11 */
#define RCU_PLL2_MUL12                  CFG1_PLL2MF(10)                     /*!< PLL2 source clock multiply by 12 */
#define RCU_PLL2_MUL13                  CFG1_PLL2MF(11)                     /*!< PLL2 source clock multiply by 13 */
#define RCU_PLL2_MUL14                  CFG1_PLL2MF(12)                     /*!< PLL2 source clock multiply by 14 */
#define RCU_PLL2_MUL15                  CFG1_PLL2MF(13)                     /*!< PLL2 source clock multiply by 15 */
#define RCU_PLL2_MUL16                  CFG1_PLL2MF(14)                     /*!< PLL2 source clock multiply by 16 */
#define RCU_PLL2_MUL20                  CFG1_PLL2MF(15)                     /*!< PLL2 source clock multiply by 20 */


/* PREDV0 input clock source selection */
#define RCU_PREDV0SRC_HXTAL             ((uint32_t)0x00000000U)             /*!< HXTAL selected as PREDV0 input source clock */
#define RCU_PREDV0SRC_CKPLL1            RCU_CFG1_PREDV0SEL                  /*!< CK_PLL1 selected as PREDV0 input source clock */

/* I2S1 clock source selection */
#define RCU_I2S1SRC_CKSYS               ((uint32_t)0x00000000U)             /*!< system clock selected as I2S1 source clock */
#define RCU_I2S1SRC_CKPLL2_MUL2         RCU_CFG1_I2S1SEL                    /*!< (CK_PLL2 x 2) selected as I2S1 source clock */

/* I2S2 clock source selection */
#define RCU_I2S2SRC_CKSYS               ((uint32_t)0x00000000U)             /*!< system clock selected as I2S2 source clock */
#define RCU_I2S2SRC_CKPLL2_MUL2         RCU_CFG1_I2S2SEL                    /*!< (CK_PLL2 x 2) selected as I2S2 source clock */


/* deep-sleep mode voltage */
#define DSV_DSLPVS(regval)              (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_DEEPSLEEP_V_1_2             DSV_DSLPVS(0)                       /*!< core voltage is 1.2V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_1             DSV_DSLPVS(1)                       /*!< core voltage is 1.1V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_0             DSV_DSLPVS(2)                       /*!< core voltage is 1.0V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_0_9             DSV_DSLPVS(3)                       /*!< core voltage is 0.9V in deep-sleep mode */


////////////////////////////////////////////////////////////////////////
/* RTC definitions */
#define RTC                          RTC_BASE

/* registers definitions */
#define RTC_INTEN                    REG32(RTC + 0x00U)      /*!< interrupt enable register */
#define RTC_CTL                      REG32(RTC + 0x04U)      /*!< control register */
#define RTC_PSCH                     REG32(RTC + 0x08U)      /*!< prescaler high register */
#define RTC_PSCL                     REG32(RTC + 0x0CU)      /*!< prescaler low register */
#define RTC_DIVH                     REG32(RTC + 0x10U)      /*!< divider high register */
#define RTC_DIVL                     REG32(RTC + 0x14U)      /*!< divider low register */
#define RTC_CNTH                     REG32(RTC + 0x18U)      /*!< counter high register */
#define RTC_CNTL                     REG32(RTC + 0x1CU)      /*!< counter low register */
#define RTC_ALRMH                    REG32(RTC + 0x20U)      /*!< alarm high register */
#define RTC_ALRML                    REG32(RTC + 0x24U)      /*!< alarm low register */

/* bits definitions */
/* RTC_INTEN */
#define RTC_INTEN_SCIE               BIT(0)                   /*!< second interrupt enable */
#define RTC_INTEN_ALRMIE             BIT(1)                   /*!< alarm interrupt enable */
#define RTC_INTEN_OVIE               BIT(2)                   /*!< overflow interrupt enable */

/* RTC_CTL */
#define RTC_CTL_SCIF                 BIT(0)                   /*!< second interrupt flag */
#define RTC_CTL_ALRMIF               BIT(1)                   /*!< alarm interrupt flag */
#define RTC_CTL_OVIF                 BIT(2)                   /*!< overflow interrupt flag */
#define RTC_CTL_RSYNF                BIT(3)                   /*!< registers synchronized flag */
#define RTC_CTL_CMF                  BIT(4)                   /*!< configuration mode flag */
#define RTC_CTL_LWOFF                BIT(5)                   /*!< last write operation finished flag */

/* RTC_PSCH */
#define RTC_PSCH_PSC                 BITS(0,3)                /*!< prescaler high value */

/* RTC_PSCL */
#define RTC_PSCL_PSC                 BITS(0,15)               /*!< prescaler low value */

/* RTC_DIVH */
#define RTC_DIVH_DIV                 BITS(0,3)                /*!< divider high value */

/* RTC_DIVL */
#define RTC_DIVL_DIV                 BITS(0,15)               /*!< divider low value */

/* RTC_CNTH */
#define RTC_CNTH_CNT                 BITS(0,15)               /*!< counter high value */

/* RTC_CNTL */
#define RTC_CNTL_CNT                 BITS(0,15)               /*!< counter low value */

/* RTC_ALRMH */
#define RTC_ALRMH_ALRM               BITS(0,15)               /*!< alarm high value */

/* RTC_ALRML */
#define RTC_ALRML_ALRM               BITS(0,15)               /*!< alarm low value */

/* constants definitions */
/* RTC interrupt enable or disable definitions */
#define RTC_INT_SECOND               RTC_INTEN_SCIE           /*!< second interrupt enable */
#define RTC_INT_ALARM                RTC_INTEN_ALRMIE         /*!< alarm interrupt enable */
#define RTC_INT_OVERFLOW             RTC_INTEN_OVIE           /*!< overflow interrupt enable */

/* RTC interrupt flag definitions */
#define RTC_INT_FLAG_SECOND          RTC_CTL_SCIF             /*!< second interrupt flag */
#define RTC_INT_FLAG_ALARM           RTC_CTL_ALRMIF           /*!< alarm interrupt flag */
#define RTC_INT_FLAG_OVERFLOW        RTC_CTL_OVIF             /*!< overflow interrupt flag */

/* RTC flag definitions */
#define RTC_FLAG_SECOND              RTC_CTL_SCIF             /*!< second interrupt flag */
#define RTC_FLAG_ALARM               RTC_CTL_ALRMIF           /*!< alarm interrupt flag */
#define RTC_FLAG_OVERFLOW            RTC_CTL_OVIF             /*!< overflow interrupt flag */
#define RTC_FLAG_RSYN                RTC_CTL_RSYNF            /*!< registers synchronized flag */
#define RTC_FLAG_LWOF                RTC_CTL_LWOFF            /*!< last write operation finished flag */


////////////////////////////////////////////////////////////////////////
/* SPIx(x=0,1,2) definitions */
#define SPI0                            (SPI_BASE + 0x0000F800U)
#define SPI1                            SPI_BASE
#define SPI2                            (SPI_BASE + 0x00000400U)

/* registers definitions */
#define SPI_CTL0(spix)                  REG32((spix) + 0x00000000U)            /*!< SPI control register 0 */
#define SPI_CTL1(spix)                  REG32((spix) + 0x00000004U)            /*!< SPI control register 1 */
#define SPI_STAT(spix)                  REG32((spix) + 0x00000008U)            /*!< SPI status register */
#define SPI_DATA(spix)                  REG32((spix) + 0x0000000CU)            /*!< SPI data register */
#define SPI_CRCPOLY(spix)               REG32((spix) + 0x00000010U)            /*!< SPI CRC polynomial register */
#define SPI_RCRC(spix)                  REG32((spix) + 0x00000014U)            /*!< SPI receive CRC register */
#define SPI_TCRC(spix)                  REG32((spix) + 0x00000018U)            /*!< SPI transmit CRC register */
#define SPI_I2SCTL(spix)                REG32((spix) + 0x0000001CU)            /*!< SPI I2S control register */
#define SPI_I2SPSC(spix)                REG32((spix) + 0x00000020U)            /*!< SPI I2S clock prescaler register */

/* bits definitions */
/* SPI_CTL0 */
#define SPI_CTL0_CKPH                   BIT(0)                                  /*!< clock phase selection*/
#define SPI_CTL0_CKPL                   BIT(1)                                  /*!< clock polarity selection */
#define SPI_CTL0_MSTMOD                 BIT(2)                                  /*!< master mode enable */
#define SPI_CTL0_PSC                    BITS(3,5)                               /*!< master clock prescaler selection */
#define SPI_CTL0_SPIEN                  BIT(6)                                  /*!< SPI enable*/
#define SPI_CTL0_LF                     BIT(7)                                  /*!< LSB first mode */
#define SPI_CTL0_SWNSS                  BIT(8)                                  /*!< NSS pin selection in NSS software mode */
#define SPI_CTL0_SWNSSEN                BIT(9)                                  /*!< NSS software mode selection */
#define SPI_CTL0_RO                     BIT(10)                                 /*!< receive only */
#define SPI_CTL0_FF16                   BIT(11)                                 /*!< data frame size */
#define SPI_CTL0_CRCNT                  BIT(12)                                 /*!< CRC next transfer */
#define SPI_CTL0_CRCEN                  BIT(13)                                 /*!< CRC calculation enable */
#define SPI_CTL0_BDOEN                  BIT(14)                                 /*!< bidirectional transmit output enable*/
#define SPI_CTL0_BDEN                   BIT(15)                                 /*!< bidirectional enable */

/* SPI_CTL1 */
#define SPI_CTL1_DMAREN                 BIT(0)                                 /*!< receive buffer DMA enable */
#define SPI_CTL1_DMATEN                 BIT(1)                                 /*!< transmit buffer DMA enable */
#define SPI_CTL1_NSSDRV                 BIT(2)                                  /*!< drive NSS output */
#define SPI_CTL1_NSSP                   BIT(3)                                  /*!< SPI NSS pulse mode enable */
#define SPI_CTL1_TMOD                   BIT(4)                                  /*!< SPI TI mode enable */
#define SPI_CTL1_ERRIE                  BIT(5)                                  /*!< errors interrupt enable */
#define SPI_CTL1_RBNEIE                 BIT(6)                                  /*!< receive buffer not empty interrupt enable */
#define SPI_CTL1_TBEIE                  BIT(7)                                  /*!< transmit buffer empty interrupt enable */

/* SPI_STAT */
#define SPI_STAT_RBNE                   BIT(0)                                  /*!< receive buffer not empty */
#define SPI_STAT_TBE                    BIT(1)                                  /*!< transmit buffer empty */
#define SPI_STAT_I2SCH                  BIT(2)                                  /*!< I2S channel side */
#define SPI_STAT_TXURERR                BIT(3)                                  /*!< I2S transmission underrun error bit */
#define SPI_STAT_CRCERR                 BIT(4)                                  /*!< SPI CRC error bit */
#define SPI_STAT_CONFERR                BIT(5)                                  /*!< SPI configuration error bit */
#define SPI_STAT_RXORERR                BIT(6)                                  /*!< SPI reception overrun error bit */
#define SPI_STAT_TRANS                  BIT(7)                                  /*!< transmitting on-going bit */
#define SPI_STAT_FERR                   BIT(8)                                  /*!< format error bit */

/* SPI_DATA */
#define SPI_DATA_DATA                   BITS(0,15)                              /*!< data transfer register */

/* SPI_CRCPOLY */
#define SPI_CRCPOLY_CRCPOLY             BITS(0,15)                              /*!< CRC polynomial value */

/* SPI_RCRC */
#define SPI_RCRC_RCRC                   BITS(0,15)                              /*!< RX CRC value */

/* SPI_TCRC */
#define SPI_TCRC_TCRC                   BITS(0,15)                              /*!< TX CRC value */

/* SPI_I2SCTL */
#define SPI_I2SCTL_CHLEN                BIT(0)                                  /*!< channel length */
#define SPI_I2SCTL_DTLEN                BITS(1,2)                               /*!< data length */
#define SPI_I2SCTL_CKPL                 BIT(3)                                  /*!< idle state clock polarity */
#define SPI_I2SCTL_I2SSTD               BITS(4,5)                               /*!< I2S standard selection */
#define SPI_I2SCTL_PCMSMOD              BIT(7)                                  /*!< PCM frame synchronization mode */
#define SPI_I2SCTL_I2SOPMOD             BITS(8,9)                               /*!< I2S operation mode */
#define SPI_I2SCTL_I2SEN                BIT(10)                                 /*!< I2S enable */
#define SPI_I2SCTL_I2SSEL               BIT(11)                                 /*!< I2S mode selection */

/* SPI_I2SPSC */
#define SPI_I2SPSC_DIV                  BITS(0,7)                               /*!< dividing factor for the prescaler */
#define SPI_I2SPSC_OF                   BIT(8)                                  /*!< odd factor for the prescaler */
#define SPI_I2SPSC_MCKOEN               BIT(9)                                  /*!< I2S MCK output enable */

/* constants definitions */
/* SPI and I2S parameter struct definitions */
typedef struct
{   
    uint32_t device_mode;                                                       /*!< SPI master or slave */
    uint32_t trans_mode;                                                        /*!< SPI transfer type */
    uint32_t frame_size;                                                        /*!< SPI frame size */
    uint32_t nss;                                                               /*!< SPI NSS control by handware or software */
    uint32_t endian;                                                            /*!< SPI big endian or little endian */
    uint32_t clock_polarity_phase;                                              /*!< SPI clock phase and polarity */
    uint32_t prescale;                                                          /*!< SPI prescaler factor */
}spi_parameter_struct;

/* SPI mode definitions */
#define SPI_MASTER                      (SPI_CTL0_MSTMOD | SPI_CTL0_SWNSS)      /*!< SPI as master */
#define SPI_SLAVE                       ((uint32_t)0x00000000U)                 /*!< SPI as slave */

/* SPI bidirectional transfer direction */
#define SPI_BIDIRECTIONAL_TRANSMIT      SPI_CTL0_BDOEN                          /*!< SPI work in transmit-only mode */
#define SPI_BIDIRECTIONAL_RECEIVE       (~SPI_CTL0_BDOEN)                       /*!< SPI work in receive-only mode */

/* SPI transmit type */
#define SPI_TRANSMODE_FULLDUPLEX        ((uint32_t)0x00000000U)                 /*!< SPI receive and send data at fullduplex communication */
#define SPI_TRANSMODE_RECEIVEONLY       SPI_CTL0_RO                             /*!< SPI only receive data */
#define SPI_TRANSMODE_BDRECEIVE         SPI_CTL0_BDEN                           /*!< bidirectional receive data */
#define SPI_TRANSMODE_BDTRANSMIT        (SPI_CTL0_BDEN | SPI_CTL0_BDOEN)        /*!< bidirectional transmit data*/

/* SPI frame size */
#define SPI_FRAMESIZE_16BIT             SPI_CTL0_FF16                           /*!< SPI frame size is 16 bits */
#define SPI_FRAMESIZE_8BIT              ((uint32_t)0x00000000U)                 /*!< SPI frame size is 8 bits */

/* SPI NSS control mode */
#define SPI_NSS_SOFT                    SPI_CTL0_SWNSSEN                        /*!< SPI NSS control by software */
#define SPI_NSS_HARD                    ((uint32_t)0x00000000U)                 /*!< SPI NSS control by hardware */

/* SPI transmit way */
#define SPI_ENDIAN_MSB                  ((uint32_t)0x00000000U)                 /*!< SPI transmit way is big endian: transmit MSB first */
#define SPI_ENDIAN_LSB                  SPI_CTL0_LF                             /*!< SPI transmit way is little endian: transmit LSB first */

/* SPI clock phase and polarity */
#define SPI_CK_PL_LOW_PH_1EDGE          ((uint32_t)0x00000000U)                 /*!< SPI clock polarity is low level and phase is first edge */
#define SPI_CK_PL_HIGH_PH_1EDGE         SPI_CTL0_CKPL                           /*!< SPI clock polarity is high level and phase is first edge */
#define SPI_CK_PL_LOW_PH_2EDGE          SPI_CTL0_CKPH                           /*!< SPI clock polarity is low level and phase is second edge */
#define SPI_CK_PL_HIGH_PH_2EDGE         (SPI_CTL0_CKPL | SPI_CTL0_CKPH)         /*!< SPI clock polarity is high level and phase is second edge */

/* SPI clock prescale factor */
#define CTL0_PSC(regval)                (BITS(3,5) & ((uint32_t)(regval) << 3))
#define SPI_PSC_2                       CTL0_PSC(0)                             /*!< SPI clock prescale factor is 2 */
#define SPI_PSC_4                       CTL0_PSC(1)                             /*!< SPI clock prescale factor is 4 */
#define SPI_PSC_8                       CTL0_PSC(2)                             /*!< SPI clock prescale factor is 8 */
#define SPI_PSC_16                      CTL0_PSC(3)                             /*!< SPI clock prescale factor is 16 */
#define SPI_PSC_32                      CTL0_PSC(4)                             /*!< SPI clock prescale factor is 32 */
#define SPI_PSC_64                      CTL0_PSC(5)                             /*!< SPI clock prescale factor is 64 */
#define SPI_PSC_128                     CTL0_PSC(6)                             /*!< SPI clock prescale factor is 128 */
#define SPI_PSC_256                     CTL0_PSC(7)                             /*!< SPI clock prescale factor is 256 */

/* I2S audio sample rate */
#define I2S_AUDIOSAMPLE_8K              ((uint32_t)8000U)                       /*!< I2S audio sample rate is 8KHz */
#define I2S_AUDIOSAMPLE_11K             ((uint32_t)11025U)                      /*!< I2S audio sample rate is 11KHz */
#define I2S_AUDIOSAMPLE_16K             ((uint32_t)16000U)                      /*!< I2S audio sample rate is 16KHz */
#define I2S_AUDIOSAMPLE_22K             ((uint32_t)22050U)                      /*!< I2S audio sample rate is 22KHz */
#define I2S_AUDIOSAMPLE_32K             ((uint32_t)32000U)                      /*!< I2S audio sample rate is 32KHz */
#define I2S_AUDIOSAMPLE_44K             ((uint32_t)44100U)                      /*!< I2S audio sample rate is 44KHz */
#define I2S_AUDIOSAMPLE_48K             ((uint32_t)48000U)                      /*!< I2S audio sample rate is 48KHz */
#define I2S_AUDIOSAMPLE_96K             ((uint32_t)96000U)                      /*!< I2S audio sample rate is 96KHz */
#define I2S_AUDIOSAMPLE_192K            ((uint32_t)192000U)                     /*!< I2S audio sample rate is 192KHz */

/* I2S frame format */
#define I2SCTL_DTLEN(regval)            (BITS(1,2) & ((uint32_t)(regval) << 1))
#define I2S_FRAMEFORMAT_DT16B_CH16B     I2SCTL_DTLEN(0)                         /*!< I2S data length is 16 bit and channel length is 16 bit */
#define I2S_FRAMEFORMAT_DT16B_CH32B     (I2SCTL_DTLEN(0) | SPI_I2SCTL_CHLEN)    /*!< I2S data length is 16 bit and channel length is 32 bit */
#define I2S_FRAMEFORMAT_DT24B_CH32B     (I2SCTL_DTLEN(1) | SPI_I2SCTL_CHLEN)    /*!< I2S data length is 24 bit and channel length is 32 bit */
#define I2S_FRAMEFORMAT_DT32B_CH32B     (I2SCTL_DTLEN(2) | SPI_I2SCTL_CHLEN)    /*!< I2S data length is 32 bit and channel length is 32 bit */

/* I2S master clock output */
#define I2S_MCKOUT_DISABLE              ((uint32_t)0x00000000U)                 /*!< I2S master clock output disable */
#define I2S_MCKOUT_ENABLE               SPI_I2SPSC_MCKOEN                       /*!< I2S master clock output enable */

/* I2S operation mode */
#define I2SCTL_I2SOPMOD(regval)         (BITS(8,9) & ((uint32_t)(regval) << 8))
#define I2S_MODE_SLAVETX                I2SCTL_I2SOPMOD(0)                      /*!< I2S slave transmit mode */
#define I2S_MODE_SLAVERX                I2SCTL_I2SOPMOD(1)                      /*!< I2S slave receive mode */
#define I2S_MODE_MASTERTX               I2SCTL_I2SOPMOD(2)                      /*!< I2S master transmit mode */
#define I2S_MODE_MASTERRX               I2SCTL_I2SOPMOD(3)                      /*!< I2S master receive mode */

/* I2S standard */
#define I2SCTL_I2SSTD(regval)           (BITS(4,5) & ((uint32_t)(regval) << 4))
#define I2S_STD_PHILLIPS                I2SCTL_I2SSTD(0)                        /*!< I2S phillips standard */
#define I2S_STD_MSB                     I2SCTL_I2SSTD(1)                        /*!< I2S MSB standard */
#define I2S_STD_LSB                     I2SCTL_I2SSTD(2)                        /*!< I2S LSB standard */
#define I2S_STD_PCMSHORT                I2SCTL_I2SSTD(3)                        /*!< I2S PCM short standard */
#define I2S_STD_PCMLONG                 (I2SCTL_I2SSTD(3) | SPI_I2SCTL_PCMSMOD) /*!< I2S PCM long standard */

/* I2S clock polarity */
#define I2S_CKPL_LOW                    ((uint32_t)0x00000000U)                 /*!< I2S clock polarity low level */
#define I2S_CKPL_HIGH                   SPI_I2SCTL_CKPL                         /*!< I2S clock polarity high level */

/* SPI DMA constants definitions */                                    
#define SPI_DMA_TRANSMIT                ((uint8_t)0x00U)                        /*!< SPI transmit data use DMA */
#define SPI_DMA_RECEIVE                 ((uint8_t)0x01U)                        /*!< SPI receive data use DMA */

/* SPI CRC constants definitions */
#define SPI_CRC_TX                      ((uint8_t)0x00U)                        /*!< SPI transmit CRC value */
#define SPI_CRC_RX                      ((uint8_t)0x01U)                        /*!< SPI receive CRC value */

/* SPI/I2S interrupt enable/disable constants definitions */
#define SPI_I2S_INT_TBE                 SPI_CTL1_TBEIE                          /*!< transmit buffer empty interrupt */
#define SPI_I2S_INT_RBNE                SPI_CTL1_RBNEIE                         /*!< receive buffer not empty interrupt */
#define SPI_I2S_INT_ERR                 SPI_CTL1_ERRIE                          /*!< error interrupt */

/* SPI/I2S interrupt flag constants definitions */
#define SPI_I2S_INT_FLAG_TBE            ((uint8_t)0x00U)                        /*!< transmit buffer empty interrupt flag */
#define SPI_I2S_INT_FLAG_RBNE           ((uint8_t)0x01U)                        /*!< receive buffer not empty interrupt flag */
#define SPI_I2S_INT_FLAG_RXORERR        ((uint8_t)0x02U)                        /*!< overrun interrupt flag */
#define SPI_INT_FLAG_CONFERR            ((uint8_t)0x03U)                        /*!< config error interrupt flag */
#define SPI_INT_FLAG_CRCERR             ((uint8_t)0x04U)                        /*!< CRC error interrupt flag */
#define I2S_INT_FLAG_TXURERR            ((uint8_t)0x05U)                        /*!< underrun error interrupt flag */
#define SPI_I2S_INT_FLAG_FERR           ((uint8_t)0x06U)                        /*!< format error interrupt flag */

/* SPI/I2S flag definitions */                                                  
#define SPI_FLAG_RBNE                   SPI_STAT_RBNE                           /*!< receive buffer not empty flag */
#define SPI_FLAG_TBE                    SPI_STAT_TBE                            /*!< transmit buffer empty flag */
#define SPI_FLAG_CRCERR                 SPI_STAT_CRCERR                         /*!< CRC error flag */
#define SPI_FLAG_CONFERR                SPI_STAT_CONFERR                        /*!< mode config error flag */
#define SPI_FLAG_RXORERR                SPI_STAT_RXORERR                        /*!< receive overrun error flag */
#define SPI_FLAG_TRANS                  SPI_STAT_TRANS                          /*!< transmit on-going flag */
#define SPI_FLAG_FERR                   SPI_STAT_FERR                           /*!< format error flag */
#define I2S_FLAG_RBNE                   SPI_STAT_RBNE                           /*!< receive buffer not empty flag */
#define I2S_FLAG_TBE                    SPI_STAT_TBE                            /*!< transmit buffer empty flag */
#define I2S_FLAG_CH                     SPI_STAT_I2SCH                          /*!< channel side flag */
#define I2S_FLAG_TXURERR                SPI_STAT_TXURERR                        /*!< underrun error flag */
#define I2S_FLAG_RXORERR                SPI_STAT_RXORERR                        /*!< overrun error flag */
#define I2S_FLAG_TRANS                  SPI_STAT_TRANS                          /*!< transmit on-going flag */
#define I2S_FLAG_FERR                   SPI_STAT_FERR                           /*!< format error flag */


////////////////////////////////////////////////////////////////////////
/* TIMERx(x=0..13) definitions */
#define TIMER0                           (TIMER_BASE + 0x00012C00U)
#define TIMER1                           (TIMER_BASE + 0x00000000U)
#define TIMER2                           (TIMER_BASE + 0x00000400U)
#define TIMER3                           (TIMER_BASE + 0x00000800U)
#define TIMER4                           (TIMER_BASE + 0x00000C00U)
#define TIMER5                           (TIMER_BASE + 0x00001000U)
#define TIMER6                           (TIMER_BASE + 0x00001400U)

/* registers definitions */
#define TIMER_CTL0(timerx)               REG32((timerx) + 0x00U)           /*!< TIMER control register 0 */
#define TIMER_CTL1(timerx)               REG32((timerx) + 0x04U)           /*!< TIMER control register 1 */
#define TIMER_SMCFG(timerx)              REG32((timerx) + 0x08U)           /*!< TIMER slave mode configuration register */
#define TIMER_DMAINTEN(timerx)           REG32((timerx) + 0x0CU)           /*!< TIMER DMA and interrupt enable register */
#define TIMER_INTF(timerx)               REG32((timerx) + 0x10U)           /*!< TIMER interrupt flag register */
#define TIMER_SWEVG(timerx)              REG32((timerx) + 0x14U)           /*!< TIMER software event generation register */
#define TIMER_CHCTL0(timerx)             REG32((timerx) + 0x18U)           /*!< TIMER channel control register 0 */
#define TIMER_CHCTL1(timerx)             REG32((timerx) + 0x1CU)           /*!< TIMER channel control register 1 */
#define TIMER_CHCTL2(timerx)             REG32((timerx) + 0x20U)           /*!< TIMER channel control register 2 */
#define TIMER_CNT(timerx)                REG32((timerx) + 0x24U)           /*!< TIMER counter register */
#define TIMER_PSC(timerx)                REG32((timerx) + 0x28U)           /*!< TIMER prescaler register */
#define TIMER_CAR(timerx)                REG32((timerx) + 0x2CU)           /*!< TIMER counter auto reload register */
#define TIMER_CREP(timerx)               REG32((timerx) + 0x30U)           /*!< TIMER counter repetition register */
#define TIMER_CH0CV(timerx)              REG32((timerx) + 0x34U)           /*!< TIMER channel 0 capture/compare value register */
#define TIMER_CH1CV(timerx)              REG32((timerx) + 0x38U)           /*!< TIMER channel 1 capture/compare value register */
#define TIMER_CH2CV(timerx)              REG32((timerx) + 0x3CU)           /*!< TIMER channel 2 capture/compare value register */
#define TIMER_CH3CV(timerx)              REG32((timerx) + 0x40U)           /*!< TIMER channel 3 capture/compare value register */
#define TIMER_CCHP(timerx)               REG32((timerx) + 0x44U)           /*!< TIMER channel complementary protection register */
#define TIMER_DMACFG(timerx)             REG32((timerx) + 0x48U)           /*!< TIMER DMA configuration register */
#define TIMER_DMATB(timerx)              REG32((timerx) + 0x4CU)           /*!< TIMER DMA transfer buffer register */

/* bits definitions */
/* TIMER_CTL0 */
#define TIMER_CTL0_CEN                   BIT(0)              /*!< TIMER counter enable */
#define TIMER_CTL0_UPDIS                 BIT(1)              /*!< update disable */
#define TIMER_CTL0_UPS                   BIT(2)              /*!< update source */
#define TIMER_CTL0_SPM                   BIT(3)              /*!< single pulse mode */
#define TIMER_CTL0_DIR                   BIT(4)              /*!< timer counter direction */
#define TIMER_CTL0_CAM                   BITS(5,6)           /*!< center-aligned mode selection */
#define TIMER_CTL0_ARSE                  BIT(7)              /*!< auto-reload shadow enable */
#define TIMER_CTL0_CKDIV                 BITS(8,9)           /*!< clock division */

/* TIMER_CTL1 */
#define TIMER_CTL1_CCSE                  BIT(0)              /*!< commutation control shadow enable */
#define TIMER_CTL1_CCUC                  BIT(2)              /*!< commutation control shadow register update control */
#define TIMER_CTL1_DMAS                  BIT(3)              /*!< DMA request source selection */
#define TIMER_CTL1_MMC                   BITS(4,6)           /*!< master mode control */
#define TIMER_CTL1_TI0S                  BIT(7)              /*!< channel 0 trigger input selection(hall mode selection) */
#define TIMER_CTL1_ISO0                  BIT(8)              /*!< idle state of channel 0 output */
#define TIMER_CTL1_ISO0N                 BIT(9)              /*!< idle state of channel 0 complementary output */
#define TIMER_CTL1_ISO1                  BIT(10)             /*!< idle state of channel 1 output */
#define TIMER_CTL1_ISO1N                 BIT(11)             /*!< idle state of channel 1 complementary output */
#define TIMER_CTL1_ISO2                  BIT(12)             /*!< idle state of channel 2 output */
#define TIMER_CTL1_ISO2N                 BIT(13)             /*!< idle state of channel 2 complementary output */
#define TIMER_CTL1_ISO3                  BIT(14)             /*!< idle state of channel 3 output */

/* TIMER_SMCFG */
#define TIMER_SMCFG_SMC                  BITS(0,2)           /*!< slave mode control */
#define TIMER_SMCFG_TRGS                 BITS(4,6)           /*!< trigger selection */
#define TIMER_SMCFG_MSM                  BIT(7)              /*!< master-slave mode */
#define TIMER_SMCFG_ETFC                 BITS(8,11)          /*!< external trigger filter control */
#define TIMER_SMCFG_ETPSC                BITS(12,13)         /*!< external trigger prescaler */
#define TIMER_SMCFG_SMC1                 BIT(14)             /*!< part of SMC for enable external clock mode 1 */
#define TIMER_SMCFG_ETP                  BIT(15)             /*!< external trigger polarity */

/* TIMER_DMAINTEN */
#define TIMER_DMAINTEN_UPIE              BIT(0)              /*!< update interrupt enable */
#define TIMER_DMAINTEN_CH0IE             BIT(1)              /*!< channel 0 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH1IE             BIT(2)              /*!< channel 1 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH2IE             BIT(3)              /*!< channel 2 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH3IE             BIT(4)              /*!< channel 3 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CMTIE             BIT(5)              /*!< commutation interrupt request enable */
#define TIMER_DMAINTEN_TRGIE             BIT(6)              /*!< trigger interrupt enable */
#define TIMER_DMAINTEN_BRKIE             BIT(7)              /*!< break interrupt enable */
#define TIMER_DMAINTEN_UPDEN             BIT(8)              /*!< update DMA request enable */
#define TIMER_DMAINTEN_CH0DEN            BIT(9)              /*!< channel 0 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH1DEN            BIT(10)             /*!< channel 1 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH2DEN            BIT(11)             /*!< channel 2 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH3DEN            BIT(12)             /*!< channel 3 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CMTDEN            BIT(13)             /*!< commutation DMA request enable */
#define TIMER_DMAINTEN_TRGDEN            BIT(14)             /*!< trigger DMA request enable */

/* TIMER_INTF */
#define TIMER_INTF_UPIF                  BIT(0)              /*!< update interrupt flag */
#define TIMER_INTF_CH0IF                 BIT(1)              /*!< channel 0 capture/compare interrupt flag */
#define TIMER_INTF_CH1IF                 BIT(2)              /*!< channel 1 capture/compare interrupt flag */
#define TIMER_INTF_CH2IF                 BIT(3)              /*!< channel 2 capture/compare interrupt flag */
#define TIMER_INTF_CH3IF                 BIT(4)              /*!< channel 3 capture/compare interrupt flag */
#define TIMER_INTF_CMTIF                 BIT(5)              /*!< channel commutation interrupt flag */
#define TIMER_INTF_TRGIF                 BIT(6)              /*!< trigger interrupt flag */
#define TIMER_INTF_BRKIF                 BIT(7)              /*!< break interrupt flag */
#define TIMER_INTF_CH0OF                 BIT(9)              /*!< channel 0 over capture flag */
#define TIMER_INTF_CH1OF                 BIT(10)             /*!< channel 1 over capture flag */
#define TIMER_INTF_CH2OF                 BIT(11)             /*!< channel 2 over capture flag */
#define TIMER_INTF_CH3OF                 BIT(12)             /*!< channel 3 over capture flag */

/* TIMER_SWEVG */
#define TIMER_SWEVG_UPG                  BIT(0)              /*!< update event generate */
#define TIMER_SWEVG_CH0G                 BIT(1)              /*!< channel 0 capture or compare event generation */
#define TIMER_SWEVG_CH1G                 BIT(2)              /*!< channel 1 capture or compare event generation */
#define TIMER_SWEVG_CH2G                 BIT(3)              /*!< channel 2 capture or compare event generation */
#define TIMER_SWEVG_CH3G                 BIT(4)              /*!< channel 3 capture or compare event generation */
#define TIMER_SWEVG_CMTG                 BIT(5)              /*!< channel commutation event generation */
#define TIMER_SWEVG_TRGG                 BIT(6)              /*!< trigger event generation */
#define TIMER_SWEVG_BRKG                 BIT(7)              /*!< break event generation */

/* TIMER_CHCTL0 */
/* output compare mode */
#define TIMER_CHCTL0_CH0MS               BITS(0,1)           /*!< channel 0 mode selection */
#define TIMER_CHCTL0_CH0COMFEN           BIT(2)              /*!< channel 0 output compare fast enable */
#define TIMER_CHCTL0_CH0COMSEN           BIT(3)              /*!< channel 0 output compare shadow enable */
#define TIMER_CHCTL0_CH0COMCTL           BITS(4,6)           /*!< channel 0 output compare control  */
#define TIMER_CHCTL0_CH0COMCEN           BIT(7)              /*!< channel 0 output compare clear enable */
#define TIMER_CHCTL0_CH1MS               BITS(8,9)           /*!< channel 1 mode selection */
#define TIMER_CHCTL0_CH1COMFEN           BIT(10)             /*!< channel 1 output compare fast enable */
#define TIMER_CHCTL0_CH1COMSEN           BIT(11)             /*!< channel 1 output compare shadow enable */
#define TIMER_CHCTL0_CH1COMCTL           BITS(12,14)         /*!< channel 1 output compare control  */
#define TIMER_CHCTL0_CH1COMCEN           BIT(15)             /*!< channel 1 output compare clear enable */
/* input capture mode */
#define TIMER_CHCTL0_CH0CAPPSC           BITS(2,3)           /*!< channel 0 input capture prescaler */
#define TIMER_CHCTL0_CH0CAPFLT           BITS(4,7)           /*!< channel 0 input capture filter control */
#define TIMER_CHCTL0_CH1CAPPSC           BITS(10,11)         /*!< channel 1 input capture prescaler */
#define TIMER_CHCTL0_CH1CAPFLT           BITS(12,15)         /*!< channel 1 input capture filter control */

/* TIMER_CHCTL1 */
/* output compare mode */
#define TIMER_CHCTL1_CH2MS               BITS(0,1)           /*!< channel 2 mode selection */
#define TIMER_CHCTL1_CH2COMFEN           BIT(2)              /*!< channel 2 output compare fast enable */
#define TIMER_CHCTL1_CH2COMSEN           BIT(3)              /*!< channel 2 output compare shadow enable */
#define TIMER_CHCTL1_CH2COMCTL           BITS(4,6)           /*!< channel 2 output compare control */
#define TIMER_CHCTL1_CH2COMCEN           BIT(7)              /*!< channel 2 output compare clear enable */
#define TIMER_CHCTL1_CH3MS               BITS(8,9)           /*!< channel 3 mode selection */
#define TIMER_CHCTL1_CH3COMFEN           BIT(10)             /*!< channel 3 output compare fast enable */
#define TIMER_CHCTL1_CH3COMSEN           BIT(11)             /*!< channel 3 output compare shadow enable */
#define TIMER_CHCTL1_CH3COMCTL           BITS(12,14)         /*!< channel 3 output compare control */
#define TIMER_CHCTL1_CH3COMCEN           BIT(15)             /*!< channel 3 output compare clear enable */
/* input capture mode */
#define TIMER_CHCTL1_CH2CAPPSC           BITS(2,3)           /*!< channel 2 input capture prescaler */
#define TIMER_CHCTL1_CH2CAPFLT           BITS(4,7)           /*!< channel 2 input capture filter control */
#define TIMER_CHCTL1_CH3CAPPSC           BITS(10,11)         /*!< channel 3 input capture prescaler */
#define TIMER_CHCTL1_CH3CAPFLT           BITS(12,15)         /*!< channel 3 input capture filter control */

/* TIMER_CHCTL2 */
#define TIMER_CHCTL2_CH0EN               BIT(0)              /*!< channel 0 capture/compare function enable */
#define TIMER_CHCTL2_CH0P                BIT(1)              /*!< channel 0 capture/compare function polarity */
#define TIMER_CHCTL2_CH0NEN              BIT(2)              /*!< channel 0 complementary output enable */
#define TIMER_CHCTL2_CH0NP               BIT(3)              /*!< channel 0 complementary output polarity */
#define TIMER_CHCTL2_CH1EN               BIT(4)              /*!< channel 1 capture/compare function enable  */
#define TIMER_CHCTL2_CH1P                BIT(5)              /*!< channel 1 capture/compare function polarity */
#define TIMER_CHCTL2_CH1NEN              BIT(6)              /*!< channel 1 complementary output enable */
#define TIMER_CHCTL2_CH1NP               BIT(7)              /*!< channel 1 complementary output polarity */
#define TIMER_CHCTL2_CH2EN               BIT(8)              /*!< channel 2 capture/compare function enable  */
#define TIMER_CHCTL2_CH2P                BIT(9)              /*!< channel 2 capture/compare function polarity */
#define TIMER_CHCTL2_CH2NEN              BIT(10)             /*!< channel 2 complementary output enable */
#define TIMER_CHCTL2_CH2NP               BIT(11)             /*!< channel 2 complementary output polarity */
#define TIMER_CHCTL2_CH3EN               BIT(12)             /*!< channel 3 capture/compare function enable  */
#define TIMER_CHCTL2_CH3P                BIT(13)             /*!< channel 3 capture/compare function polarity */

/* TIMER_CNT */
#define TIMER_CNT_CNT                    BITS(0,15)          /*!< 16 bit timer counter */

/* TIMER_PSC */
#define TIMER_PSC_PSC                    BITS(0,15)          /*!< prescaler value of the counter clock */

/* TIMER_CAR */
#define TIMER_CAR_CARL                   BITS(0,15)          /*!< 16 bit counter auto reload value */

/* TIMER_CREP */
#define TIMER_CREP_CREP                  BITS(0,7)           /*!< counter repetition value */

/* TIMER_CH0CV */
#define TIMER_CH0CV_CH0VAL               BITS(0,15)          /*!< 16 bit capture/compare value of channel 0 */

/* TIMER_CH1CV */
#define TIMER_CH1CV_CH1VAL               BITS(0,15)          /*!< 16 bit capture/compare value of channel 1 */

/* TIMER_CH2CV */
#define TIMER_CH2CV_CH2VAL               BITS(0,15)          /*!< 16 bit capture/compare value of channel 2 */

/* TIMER_CH3CV */
#define TIMER_CH3CV_CH3VAL               BITS(0,15)          /*!< 16 bit capture/compare value of channel 3 */

/* TIMER_CCHP */
#define TIMER_CCHP_DTCFG                 BITS(0,7)           /*!< dead time configure */
#define TIMER_CCHP_PROT                  BITS(8,9)           /*!< complementary register protect control */
#define TIMER_CCHP_IOS                   BIT(10)             /*!< idle mode off-state configure */
#define TIMER_CCHP_ROS                   BIT(11)             /*!< run mode off-state configure */
#define TIMER_CCHP_BRKEN                 BIT(12)             /*!< break enable */
#define TIMER_CCHP_BRKP                  BIT(13)             /*!< break polarity */
#define TIMER_CCHP_OAEN                  BIT(14)             /*!< output automatic enable */
#define TIMER_CCHP_POEN                  BIT(15)             /*!< primary output enable */

/* TIMER_DMACFG */
#define TIMER_DMACFG_DMATA               BITS(0,4)           /*!< DMA transfer access start address */
#define TIMER_DMACFG_DMATC               BITS(8,12)          /*!< DMA transfer count */

/* TIMER_DMATB */
#define TIMER_DMATB_DMATB                BITS(0,15)          /*!< DMA transfer buffer address */

/* constants definitions */
/* TIMER init parameter struct definitions */
typedef struct
{ 
    uint16_t prescaler;                                      /*!< prescaler value */
    uint16_t alignedmode;                                    /*!< aligned mode */
    uint16_t counterdirection;                               /*!< counter direction */
    uint32_t period;                                         /*!< period value */
    uint16_t clockdivision;                                  /*!< clock division value */
    uint8_t  repetitioncounter;                              /*!< the counter repetition value */
}timer_parameter_struct;

/* break parameter struct definitions */
typedef struct
{ 
    uint16_t runoffstate;                                    /*!< run mode off-state */
    uint16_t ideloffstate;                                   /*!< idle mode off-state */
    uint16_t deadtime;                                       /*!< dead time */
    uint16_t breakpolarity;                                  /*!< break polarity */
    uint16_t outputautostate;                                /*!< output automatic enable */
    uint16_t protectmode;                                    /*!< complementary register protect control */
    uint16_t breakstate;                                     /*!< break enable */
}timer_break_parameter_struct;

/* channel output parameter struct definitions */
typedef struct
{ 
    uint16_t outputstate;                                    /*!< channel output state */
    uint16_t outputnstate;                                   /*!< channel complementary output state */
    uint16_t ocpolarity;                                     /*!< channel output polarity */
    uint16_t ocnpolarity;                                    /*!< channel complementary output polarity */
    uint16_t ocidlestate;                                    /*!< idle state of channel output */
    uint16_t ocnidlestate;                                   /*!< idle state of channel complementary output */
}timer_oc_parameter_struct;

/* channel input parameter struct definitions */
typedef struct
{ 
    uint16_t icpolarity;                                     /*!< channel input polarity */
    uint16_t icselection;                                    /*!< channel input mode selection */
    uint16_t icprescaler;                                    /*!< channel input capture prescaler */
    uint16_t icfilter;                                       /*!< channel input capture filter control */
}timer_ic_parameter_struct;

/* TIMER interrupt enable or disable */
#define TIMER_INT_UP                        TIMER_DMAINTEN_UPIE                     /*!< update interrupt */
#define TIMER_INT_CH0                       TIMER_DMAINTEN_CH0IE                    /*!< channel 0 interrupt */
#define TIMER_INT_CH1                       TIMER_DMAINTEN_CH1IE                    /*!< channel 1 interrupt */
#define TIMER_INT_CH2                       TIMER_DMAINTEN_CH2IE                    /*!< channel 2 interrupt */
#define TIMER_INT_CH3                       TIMER_DMAINTEN_CH3IE                    /*!< channel 3 interrupt */
#define TIMER_INT_CMT                       TIMER_DMAINTEN_CMTIE                    /*!< channel commutation interrupt flag */
#define TIMER_INT_TRG                       TIMER_DMAINTEN_TRGIE                    /*!< trigger interrupt */
#define TIMER_INT_BRK                       TIMER_DMAINTEN_BRKIE                    /*!< break interrupt */

/* TIMER interrupt flag */
#define TIMER_INT_FLAG_UP                   TIMER_INT_UP                            /*!< update interrupt */
#define TIMER_INT_FLAG_CH0                  TIMER_INT_CH0                           /*!< channel 0 interrupt */
#define TIMER_INT_FLAG_CH1                  TIMER_INT_CH1                           /*!< channel 1 interrupt */
#define TIMER_INT_FLAG_CH2                  TIMER_INT_CH2                           /*!< channel 2 interrupt */
#define TIMER_INT_FLAG_CH3                  TIMER_INT_CH3                           /*!< channel 3 interrupt */
#define TIMER_INT_FLAG_CMT                  TIMER_INT_CMT                           /*!< channel commutation interrupt flag */
#define TIMER_INT_FLAG_TRG                  TIMER_INT_TRG                           /*!< trigger interrupt */
#define TIMER_INT_FLAG_BRK                  TIMER_INT_BRK  

/* TIMER flag */
#define TIMER_FLAG_UP                       TIMER_INTF_UPIF                         /*!< update flag */
#define TIMER_FLAG_CH0                      TIMER_INTF_CH0IF                        /*!< channel 0 flag */
#define TIMER_FLAG_CH1                      TIMER_INTF_CH1IF                        /*!< channel 1 flag */
#define TIMER_FLAG_CH2                      TIMER_INTF_CH2IF                        /*!< channel 2 flag */
#define TIMER_FLAG_CH3                      TIMER_INTF_CH3IF                        /*!< channel 3 flag */
#define TIMER_FLAG_CMT                      TIMER_INTF_CMTIF                        /*!< channel control update flag */
#define TIMER_FLAG_TRG                      TIMER_INTF_TRGIF                        /*!< trigger flag */
#define TIMER_FLAG_BRK                      TIMER_INTF_BRKIF                        /*!< break flag */
#define TIMER_FLAG_CH0O                     TIMER_INTF_CH0OF                        /*!< channel 0 overcapture flag */
#define TIMER_FLAG_CH1O                     TIMER_INTF_CH1OF                        /*!< channel 1 overcapture flag */
#define TIMER_FLAG_CH2O                     TIMER_INTF_CH2OF                        /*!< channel 2 overcapture flag */
#define TIMER_FLAG_CH3O                     TIMER_INTF_CH3OF                        /*!< channel 3 overcapture flag */

/* TIMER DMA source enable */
#define TIMER_DMA_UPD                       ((uint16_t)TIMER_DMAINTEN_UPDEN)        /*!< update DMA enable */
#define TIMER_DMA_CH0D                      ((uint16_t)TIMER_DMAINTEN_CH0DEN)       /*!< channel 0 DMA enable */
#define TIMER_DMA_CH1D                      ((uint16_t)TIMER_DMAINTEN_CH1DEN)       /*!< channel 1 DMA enable */
#define TIMER_DMA_CH2D                      ((uint16_t)TIMER_DMAINTEN_CH2DEN)       /*!< channel 2 DMA enable */
#define TIMER_DMA_CH3D                      ((uint16_t)TIMER_DMAINTEN_CH3DEN)       /*!< channel 3 DMA enable */
#define TIMER_DMA_CMTD                      ((uint16_t)TIMER_DMAINTEN_CMTDEN)       /*!< commutation DMA request enable */
#define TIMER_DMA_TRGD                      ((uint16_t)TIMER_DMAINTEN_TRGDEN)       /*!< trigger DMA enable */

/* channel DMA request source selection */ 
#define TIMER_DMAREQUEST_UPDATEEVENT        TIMER_CTL1_DMAS                         /*!< DMA request of channel n is sent when update event occurs */
#define TIMER_DMAREQUEST_CHANNELEVENT       ((uint32_t)0x00000000U)                 /*!< DMA request of channel n is sent when channel n event occurs */

/* DMA access base address */
#define DMACFG_DMATA(regval)                (BITS(0, 4) & ((uint32_t)(regval) << 0U))
#define TIMER_DMACFG_DMATA_CTL0             DMACFG_DMATA(0)                         /*!< DMA transfer address is TIMER_CTL0 */
#define TIMER_DMACFG_DMATA_CTL1             DMACFG_DMATA(1)                         /*!< DMA transfer address is TIMER_CTL1 */
#define TIMER_DMACFG_DMATA_SMCFG            DMACFG_DMATA(2)                         /*!< DMA transfer address is TIMER_SMCFG */
#define TIMER_DMACFG_DMATA_DMAINTEN         DMACFG_DMATA(3)                         /*!< DMA transfer address is TIMER_DMAINTEN */
#define TIMER_DMACFG_DMATA_INTF             DMACFG_DMATA(4)                         /*!< DMA transfer address is TIMER_INTF */
#define TIMER_DMACFG_DMATA_SWEVG            DMACFG_DMATA(5)                         /*!< DMA transfer address is TIMER_SWEVG */
#define TIMER_DMACFG_DMATA_CHCTL0           DMACFG_DMATA(6)                         /*!< DMA transfer address is TIMER_CHCTL0 */
#define TIMER_DMACFG_DMATA_CHCTL1           DMACFG_DMATA(7)                         /*!< DMA transfer address is TIMER_CHCTL1 */
#define TIMER_DMACFG_DMATA_CHCTL2           DMACFG_DMATA(8)                         /*!< DMA transfer address is TIMER_CHCTL2 */
#define TIMER_DMACFG_DMATA_CNT              DMACFG_DMATA(9)                         /*!< DMA transfer address is TIMER_CNT */
#define TIMER_DMACFG_DMATA_PSC              DMACFG_DMATA(10)                        /*!< DMA transfer address is TIMER_PSC */
#define TIMER_DMACFG_DMATA_CAR              DMACFG_DMATA(11)                        /*!< DMA transfer address is TIMER_CAR */
#define TIMER_DMACFG_DMATA_CREP             DMACFG_DMATA(12)                        /*!< DMA transfer address is TIMER_CREP */
#define TIMER_DMACFG_DMATA_CH0CV            DMACFG_DMATA(13)                        /*!< DMA transfer address is TIMER_CH0CV */
#define TIMER_DMACFG_DMATA_CH1CV            DMACFG_DMATA(14)                        /*!< DMA transfer address is TIMER_CH1CV */
#define TIMER_DMACFG_DMATA_CH2CV            DMACFG_DMATA(15)                        /*!< DMA transfer address is TIMER_CH2CV */
#define TIMER_DMACFG_DMATA_CH3CV            DMACFG_DMATA(16)                        /*!< DMA transfer address is TIMER_CH3CV */
#define TIMER_DMACFG_DMATA_CCHP             DMACFG_DMATA(17)                        /*!< DMA transfer address is TIMER_CCHP */
#define TIMER_DMACFG_DMATA_DMACFG           DMACFG_DMATA(18)                        /*!< DMA transfer address is TIMER_DMACFG */

/* DMA access burst length */
#define DMACFG_DMATC(regval)                (BITS(8, 12) & ((uint32_t)(regval) << 8U))
#define TIMER_DMACFG_DMATC_1TRANSFER        DMACFG_DMATC(0)                         /*!< DMA transfer 1 time */
#define TIMER_DMACFG_DMATC_2TRANSFER        DMACFG_DMATC(1)                         /*!< DMA transfer 2 times */
#define TIMER_DMACFG_DMATC_3TRANSFER        DMACFG_DMATC(2)                         /*!< DMA transfer 3 times */
#define TIMER_DMACFG_DMATC_4TRANSFER        DMACFG_DMATC(3)                         /*!< DMA transfer 4 times */
#define TIMER_DMACFG_DMATC_5TRANSFER        DMACFG_DMATC(4)                         /*!< DMA transfer 5 times */
#define TIMER_DMACFG_DMATC_6TRANSFER        DMACFG_DMATC(5)                         /*!< DMA transfer 6 times */
#define TIMER_DMACFG_DMATC_7TRANSFER        DMACFG_DMATC(6)                         /*!< DMA transfer 7 times */
#define TIMER_DMACFG_DMATC_8TRANSFER        DMACFG_DMATC(7)                         /*!< DMA transfer 8 times */
#define TIMER_DMACFG_DMATC_9TRANSFER        DMACFG_DMATC(8)                         /*!< DMA transfer 9 times */
#define TIMER_DMACFG_DMATC_10TRANSFER       DMACFG_DMATC(9)                         /*!< DMA transfer 10 times */
#define TIMER_DMACFG_DMATC_11TRANSFER       DMACFG_DMATC(10)                        /*!< DMA transfer 11 times */
#define TIMER_DMACFG_DMATC_12TRANSFER       DMACFG_DMATC(11)                        /*!< DMA transfer 12 times */
#define TIMER_DMACFG_DMATC_13TRANSFER       DMACFG_DMATC(12)                        /*!< DMA transfer 13 times */
#define TIMER_DMACFG_DMATC_14TRANSFER       DMACFG_DMATC(13)                        /*!< DMA transfer 14 times */
#define TIMER_DMACFG_DMATC_15TRANSFER       DMACFG_DMATC(14)                        /*!< DMA transfer 15 times */
#define TIMER_DMACFG_DMATC_16TRANSFER       DMACFG_DMATC(15)                        /*!< DMA transfer 16 times */
#define TIMER_DMACFG_DMATC_17TRANSFER       DMACFG_DMATC(16)                        /*!< DMA transfer 17 times */
#define TIMER_DMACFG_DMATC_18TRANSFER       DMACFG_DMATC(17)                        /*!< DMA transfer 18 times */

/* TIMER software event generation source */
#define TIMER_EVENT_SRC_UPG                 ((uint16_t)0x0001U)                     /*!< update event generation */
#define TIMER_EVENT_SRC_CH0G                ((uint16_t)0x0002U)                     /*!< channel 0 capture or compare event generation */
#define TIMER_EVENT_SRC_CH1G                ((uint16_t)0x0004U)                     /*!< channel 1 capture or compare event generation */
#define TIMER_EVENT_SRC_CH2G                ((uint16_t)0x0008U)                     /*!< channel 2 capture or compare event generation */
#define TIMER_EVENT_SRC_CH3G                ((uint16_t)0x0010U)                     /*!< channel 3 capture or compare event generation */
#define TIMER_EVENT_SRC_CMTG                ((uint16_t)0x0020U)                     /*!< channel commutation event generation */
#define TIMER_EVENT_SRC_TRGG                ((uint16_t)0x0040U)                     /*!< trigger event generation */
#define TIMER_EVENT_SRC_BRKG                ((uint16_t)0x0080U)                     /*!< break event generation */

/* center-aligned mode selection */
#define CTL0_CAM(regval)                    ((uint16_t)(BITS(5, 6) & ((uint32_t)(regval) << 5U)))
#define TIMER_COUNTER_EDGE                  CTL0_CAM(0)                             /*!< edge-aligned mode */
#define TIMER_COUNTER_CENTER_DOWN           CTL0_CAM(1)                             /*!< center-aligned and counting down assert mode */
#define TIMER_COUNTER_CENTER_UP             CTL0_CAM(2)                             /*!< center-aligned and counting up assert mode */
#define TIMER_COUNTER_CENTER_BOTH           CTL0_CAM(3)                             /*!< center-aligned and counting up/down assert mode */

/* TIMER prescaler reload mode */
#define TIMER_PSC_RELOAD_NOW                TIMER_SWEVG_UPG                         /*!< the prescaler is loaded right now */
#define TIMER_PSC_RELOAD_UPDATE             ((uint32_t)0x00000000U)                 /*!< the prescaler is loaded at the next update event */

/* count direction */
#define TIMER_COUNTER_UP                    ((uint16_t)0x0000U)                     /*!< counter up direction */
#define TIMER_COUNTER_DOWN                  ((uint16_t)TIMER_CTL0_DIR)              /*!< counter down direction */

/* specify division ratio between TIMER clock and dead-time and sampling clock */
#define CTL0_CKDIV(regval)                  ((uint16_t)(BITS(8, 9) & ((uint32_t)(regval) << 8U)))
#define TIMER_CKDIV_DIV1                    CTL0_CKDIV(0)                           /*!< clock division value is 1,fDTS=fTIMER_CK */
#define TIMER_CKDIV_DIV2                    CTL0_CKDIV(1)                           /*!< clock division value is 2,fDTS= fTIMER_CK/2 */
#define TIMER_CKDIV_DIV4                    CTL0_CKDIV(2)                           /*!< clock division value is 4, fDTS= fTIMER_CK/4 */

/* single pulse mode */
#define TIMER_SP_MODE_SINGLE                TIMER_CTL0_SPM                          /*!< single pulse mode */
#define TIMER_SP_MODE_REPETITIVE            ((uint32_t)0x00000000U)                 /*!< repetitive pulse mode */

/* update source */
#define TIMER_UPDATE_SRC_REGULAR            TIMER_CTL0_UPS                          /*!< update generate only by counter overflow/underflow */
#define TIMER_UPDATE_SRC_GLOBAL             ((uint32_t)0x00000000U)                 /*!< update generate by setting of UPG bit or the counter overflow/underflow,or the slave mode controller trigger */

/* run mode off-state configure */
#define TIMER_ROS_STATE_ENABLE              ((uint16_t)TIMER_CCHP_ROS)              /*!< when POEN bit is set, the channel output signals(CHx_O/CHx_ON) are enabled, with relationship to CHxEN/CHxNEN bits */
#define TIMER_ROS_STATE_DISABLE             ((uint16_t)0x0000U)                     /*!< when POEN bit is set, the channel output signals(CHx_O/CHx_ON) are disabled */


/* idle mode off-state configure */                                                 
#define TIMER_IOS_STATE_ENABLE              ((uint16_t)TIMER_CCHP_IOS)              /*!< when POEN bit is reset, he channel output signals(CHx_O/CHx_ON) are enabled, with relationship to CHxEN/CHxNEN bits */
#define TIMER_IOS_STATE_DISABLE             ((uint16_t)0x0000U)                     /*!< when POEN bit is reset, the channel output signals(CHx_O/CHx_ON) are disabled */

/* break input polarity */
#define TIMER_BREAK_POLARITY_LOW            ((uint16_t)0x0000U)                     /*!< break input polarity is low */
#define TIMER_BREAK_POLARITY_HIGH           ((uint16_t)TIMER_CCHP_BRKP)             /*!< break input polarity is high */

/* output automatic enable */
#define TIMER_OUTAUTO_ENABLE                ((uint16_t)TIMER_CCHP_OAEN)             /*!< output automatic enable */
#define TIMER_OUTAUTO_DISABLE               ((uint16_t)0x0000U)                     /*!< output automatic disable */

/* complementary register protect control */
#define CCHP_PROT(regval)                   ((uint16_t)(BITS(8, 9) & ((uint32_t)(regval) << 8U)))
#define TIMER_CCHP_PROT_OFF                 CCHP_PROT(0)                            /*!< protect disable */
#define TIMER_CCHP_PROT_0                   CCHP_PROT(1)                            /*!< PROT mode 0 */
#define TIMER_CCHP_PROT_1                   CCHP_PROT(2)                            /*!< PROT mode 1 */
#define TIMER_CCHP_PROT_2                   CCHP_PROT(3)                            /*!< PROT mode 2 */

/* break input enable */
#define TIMER_BREAK_ENABLE                  ((uint16_t)TIMER_CCHP_BRKEN)            /*!< break input enable */
#define TIMER_BREAK_DISABLE                 ((uint16_t)0x0000U)                     /*!< break input disable */

/* TIMER channel n(n=0,1,2,3) */
#define TIMER_CH_0                          ((uint16_t)0x0000U)                     /*!< TIMER channel 0(TIMERx(x=0..4)) */
#define TIMER_CH_1                          ((uint16_t)0x0001U)                     /*!< TIMER channel 1(TIMERx(x=0..4)) */
#define TIMER_CH_2                          ((uint16_t)0x0002U)                     /*!< TIMER channel 2(TIMERx(x=0..4)) */
#define TIMER_CH_3                          ((uint16_t)0x0003U)                     /*!< TIMER channel 3(TIMERx(x=0..4)) */

/* channel enable state */
#define TIMER_CCX_ENABLE                    ((uint16_t)0x0001U)                     /*!< channel enable */
#define TIMER_CCX_DISABLE                   ((uint16_t)0x0000U)                     /*!< channel disable */

/* channel complementary output enable state */
#define TIMER_CCXN_ENABLE                   ((uint16_t)0x0004U)                     /*!< channel complementary enable */
#define TIMER_CCXN_DISABLE                  ((uint16_t)0x0000U)                     /*!< channel complementary disable */

/* channel output polarity */
#define TIMER_OC_POLARITY_HIGH              ((uint16_t)0x0000U)                     /*!< channel output polarity is high */
#define TIMER_OC_POLARITY_LOW               ((uint16_t)0x0002U)                     /*!< channel output polarity is low */

/* channel complementary output polarity */
#define TIMER_OCN_POLARITY_HIGH             ((uint16_t)0x0000U)                     /*!< channel complementary output polarity is high */
#define TIMER_OCN_POLARITY_LOW              ((uint16_t)0x0008U)                     /*!< channel complementary output polarity is low */

/* idle state of channel output */ 
#define TIMER_OC_IDLE_STATE_HIGH            ((uint16_t)0x0100)                      /*!< idle state of channel output is high */
#define TIMER_OC_IDLE_STATE_LOW             ((uint16_t)0x0000)                      /*!< idle state of channel output is low */

/* idle state of channel complementary output */ 
#define TIMER_OCN_IDLE_STATE_HIGH           ((uint16_t)0x0200U)                     /*!< idle state of channel complementary output is high */
#define TIMER_OCN_IDLE_STATE_LOW            ((uint16_t)0x0000U)                     /*!< idle state of channel complementary output is low */

/* channel output compare mode */
#define TIMER_OC_MODE_TIMING                ((uint16_t)0x0000U)                     /*!< timing mode */
#define TIMER_OC_MODE_ACTIVE                ((uint16_t)0x0010U)                     /*!< active mode */
#define TIMER_OC_MODE_INACTIVE              ((uint16_t)0x0020U)                     /*!< inactive mode */
#define TIMER_OC_MODE_TOGGLE                ((uint16_t)0x0030U)                     /*!< toggle mode */
#define TIMER_OC_MODE_LOW                   ((uint16_t)0x0040U)                     /*!< force low mode */
#define TIMER_OC_MODE_HIGH                  ((uint16_t)0x0050U)                     /*!< force high mode */
#define TIMER_OC_MODE_PWM0                  ((uint16_t)0x0060U)                     /*!< PWM0 mode */
#define TIMER_OC_MODE_PWM1                  ((uint16_t)0x0070U)                     /*!< PWM1 mode */

/* channel output compare shadow enable */
#define TIMER_OC_SHADOW_ENABLE              ((uint16_t)0x0008U)                     /*!< channel output shadow state enable */
#define TIMER_OC_SHADOW_DISABLE             ((uint16_t)0x0000U)                     /*!< channel output shadow state disable */

/* channel output compare fast enable */
#define TIMER_OC_FAST_ENABLE                ((uint16_t)0x0004)                      /*!< channel output fast function enable */
#define TIMER_OC_FAST_DISABLE               ((uint16_t)0x0000)                      /*!< channel output fast function disable */

/* channel output compare clear enable */
#define TIMER_OC_CLEAR_ENABLE               ((uint16_t)0x0080U)                     /*!< channel output clear function enable */
#define TIMER_OC_CLEAR_DISABLE              ((uint16_t)0x0000U)                     /*!< channel output clear function disable */

/* channel control shadow register update control */ 
#define TIMER_UPDATECTL_CCU                 ((uint32_t)0x00000000U)                 /*!< the shadow registers update by when CMTG bit is set */
#define TIMER_UPDATECTL_CCUTRI              TIMER_CTL1_CCUC                         /*!< the shadow registers update by when CMTG bit is set or an rising edge of TRGI occurs */

/* channel input capture polarity */
#define TIMER_IC_POLARITY_RISING            ((uint16_t)0x0000U)                     /*!< input capture rising edge */
#define TIMER_IC_POLARITY_FALLING           ((uint16_t)0x0002U)                     /*!< input capture falling edge */
#define TIMER_IC_POLARITY_BOTH_EDGE         ((uint16_t)0x000AU)                     /*!< input capture both edge */

/* TIMER input capture selection */
#define TIMER_IC_SELECTION_DIRECTTI         ((uint16_t)0x0001U)                     /*!< channel n is configured as input and icy is mapped on CIy */
#define TIMER_IC_SELECTION_INDIRECTTI       ((uint16_t)0x0002U)                     /*!< channel n is configured as input and icy is mapped on opposite input */
#define TIMER_IC_SELECTION_ITS              ((uint16_t)0x0003U)                     /*!< channel n is configured as input and icy is mapped on ITS */

/* channel input capture prescaler */
#define TIMER_IC_PSC_DIV1                   ((uint16_t)0x0000U)                     /*!< no prescaler */
#define TIMER_IC_PSC_DIV2                   ((uint16_t)0x0004U)                     /*!< divided by 2 */
#define TIMER_IC_PSC_DIV4                   ((uint16_t)0x0008U)                     /*!< divided by 4 */
#define TIMER_IC_PSC_DIV8                   ((uint16_t)0x000CU)                     /*!< divided by 8 */

/* trigger selection */
#define SMCFG_TRGSEL(regval)                (BITS(4, 6) & ((uint32_t)(regval) << 4U))
#define TIMER_SMCFG_TRGSEL_ITI0             SMCFG_TRGSEL(0)                         /*!< internal trigger 0 */
#define TIMER_SMCFG_TRGSEL_ITI1             SMCFG_TRGSEL(1)                         /*!< internal trigger 1 */
#define TIMER_SMCFG_TRGSEL_ITI2             SMCFG_TRGSEL(2)                         /*!< internal trigger 2 */
#define TIMER_SMCFG_TRGSEL_ITI3             SMCFG_TRGSEL(3)                         /*!< internal trigger 3 */
#define TIMER_SMCFG_TRGSEL_CI0F_ED          SMCFG_TRGSEL(4)                         /*!< TI0 Edge Detector */
#define TIMER_SMCFG_TRGSEL_CI0FE0           SMCFG_TRGSEL(5)                         /*!< filtered TIMER input 0 */
#define TIMER_SMCFG_TRGSEL_CI1FE1           SMCFG_TRGSEL(6)                         /*!< filtered TIMER input 1 */
#define TIMER_SMCFG_TRGSEL_ETIFP            SMCFG_TRGSEL(7)                         /*!< filtered external trigger input */

/* master mode control */
#define CTL1_MMC(regval)                    (BITS(4, 6) & ((uint32_t)(regval) << 4U))
#define TIMER_TRI_OUT_SRC_RESET             CTL1_MMC(0)                             /*!< the UPG bit as trigger output */
#define TIMER_TRI_OUT_SRC_ENABLE            CTL1_MMC(1)                             /*!< the counter enable signal TIMER_CTL0_CEN as trigger output */
#define TIMER_TRI_OUT_SRC_UPDATE            CTL1_MMC(2)                             /*!< update event as trigger output */
#define TIMER_TRI_OUT_SRC_CH0               CTL1_MMC(3)                             /*!< a capture or a compare match occurred in channel 0 as trigger output TRGO */
#define TIMER_TRI_OUT_SRC_O0CPRE            CTL1_MMC(4)                             /*!< O0CPRE as trigger output */
#define TIMER_TRI_OUT_SRC_O1CPRE            CTL1_MMC(5)                             /*!< O1CPRE as trigger output */
#define TIMER_TRI_OUT_SRC_O2CPRE            CTL1_MMC(6)                             /*!< O2CPRE as trigger output */
#define TIMER_TRI_OUT_SRC_O3CPRE            CTL1_MMC(7)                             /*!< O3CPRE as trigger output */

/* slave mode control */
#define SMCFG_SMC(regval)                   (BITS(0, 2) & ((uint32_t)(regval) << 0U)) 
#define TIMER_SLAVE_MODE_DISABLE            SMCFG_SMC(0)                            /*!< slave mode disable */
#define TIMER_ENCODER_MODE0                 SMCFG_SMC(1)                            /*!< encoder mode 0 */
#define TIMER_ENCODER_MODE1                 SMCFG_SMC(2)                            /*!< encoder mode 1 */
#define TIMER_ENCODER_MODE2                 SMCFG_SMC(3)                            /*!< encoder mode 2 */
#define TIMER_SLAVE_MODE_RESTART            SMCFG_SMC(4)                            /*!< restart mode */
#define TIMER_SLAVE_MODE_PAUSE              SMCFG_SMC(5)                            /*!< pause mode */
#define TIMER_SLAVE_MODE_EVENT              SMCFG_SMC(6)                            /*!< event mode */
#define TIMER_SLAVE_MODE_EXTERNAL0          SMCFG_SMC(7)                            /*!< external clock mode 0 */

/* master slave mode selection */ 
#define TIMER_MASTER_SLAVE_MODE_ENABLE      TIMER_SMCFG_MSM                         /*!< master slave mode enable */
#define TIMER_MASTER_SLAVE_MODE_DISABLE     ((uint32_t)0x00000000U)                 /*!< master slave mode disable */

/* external trigger prescaler */
#define SMCFG_ETPSC(regval)                 (BITS(12, 13) & ((uint32_t)(regval) << 12U))
#define TIMER_EXT_TRI_PSC_OFF               SMCFG_ETPSC(0)                          /*!< no divided */
#define TIMER_EXT_TRI_PSC_DIV2              SMCFG_ETPSC(1)                          /*!< divided by 2 */
#define TIMER_EXT_TRI_PSC_DIV4              SMCFG_ETPSC(2)                          /*!< divided by 4 */
#define TIMER_EXT_TRI_PSC_DIV8              SMCFG_ETPSC(3)                          /*!< divided by 8 */

/* external trigger polarity */
#define TIMER_ETP_FALLING                   TIMER_SMCFG_ETP                         /*!< active low or falling edge active */
#define TIMER_ETP_RISING                    ((uint32_t)0x00000000U)                 /*!< active high or rising edge active */

/* channel 0 trigger input selection */ 
#define TIMER_HALLINTERFACE_ENABLE          TIMER_CTL1_TI0S                         /*!< TIMER hall sensor mode enable */
#define TIMER_HALLINTERFACE_DISABLE         ((uint32_t)0x00000000U)                 /*!< TIMER hall sensor mode disable */

/* TIMERx(x=0..4) write CHxVAL register selection */
#define TIMER_CHVSEL_ENABLE                 ((uint16_t)TIMER_CFG_OUTSEL)            /*!< write CHxVAL register selection enable */
#define TIMER_CHVSEL_DISABLE                ((uint16_t)0x0000U)                     /*!< write CHxVAL register selection disable */


////////////////////////////////////////////////////////////////////////
/* USARTx(x=0,1,2)/UARTx(x=3,4) definitions */
#define USART1                        USART_BASE                        /*!< USART1 base address */
#define USART2                        (USART_BASE+(0x00000400U))        /*!< USART2 base address */
#define UART3                         (USART_BASE+(0x00000800U))        /*!< UART3 base address */
#define UART4                         (USART_BASE+(0x00000C00U))        /*!< UART4 base address */
#define USART0                        (USART_BASE+(0x0000F400U))        /*!< USART0 base address */

/* registers definitions */
#define USART_STAT(usartx)            REG32((usartx) + (0x00000000U))   /*!< USART status register */
#define USART_DATA(usartx)            REG32((usartx) + (0x00000004U))   /*!< USART data register */
#define USART_BAUD(usartx)            REG32((usartx) + (0x00000008U))   /*!< USART baud rate register */
#define USART_CTL0(usartx)            REG32((usartx) + (0x0000000CU))   /*!< USART control register 0 */
#define USART_CTL1(usartx)            REG32((usartx) + (0x00000010U))   /*!< USART control register 1 */
#define USART_CTL2(usartx)            REG32((usartx) + (0x00000014U))   /*!< USART control register 2 */
#define USART_GP(usartx)              REG32((usartx) + (0x00000018U))   /*!< USART guard time and prescaler register */

/* bits definitions */
/* USARTx_STAT */
#define USART_STAT_PERR               BIT(0)                            /*!< parity error flag */
#define USART_STAT_FERR               BIT(1)                            /*!< frame error flag */
#define USART_STAT_NERR               BIT(2)                            /*!< noise error flag */
#define USART_STAT_ORERR              BIT(3)                            /*!< overrun error */
#define USART_STAT_IDLEF              BIT(4)                            /*!< IDLE frame detected flag */
#define USART_STAT_RBNE               BIT(5)                            /*!< read data buffer not empty */
#define USART_STAT_TC                 BIT(6)                            /*!< transmission complete */
#define USART_STAT_TBE                BIT(7)                            /*!< transmit data buffer empty */
#define USART_STAT_LBDF               BIT(8)                            /*!< LIN break detected flag */
#define USART_STAT_CTSF               BIT(9)                            /*!< CTS change flag */

/* USARTx_DATA */
#define USART_DATA_DATA               BITS(0,8)                         /*!< transmit or read data value */

/* USARTx_BAUD */
#define USART_BAUD_FRADIV             BITS(0,3)                         /*!< fraction part of baud-rate divider */
#define USART_BAUD_INTDIV             BITS(4,15)                        /*!< integer part of baud-rate divider */

/* USARTx_CTL0 */
#define USART_CTL0_SBKCMD             BIT(0)                            /*!< send break command */
#define USART_CTL0_RWU                BIT(1)                            /*!< receiver wakeup from mute mode */
#define USART_CTL0_REN                BIT(2)                            /*!< receiver enable */
#define USART_CTL0_TEN                BIT(3)                            /*!< transmitter enable */
#define USART_CTL0_IDLEIE             BIT(4)                            /*!< idle line detected interrupt enable */
#define USART_CTL0_RBNEIE             BIT(5)                            /*!< read data buffer not empty interrupt and overrun error interrupt enable */
#define USART_CTL0_TCIE               BIT(6)                            /*!< transmission complete interrupt enable */
#define USART_CTL0_TBEIE              BIT(7)                            /*!< transmitter buffer empty interrupt enable */
#define USART_CTL0_PERRIE             BIT(8)                            /*!< parity error interrupt enable */
#define USART_CTL0_PM                 BIT(9)                            /*!< parity mode */
#define USART_CTL0_PCEN               BIT(10)                           /*!< parity check function enable */
#define USART_CTL0_WM                 BIT(11)                           /*!< wakeup method in mute mode */
#define USART_CTL0_WL                 BIT(12)                           /*!< word length */
#define USART_CTL0_UEN                BIT(13)                           /*!< USART enable */

/* USARTx_CTL1 */
#define USART_CTL1_ADDR               BITS(0,3)                         /*!< address of USART */
#define USART_CTL1_LBLEN              BIT(5)                            /*!< LIN break frame length */
#define USART_CTL1_LBDIE              BIT(6)                            /*!< LIN break detected interrupt eanble */
#define USART_CTL1_CLEN               BIT(8)                            /*!< CK length */
#define USART_CTL1_CPH                BIT(9)                            /*!< CK phase */
#define USART_CTL1_CPL                BIT(10)                           /*!< CK polarity */
#define USART_CTL1_CKEN               BIT(11)                           /*!< CK pin enable */
#define USART_CTL1_STB                BITS(12,13)                       /*!< STOP bits length */
#define USART_CTL1_LMEN               BIT(14)                           /*!< LIN mode enable */

/* USARTx_CTL2 */
#define USART_CTL2_ERRIE              BIT(0)                            /*!< error interrupt enable */
#define USART_CTL2_IREN               BIT(1)                            /*!< IrDA mode enable */
#define USART_CTL2_IRLP               BIT(2)                            /*!< IrDA low-power */
#define USART_CTL2_HDEN               BIT(3)                            /*!< half-duplex enable */
#define USART_CTL2_NKEN               BIT(4)                            /*!< NACK enable in smartcard mode */
#define USART_CTL2_SCEN               BIT(5)                            /*!< smartcard mode enable */
#define USART_CTL2_DENR               BIT(6)                            /*!< DMA request enable for reception */
#define USART_CTL2_DENT               BIT(7)                            /*!< DMA request enable for transmission */
#define USART_CTL2_RTSEN              BIT(8)                            /*!< RTS enable */
#define USART_CTL2_CTSEN              BIT(9)                            /*!< CTS enable */
#define USART_CTL2_CTSIE              BIT(10)                           /*!< CTS interrupt enable */

/* USARTx_GP */
#define USART_GP_PSC                  BITS(0,7)                         /*!< prescaler value for dividing the system clock */
#define USART_GP_GUAT                 BITS(8,15)                        /*!< guard time value in smartcard mode */

/* constants definitions */
/* define the USART bit position and its register index offset */
#define USART_REGIDX_BIT(regidx, bitpos)     (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define USART_REG_VAL(usartx, offset)        (REG32((usartx) + (((uint32_t)(offset) & (0x0000FFFFU)) >> 6)))
#define USART_BIT_POS(val)                   ((uint32_t)(val) & (0x0000001FU))
#define USART_REGIDX_BIT2(regidx, bitpos, regidx2, bitpos2)   (((uint32_t)(regidx2) << 22) | (uint32_t)((bitpos2) << 16)\
                                                              | (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos)))
#define USART_REG_VAL2(usartx, offset)       (REG32((usartx) + ((uint32_t)(offset) >> 22)))
#define USART_BIT_POS2(val)                  (((uint32_t)(val) & (0x001F0000U)) >> 16)

/* register offset */
#define USART_STAT_REG_OFFSET                     (0x00000000U)        	/*!< STAT register offset */
#define USART_CTL0_REG_OFFSET                     (0x0000000CU)        	/*!< CTL0 register offset */
#define USART_CTL1_REG_OFFSET                     (0x00000010U)        	/*!< CTL1 register offset */
#define USART_CTL2_REG_OFFSET                     (0x00000014U)        	/*!< CTL2 register offset */

/* USART flags */
typedef enum
{
    /* flags in STAT register */
    USART_FLAG_CTS  = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 9U),      /*!< CTS change flag */
    USART_FLAG_LBD  = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 8U),      /*!< LIN break detected flag */
    USART_FLAG_TBE = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 7U),       /*!< transmit data buffer empty */
    USART_FLAG_TC = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 6U),        /*!< transmission complete */
    USART_FLAG_RBNE = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 5U),      /*!< read data buffer not empty */
    USART_FLAG_IDLE  = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 4U),     /*!< IDLE frame detected flag */
    USART_FLAG_ORERR = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 3U),     /*!< overrun error */
    USART_FLAG_NERR = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 2U),      /*!< noise error flag */
    USART_FLAG_FERR = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 1U),      /*!< frame error flag */
    USART_FLAG_PERR = USART_REGIDX_BIT(USART_STAT_REG_OFFSET, 0U),      /*!< parity error flag */
}usart_flag_enum;

/* USART interrupt flags */
typedef enum
{
    /* interrupt flags in CTL0 register */
    USART_INT_FLAG_PERR = USART_REGIDX_BIT2(USART_CTL0_REG_OFFSET, 8U, USART_STAT_REG_OFFSET, 0U),       /*!< parity error interrupt and flag */
    USART_INT_FLAG_TBE = USART_REGIDX_BIT2(USART_CTL0_REG_OFFSET, 7U, USART_STAT_REG_OFFSET, 7U),        /*!< transmitter buffer empty interrupt and flag */
    USART_INT_FLAG_TC = USART_REGIDX_BIT2(USART_CTL0_REG_OFFSET, 6U, USART_STAT_REG_OFFSET, 6U),         /*!< transmission complete interrupt and flag */
    USART_INT_FLAG_RBNE = USART_REGIDX_BIT2(USART_CTL0_REG_OFFSET, 5U, USART_STAT_REG_OFFSET, 5U),       /*!< read data buffer not empty interrupt and flag */
    USART_INT_FLAG_RBNE_ORERR = USART_REGIDX_BIT2(USART_CTL0_REG_OFFSET, 5U, USART_STAT_REG_OFFSET, 3U), /*!< read data buffer not empty interrupt and overrun error flag */
    USART_INT_FLAG_IDLE = USART_REGIDX_BIT2(USART_CTL0_REG_OFFSET, 4U, USART_STAT_REG_OFFSET, 4U),       /*!< IDLE line detected interrupt and flag */
    /* interrupt flags in CTL1 register */
    USART_INT_FLAG_LBD = USART_REGIDX_BIT2(USART_CTL1_REG_OFFSET, 6U, USART_STAT_REG_OFFSET, 8U),        /*!< LIN break detected interrupt and flag */
    /* interrupt flags in CTL2 register */
    USART_INT_FLAG_CTS = USART_REGIDX_BIT2(USART_CTL2_REG_OFFSET, 10U, USART_STAT_REG_OFFSET, 9U),       /*!< CTS interrupt and flag */
    USART_INT_FLAG_ERR_ORERR = USART_REGIDX_BIT2(USART_CTL2_REG_OFFSET, 0U, USART_STAT_REG_OFFSET, 3U),  /*!< error interrupt and overrun error */
    USART_INT_FLAG_ERR_NERR = USART_REGIDX_BIT2(USART_CTL2_REG_OFFSET, 0U, USART_STAT_REG_OFFSET, 2U),   /*!< error interrupt and noise error flag */
    USART_INT_FLAG_ERR_FERR = USART_REGIDX_BIT2(USART_CTL2_REG_OFFSET, 0U, USART_STAT_REG_OFFSET, 1U),   /*!< error interrupt and frame error flag */
}usart_interrupt_flag_enum;

/* USART interrupt enable or disable */
typedef enum
{
    /* interrupt in CTL0 register */
    USART_INT_PERR = USART_REGIDX_BIT(USART_CTL0_REG_OFFSET, 8U),       /*!< parity error interrupt */
    USART_INT_TBE = USART_REGIDX_BIT(USART_CTL0_REG_OFFSET, 7U),        /*!< transmitter buffer empty interrupt */
    USART_INT_TC = USART_REGIDX_BIT(USART_CTL0_REG_OFFSET, 6U),         /*!< transmission complete interrupt */
    USART_INT_RBNE = USART_REGIDX_BIT(USART_CTL0_REG_OFFSET, 5U),       /*!< read data buffer not empty interrupt and overrun error interrupt */
    USART_INT_IDLE = USART_REGIDX_BIT(USART_CTL0_REG_OFFSET, 4U),       /*!< IDLE line detected interrupt */
    /* interrupt in CTL1 register */
    USART_INT_LBD = USART_REGIDX_BIT(USART_CTL1_REG_OFFSET, 6U),        /*!< LIN break detected interrupt */
    /* interrupt in CTL2 register */
    USART_INT_CTS = USART_REGIDX_BIT(USART_CTL2_REG_OFFSET, 10U),       /*!< CTS interrupt */
    USART_INT_ERR = USART_REGIDX_BIT(USART_CTL2_REG_OFFSET, 0U),        /*!< error interrupt */
}usart_interrupt_enum;

/* USART receiver configure */
#define CTL0_REN(regval)              (BIT(2) & ((uint32_t)(regval) << 2))
#define USART_RECEIVE_ENABLE          CTL0_REN(1)                       /*!< enable receiver */
#define USART_RECEIVE_DISABLE         CTL0_REN(0)                       /*!< disable receiver */

/* USART transmitter configure */
#define CTL0_TEN(regval)              (BIT(3) & ((uint32_t)(regval) << 3))
#define USART_TRANSMIT_ENABLE         CTL0_TEN(1)                       /*!< enable transmitter */
#define USART_TRANSMIT_DISABLE        CTL0_TEN(0)                       /*!< disable transmitter */

/* USART parity bits definitions */
#define CTL0_PM(regval)               (BITS(9,10) & ((uint32_t)(regval) << 9))
#define USART_PM_NONE                 CTL0_PM(0)                        /*!< no parity */
#define USART_PM_EVEN                 CTL0_PM(2)                        /*!< even parity */
#define USART_PM_ODD                  CTL0_PM(3)                        /*!< odd parity */

/* USART wakeup method in mute mode */
#define CTL0_WM(regval)               (BIT(11) & ((uint32_t)(regval) << 11))
#define USART_WM_IDLE                 CTL0_WM(0)                        /*!< idle line */
#define USART_WM_ADDR                 CTL0_WM(1)                        /*!< address match */

/* USART word length definitions */
#define CTL0_WL(regval)               (BIT(12) & ((uint32_t)(regval) << 12))
#define USART_WL_8BIT                 CTL0_WL(0)                        /*!< 8 bits */
#define USART_WL_9BIT                 CTL0_WL(1)                        /*!< 9 bits */

/* USART stop bits definitions */
#define CTL1_STB(regval)              (BITS(12,13) & ((uint32_t)(regval) << 12))
#define USART_STB_1BIT                CTL1_STB(0)                       /*!< 1 bit */
#define USART_STB_0_5BIT              CTL1_STB(1)                       /*!< 0.5 bit */
#define USART_STB_2BIT                CTL1_STB(2)                       /*!< 2 bits */
#define USART_STB_1_5BIT              CTL1_STB(3)                       /*!< 1.5 bits */

/* USART LIN break frame length */
#define CTL1_LBLEN(regval)            (BIT(5) & ((uint32_t)(regval) << 5))
#define USART_LBLEN_10B               CTL1_LBLEN(0)                     /*!< 10 bits */
#define USART_LBLEN_11B               CTL1_LBLEN(1)                     /*!< 11 bits */

/* USART CK length */
#define CTL1_CLEN(regval)             (BIT(8) & ((uint32_t)(regval) << 8))
#define USART_CLEN_NONE               CTL1_CLEN(0)                      /*!< there are 7 CK pulses for an 8 bit frame and 8 CK pulses for a 9 bit frame */
#define USART_CLEN_EN                 CTL1_CLEN(1)                      /*!< there are 8 CK pulses for an 8 bit frame and 9 CK pulses for a 9 bit frame */

/* USART clock phase */
#define CTL1_CPH(regval)              (BIT(9) & ((uint32_t)(regval) << 9))
#define USART_CPH_1CK                 CTL1_CPH(0)                       /*!< first clock transition is the first data capture edge */
#define USART_CPH_2CK                 CTL1_CPH(1)                       /*!< second clock transition is the first data capture edge */

/* USART clock polarity */
#define CTL1_CPL(regval)              (BIT(10) & ((uint32_t)(regval) << 10))
#define USART_CPL_LOW                 CTL1_CPL(0)                       /*!< steady low value on CK pin */
#define USART_CPL_HIGH                CTL1_CPL(1)                       /*!< steady high value on CK pin */

/* USART DMA request for receive configure */
#define CLT2_DENR(regval)             (BIT(6) & ((uint32_t)(regval) << 6))
#define USART_DENR_ENABLE             CLT2_DENR(1)                      /*!< DMA request enable for reception */
#define USART_DENR_DISABLE            CLT2_DENR(0)                      /*!< DMA request disable for reception */

/* USART DMA request for transmission configure */
#define CLT2_DENT(regval)             (BIT(7) & ((uint32_t)(regval) << 7))
#define USART_DENT_ENABLE             CLT2_DENT(1)                      /*!< DMA request enable for transmission */
#define USART_DENT_DISABLE            CLT2_DENT(0)                      /*!< DMA request disable for transmission */

/* USART RTS configure */
#define CLT2_RTSEN(regval)            (BIT(8) & ((uint32_t)(regval) << 8))
#define USART_RTS_ENABLE              CLT2_RTSEN(1)                     /*!< RTS enable */
#define USART_RTS_DISABLE             CLT2_RTSEN(0)                     /*!< RTS disable */

/* USART CTS configure */
#define CLT2_CTSEN(regval)            (BIT(9) & ((uint32_t)(regval) << 9))
#define USART_CTS_ENABLE              CLT2_CTSEN(1)                     /*!< CTS enable */
#define USART_CTS_DISABLE             CLT2_CTSEN(0)                     /*!< CTS disable */

/* USART IrDA low-power enable */
#define CTL2_IRLP(regval)             (BIT(2) & ((uint32_t)(regval) << 2))
#define USART_IRLP_LOW                CTL2_IRLP(1)                      /*!< low-power */
#define USART_IRLP_NORMAL             CTL2_IRLP(0)                      /*!< normal */


////////////////////////////////////////////////////////////////////////
/* WWDGT definitions */
#define WWDGT                       WWDGT_BASE                                /*!< WWDGT base address */

/* registers definitions */
#define WWDGT_CTL                   REG32((WWDGT) + 0x00000000U)              /*!< WWDGT control register */
#define WWDGT_CFG                   REG32((WWDGT) + 0x00000004U)              /*!< WWDGT configuration register */
#define WWDGT_STAT                  REG32((WWDGT) + 0x00000008U)              /*!< WWDGT status register */

/* bits definitions */
/* WWDGT_CTL */
#define WWDGT_CTL_CNT               BITS(0,6)                                 /*!< WWDGT counter value */
#define WWDGT_CTL_WDGTEN            BIT(7)                                    /*!< WWDGT counter enable */

/* WWDGT_CFG */
#define WWDGT_CFG_WIN               BITS(0,6)                                 /*!< WWDGT counter window value */
#define WWDGT_CFG_PSC               BITS(7,8)                                 /*!< WWDGT prescaler divider value */
#define WWDGT_CFG_EWIE              BIT(9)                                    /*!< early wakeup interrupt enable */

/* WWDGT_STAT */
#define WWDGT_STAT_EWIF             BIT(0)                                    /*!< early wakeup interrupt flag */

/* constants definitions */
#define CFG_PSC(regval)             (BITS(7,8) & ((uint32_t)(regval) << 7))   /*!< write value to WWDGT_CFG_PSC bit field */
#define WWDGT_CFG_PSC_DIV1          CFG_PSC(0)                                /*!< the time base of WWDGT = (PCLK1/4096)/1 */
#define WWDGT_CFG_PSC_DIV2          CFG_PSC(1)                                /*!< the time base of WWDGT = (PCLK1/4096)/2 */
#define WWDGT_CFG_PSC_DIV4          CFG_PSC(2)                                /*!< the time base of WWDGT = (PCLK1/4096)/4 */
#define WWDGT_CFG_PSC_DIV8          CFG_PSC(3)                                /*!< the time base of WWDGT = (PCLK1/4096)/8 */

/* write value to WWDGT_CTL_CNT bit field */
#define CTL_CNT(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))
/* write value to WWDGT_CFG_WIN bit field */
#define CFG_WIN(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))



#ifdef __cplusplus
}
#endif
#endif 
