#include "n200_func.h"
#include "gd32vf103.h"

#include <stdint.h>


void loop_int_handler(void);
void main(void);

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

const uint32_t g_pll_mults[31] = {
  RCU_PLL_MUL2,   RCU_PLL_MUL3,   RCU_PLL_MUL4,   RCU_PLL_MUL5
, RCU_PLL_MUL6,   RCU_PLL_MUL7,   RCU_PLL_MUL8,   RCU_PLL_MUL9
, RCU_PLL_MUL10,  RCU_PLL_MUL11,  RCU_PLL_MUL12,  RCU_PLL_MUL13
, RCU_PLL_MUL14,  RCU_PLL_MUL6_5, RCU_PLL_MUL16,  RCU_PLL_MUL17
, RCU_PLL_MUL18,  RCU_PLL_MUL19,  RCU_PLL_MUL20,  RCU_PLL_MUL21
, RCU_PLL_MUL22,  RCU_PLL_MUL23,  RCU_PLL_MUL24,  RCU_PLL_MUL25
, RCU_PLL_MUL26,  RCU_PLL_MUL27,  RCU_PLL_MUL28,  RCU_PLL_MUL29
, RCU_PLL_MUL30,  RCU_PLL_MUL31,  RCU_PLL_MUL32
};

const uint32_t g_predv0_divs[16] = {
  RCU_PREDV0_DIV1,  RCU_PREDV0_DIV2,  RCU_PREDV0_DIV3,  RCU_PREDV0_DIV4
, RCU_PREDV0_DIV5,  RCU_PREDV0_DIV6,  RCU_PREDV0_DIV7,  RCU_PREDV0_DIV8
, RCU_PREDV0_DIV9,  RCU_PREDV0_DIV10, RCU_PREDV0_DIV11, RCU_PREDV0_DIV12
, RCU_PREDV0_DIV13, RCU_PREDV0_DIV14, RCU_PREDV0_DIV15, RCU_PREDV0_DIV16
};


uint32_t SystemCoreClock = HXTAL_VALUE;
volatile uint32_t g_milliseconds = 0u;


void startup_code(void) {
  // copy initialized data
  uint32_t * v_ptr = (uint32_t *)&_sidata;
  for ( uint32_t * i = (uint32_t *)&_sdata; i < (uint32_t *)&_edata; ++i ) {
    *i = *v_ptr++;
  }
  // zero initialized data
  for ( uint32_t * i = (uint32_t *)&_sbss; i < (uint32_t *)&_ebss; ++i ) {
    *i = 0;
  }
  // setup clocks
  // enable crystal OSC
  RCU_CTL |= RCU_CTL_HXTALEN;
  // wait for it stabilized
  while ( 0u == (RCU_CTL & RCU_CTL_HXTALSTB) ) {}
  // bus dividers
  // AHB = SYSCLK
  RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
  // APB2 = AHB/1
  RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
  // APB1 = AHB/2
  RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;
  // find divider and multiplicant
  bool v_found = FALSE;
  uint32_t v_div, v_mul;
  for ( v_div = 1u; v_div <= 16u; ++v_div ) {
    for ( v_mul = 2u; v_mul <= 32u; ++v_mul ) {
      if ( v_mul != 15u ) {
        v_found = ( CK_SYS_VALUE == ((HXTAL_VALUE * v_mul) / v_div) );
      } else {
        v_found = ( CK_SYS_VALUE == (((HXTAL_VALUE * 6u) + (HXTAL_VALUE / 2u)) / v_div) );
      }
      if ( v_found ) {
        break;
      }
    }
    if ( v_found ) {
      break;
    }
  }
  // check
  if ( !v_found ) {
    // oh, make CK_SYS == HXTAL freq
    v_div = 2;
    v_mul = 2;
  } else {
    SystemCoreClock = CK_SYS_VALUE;
  }
  // setup PLL source and multiplier, assume than RCU_CFG0 == 0 after reset
  RCU_CFG0 |= (RCU_PLLSRC_HXTAL | g_pll_mults[v_mul - 2u]);
  // setup PREDV0 source and divider
  RCU_CFG1 |= (RCU_PREDV0SRC_HXTAL | g_predv0_divs[v_div - 1u] );
  // enable PLL
  RCU_CTL |= RCU_CTL_PLLEN;
  // wait for PLL stabilized
  while( 0u == (RCU_CTL & RCU_CTL_PLLSTB) ) {}  
  // select PLL as source for CK_SYS
  RCU_CFG0 |= RCU_CKSYSSRC_PLL;
  // wait for selection complete
  while ( RCU_SCSS_PLL != (RCU_CFG0 & RCU_SCSS_MASK) ) {}
  // init and enable ECLIC
  eclic_init(ECLIC_NUM_INTERRUPTS);
  eclic_mode_enable();
  // Set 'vector mode' so the timer interrupt uses the vector table.
  eclic_set_vmode( CLIC_INT_TMR );
  // Enable the timer interrupt with low priority and 'level'.
  eclic_set_irq_lvl_abs( CLIC_INT_TMR, 1 );
  eclic_set_irq_priority( CLIC_INT_TMR, 1 );
  eclic_enable_interrupt( CLIC_INT_TMR );
  //
  reset_timer_value();
  set_timer_cmp_value( TIMER_FREQ / 1000 );
  // enable interrupts
  set_csr(mstatus, MSTATUS_MIE);
  //
  main();
  // freeze
  for( ;; ) {
  }
}


__attribute__ ((naked))
void loop_int_handler(void) {
  for( ;; ) {
  }
}


__attribute__ ((interrupt))
void eclic_mtip_handler(void) {
  // reset timer, it reset pending interrupt
  reset_timer_value();
  // add 1 for milliseconds counter
  ++g_milliseconds;
}


// delay for a_ms milliseconds
void delay_ms( uint32_t a_ms ) {
  uint32_t a_from = g_milliseconds;
  while ( ((uint32_t)(g_milliseconds - a_from)) < a_ms ) {__WFI();}
}
