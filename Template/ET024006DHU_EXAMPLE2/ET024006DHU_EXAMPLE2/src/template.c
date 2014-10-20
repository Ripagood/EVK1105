/*
Template 
*/


#include "board.h"
#include "gpio.h"
#include "power_clocks_lib.h"
#include "delay.h"
#include "conf_clock.h"
#include "tc.h"
#include "usart.h"
#include "compiler.h"
#include "pm.h"
#include "spi.h"
#include "eic.h"
#include "interrupt.h"
#include "template.h"
#include <stdlib.h>
#include <stdio.h>


#define EXAMPLE_TC (&AVR32_TC)
#define TC_CHANNEL0 0
#define TC_CHANNEL1 1
#define AVR32_TC_IRQ0 448
#define EXAMPLE_TC_IRQ AVR32_TC_IRQ0
#define FALSE 0

volatile int left_press=0;
volatile int right_press=0;
volatile int center_press=0;

volatile int segundos3=0;
volatile int tc_tick=0;
volatile avr32_tc_t *tc =  (&AVR32_TC);

volatile int pulso=1;
volatile char sentido=0;


__attribute__((__interrupt__))
static void tc_CTC(void)
{
	// Increment the 10ms seconds counter
	tc_tick++;

	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr( &AVR32_TC, TC_CHANNEL1);
	
	if (tc_tick>=300)//3SEGUNDOS
	{
		tc_tick=0;
		segundos3=1;
	}
	
}

__attribute__ ((__interrupt__))
static void touch_button_isr(void){
	
	eic_clear_interrupt_line(&AVR32_EIC, QT1081_EIC_EXTINT_INT);
	// UP
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_UP))
	{
		pulso++;
		if (pulso>=4)
		{
			pulso=4;
		}
		tc_write_ra(tc,TC_CHANNEL0,pulso*7500);
		
	}
	// DOWN
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_DOWN))
	{
		pulso--;
		if (pulso<=1)
		{
			pulso=1;
		}
		tc_write_ra(tc,TC_CHANNEL0,pulso*7500);
		
	}
	
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_RIGHT))
	{
		right_press=1;
		sentido=0;
	}
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_LEFT))
	{
		left_press=1;
		sentido=1;
		
	}
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER))
	{
		center_press=1;
	}
}


__attribute__ ((__interrupt__)) 
static void tecla_lrc_isr(void){//handler teclas left, right o center

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_LEFT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_LEFT);
	}

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_RIGHT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_RIGHT);
		
	}
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP))
	{
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP);
	}
	
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN))
	{
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN);		
	}
	
	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_ENTER))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER);
	}	

}

__attribute__((__interrupt__))
static void uart_interrupt(void){
	
	int c;
	char cadena[20];
	
	
	usart_read_char((&AVR32_USART0),&c);
	
	
} 

// Main function
int main(void)
{
    // Set CPU and PBA clock
    initClocks();
	Disable_global_interrupt();
	INTC_init_interrupts();
	setupCTC();
	setupEIC();
	//setUpGpioInterrupt();
	Enable_global_exception();
	
	setupPWM();
	setupSPI();
	
	gpio_enable_gpio_pin(LED0_GPIO);
	gpio_enable_gpio_pin(LED1_GPIO);
	gpio_enable_gpio_pin(LED2_GPIO);
	gpio_enable_gpio_pin(LED3_GPIO);
	
	gpio_enable_gpio_pin(AVR32_PIN_PA25);
	gpio_enable_gpio_pin(AVR32_PIN_PA26);
	
	 gpio_set_gpio_pin(AVR32_PIN_PA25);
	 gpio_clr_gpio_pin(AVR32_PIN_PA26);

  while(1){  
	  
	  if (right_press)
	  {
		  right_press=0;
		  tc_start(tc,TC_CHANNEL1); //empieza la interrupcion de 10ms
		  gpio_clr_gpio_pin(AVR32_PIN_PA25);
		  while(segundos3==0); //espera a la bandera
		  segundos3=0;
		  gpio_set_gpio_pin(AVR32_PIN_PA25);
		  tc_stop(tc,TC_CHANNEL1);
	  }
	  
	  if (left_press)
	  {
		  left_press=0;
		  tc_start(tc,TC_CHANNEL1); //empieza la interrupcion de 10ms
		  gpio_set_gpio_pin(AVR32_PIN_PA26);
		  while(segundos3==0); //espera a la bandera
		  segundos3=0;
		  gpio_clr_gpio_pin(AVR32_PIN_PA26);
		  tc_stop(tc,TC_CHANNEL1);
	  }
	  
	  if (center_press)
	  {
		  center_press=0;
		  spi_selectChip(&AVR32_SPI0,2);
		  spi_write(&AVR32_SPI0,pulso*10);
		  spi_write(&AVR32_SPI0,sentido);
	  }
	  
	  
	}
	  
	  
	
  }
  
 
void initClocks (void){
	
	//pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); //osc0 a 12Mhz
	pcl_switch_to_osc(PCL_OSC0,FOSC0,3);//3 = 2048 = 18ms
	
	
	pm_cksel(&AVR32_PM, 1, // pbadiv.
						2, // pbasel.2^(N+1)
						0,// pbbdiv.
						0,// pbbsel.
						0,// hsbdiv. CPU = HSB
						0);// hsbsel.2^(0+1)=2
	pm_gc_setup(&AVR32_PM,
						2, //numero de GC
						0,// 0= OSC, 1=PLL
						1, //OSCx, PLLx x=1
						1,//DIV enable
						15);// 2*(DIV+1)=32
	pm_gc_enable(&AVR32_PM,2);
	
	
}

void setupEIC(void){
	
	
	//Interrupciones
	
	//! Structure holding the configuration parameters of the EIC module.
	eic_options_t eic_options[2];
	// Enable edge-triggered interrupt.
	eic_options[0].eic_mode  = EIC_MODE_EDGE_TRIGGERED;
	// Interrupt will trigger on falling edge (this is a must-do for the keypad scan
	// feature if the chosen mode is edge-triggered).
	eic_options[0].eic_edge  = EIC_EDGE_RISING_EDGE;
	// Initialize in synchronous mode : interrupt is synchronized to the clock
	eic_options[0].eic_async = EIC_SYNCH_MODE;
	// Set the interrupt line number.
	eic_options[0].eic_line  = QT1081_EIC_EXTINT_INT;
    /* Register the EXTINT1 interrupt handler to the interrupt controller
     */
	INTC_register_interrupt(&touch_button_isr, QT1081_EIC_EXTINT_IRQ, AVR32_INTC_INT0);
	
	 // Init the EIC controller with the options
	 eic_init(&AVR32_EIC, eic_options, 1);
	 // Enable the EIC lines.
	 eic_enable_lines(&AVR32_EIC, (1<<eic_options[0].eic_line));
	 // Enable the interrupt for each EIC line.
	 eic_enable_interrupt_lines(&AVR32_EIC, (1<<eic_options[0].eic_line));
 
	 gpio_enable_module_pin( QT1081_EIC_EXTINT_PIN, QT1081_EIC_EXTINT_FUNCTION);
 
}

void initUSART(void){
	#  define EXAMPLE_USART                 (&AVR32_USART0)
	#  define EXAMPLE_USART_RX_PIN          AVR32_USART0_RXD_0_0_PIN
	#  define EXAMPLE_USART_RX_FUNCTION     AVR32_USART0_RXD_0_0_FUNCTION
	#  define EXAMPLE_USART_TX_PIN          AVR32_USART0_TXD_0_0_PIN
	#  define EXAMPLE_USART_TX_FUNCTION     AVR32_USART0_TXD_0_0_FUNCTION
	#  define EXAMPLE_USART_CLOCK_MASK      AVR32_USART0_CLK_PBA
	#  define EXAMPLE_PDCA_CLOCK_HSB        AVR32_PDCA_CLK_HSB
	#  define EXAMPLE_PDCA_CLOCK_PB         AVR32_PDCA_CLK_PBA
	static const gpio_map_t USART_GPIO_MAP =
	{
		{EXAMPLE_USART_RX_PIN, EXAMPLE_USART_RX_FUNCTION},
		{EXAMPLE_USART_TX_PIN, EXAMPLE_USART_TX_FUNCTION}
	};

	// USART options.
	static const usart_options_t USART_OPTIONS =
	{
		.baudrate     = 57600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE
	};

	// Assign GPIO to USART.
	gpio_enable_module(USART_GPIO_MAP,
	sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// Initialize USART in RS232 mode.
	usart_init_rs232(EXAMPLE_USART, &USART_OPTIONS, FOSC0);

	// Hello world!
	usart_write_line(EXAMPLE_USART, "Hello, this is the AVR UC3 MCU saying hello!\r\n");

	
	
}

void setupPWM(void){
	// Options for waveform genration.
	static const tc_waveform_opt_t WAVEFORM_OPT =
	{
		.channel  = TC_CHANNEL0,                        // Channel selection. 0

		.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

		.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
		.acpc     = TC_EVT_EFFECT_SET,                // RC compare effect on TIOA: toggle.
		.acpa     = TC_EVT_EFFECT_CLEAR,                  // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
		.enetrg   = FALSE,                                          // External event trigger enable.
		.eevt     = 0,                                                    // External event selection.
		.eevtedg  = TC_SEL_NO_EDGE,                   // External event edge selection.
		.cpcdis   = FALSE,                                         // Counter disable when RC compare.
		.cpcstop  = FALSE,                                        // Counter clock stopped with RC compare.

		.burst    = FALSE,                                           // Burst signal selection.
		.clki     = FALSE,                                            // Clock inversion.
		.tcclks   = TC_CLOCK_SOURCE_TC2         // Internal source clock 2, connected to fPBA / 2.
	};
	gpio_enable_module_pin(AVR32_TC_A0_0_0_PIN , AVR32_TC_A0_0_0_FUNCTION); // 55 , 0
	
	tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.
	tc_write_rc(tc, TC_CHANNEL0, 37500);            // Set RC value.
	tc_write_ra(tc,TC_CHANNEL0,7500);
	// Start the timer/counter.
	tc_start(tc, TC_CHANNEL0);                    // And start the timer/counter.
	
	
}

void setupCTC(void){
	
	// Options for waveform genration.
	static const tc_waveform_opt_t WAVEFORM_OPT2 =
	{
		.channel  = TC_CHANNEL1,                        // Channel selection. 2

		.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

		.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
		.acpa     = TC_EVT_EFFECT_NOOP,                  // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
		.enetrg   = FALSE,                                          // External event trigger enable.
		.eevt     = 0,                                                    // External event selection.
		.eevtedg  = TC_SEL_NO_EDGE,                   // External event edge selection.
		.cpcdis   = FALSE,                                         // Counter disable when RC compare.
		.cpcstop  = FALSE,                                        // Counter clock stopped with RC compare.

		.burst    = FALSE,                                           // Burst signal selection.
		.clki     = FALSE,                                            // Clock inversion.
		.tcclks   = TC_CLOCK_SOURCE_TC2         // Internal source clock 2, connected to fPBA / 2.
	};

	static const tc_interrupt_t TC_INTERRUPT =
	{
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1,   // Habilitar interrupción por comparación con RC
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	
	//Interrupciones

	INTC_register_interrupt(&tc_CTC, AVR32_TC_IRQ0, AVR32_INTC_INT0);//prioridad 0
	
	tc_init_waveform(tc, &WAVEFORM_OPT2);         // Initialize the timer/counter waveform.

	//10ms
	
	tc_write_rc(tc, TC_CHANNEL1, 7500);            // Set RC value.

	tc_configure_interrupts(tc, TC_CHANNEL1, &TC_INTERRUPT);

	// Start the timer/counter.
	//tc_start(tc, TC_CHANNEL1);                    // And start the timer/counter.

}

void setupSPI(void){
	#define  DLYBCS 60
	#define  PS     0
	#define  PCS_DECODE 0
	static const gpio_map_t SPI_GPIO_MAP =
	{
		{AVR32_SPI0_SCK_0_0_PIN , AVR32_SPI0_SCK_0_0_FUNCTION},  // SPI Clock.
		{AVR32_SPI0_MISO_0_0_PIN, AVR32_SPI0_MOSI_0_0_FUNCTION},  // MISO.
		{AVR32_SPI0_MOSI_0_0_PIN, AVR32_SPI0_MOSI_0_0_FUNCTION},  // MOSI.
		{AVR32_SPI0_NPCS_2_0_PIN, AVR32_SPI0_SCK_0_0_FUNCTION}   // Chip Select NPCS.
	};

	// SPI options.
	spi_options_t spiOptions =
	{
		.reg          =  2,//seleccion chip select
		.baudrate     =  200000,
		.bits         =  8,
		.spck_delay   =  0 ,//Delay antes del SPCK (DLYBS),
		.trans_delay  =  0,//Delay entre transiciones consecutivas (DLYBCT) = D / MCK
		.stay_act     =  1, //Deselección de perífpericos (CSAAT),
		.spi_mode     =  0, //Modo (CPOL y NCPHA),
		.modfdis      =  1, //Modo Fault Detection  1- Inhabilitado  0 -Habilitado
	};

	gpio_enable_module(SPI_GPIO_MAP,
	sizeof(SPI_GPIO_MAP) / sizeof(SPI_GPIO_MAP[0]));

	spi_initMaster(&AVR32_SPI0, &spiOptions);

	// Set SPI selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode(&AVR32_SPI0, PS, PCS_DECODE, DLYBCS);

	// Enable SPI module.
	spi_enable(&AVR32_SPI0);

	spi_setupChipReg(&AVR32_SPI0,&spiOptions,FOSC0);
	
}

void setUpGpioInterrupt(void){
	
	
	//Interrupciones

	INTC_register_interrupt(&tecla_lrc_isr, 71, 0);
	INTC_register_interrupt(&tecla_lrc_isr, 70,0);
	
	// INTC_register_interrupt(&uart_interrupt,AVR32_USART0_IRQ,AVR32_INTC_INT0);
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_LEFT,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_RIGHT,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_ENTER,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_UP,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_DOWN,GPIO_RISING_EDGE);
	
}

uint32_t debounce2( uint32_t GPIO_PIN ){//regresar se presiono el boton o no
	if(gpio_get_pin_value(GPIO_PIN)==1){// se presiono el boton?, sino salir de la funcion
		delay_ms(10);
		if (gpio_get_pin_value(GPIO_PIN)==0){//Si ya se libero, es ruido, salir sin hacer nada
			goto salir;
		}
		espera://espera a que suelte el botón
		while (gpio_get_pin_value(GPIO_PIN)==1){}
		delay_ms(10);
		if (gpio_get_pin_value(GPIO_PIN)==1) {//si ya lo presiono otra vez , es ruido, regresa a esperar
			goto espera;
		}
		return 1;//debounce completo regresa 1
	}
	salir:
	return 0;
}




