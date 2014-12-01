#include "board.h"
#include "gpio.h"
#include "power_clocks_lib.h"
#include "et024006dhu.h"
#include "delay.h"
#include "avr32_logo.h"
#include "conf_clock.h"
#include <stdio.h>
#include "iggy.h"
#include "tc.h"
#include "conf_clock.h"
#include "usart.h"
#include <stdlib.h>
#include "Practica4Lab.h"
#include "spi.h"
#include "conf_sd_mmc_spi.h"
#include "sd_mmc_spi.h"
#include "pdca.h"
#include "conf_tpa6130.h"
#include "board.h"
#include "audio.h"
#include "twi.h"
#include "ExamenPractico2.h"
#include "flashc.h"
#include "string.h"


#define FREQ 62092800

volatile int enter=0;
volatile int fin=0;
volatile int running=0;
volatile int end_of_transfer=0;
// PDCA Channel pointer
volatile avr32_pdca_channel_t* pdca_channel_usart ;

// PDCA Channel pointer
volatile avr32_pdca_channel_t* pdca_channelrx ;
volatile avr32_pdca_channel_t* pdca_channeltx ;

// Dummy char table
const char dummy_data[] =
#include "dummy.h"
;
volatile char ram_buffer[516];
/*! \brief Initialize PDCA (Peripheral DMA Controller A) resources for the SPI transfer and start a dummy transfer
 */
#define TC_CHANNEL    0
#define FALSE 0
#define AVR32_TC_IRQ0 448
#define EXAMPLE_TC_IRQ AVR32_TC_IRQ0

#  define EXAMPLE_USART                 (&AVR32_USART0)
#  define EXAMPLE_USART_RX_PIN          AVR32_USART0_RXD_0_0_PIN
#  define EXAMPLE_USART_RX_FUNCTION     AVR32_USART0_RXD_0_0_FUNCTION
#  define EXAMPLE_USART_TX_PIN          AVR32_USART0_TXD_0_0_PIN
#  define EXAMPLE_USART_TX_FUNCTION     AVR32_USART0_TXD_0_0_FUNCTION
#  define EXAMPLE_USART_CLOCK_MASK      AVR32_USART0_CLK_PBA
#  define EXAMPLE_PDCA_CLOCK_HSB        AVR32_PDCA_CLK_HSB
#  define EXAMPLE_PDCA_CLOCK_PB         AVR32_PDCA_CLK_PBA

#define PDCA_CHANNEL_USART_EXAMPLE 2
#define AVR32_PDCA_CHANNEL_SPI_TX 1 //CANAL 1 para SPI TX
#define AVR32_PDCA_CHANNEL_SPI_RX 0 //CANAL 2 para SPI RX

//! Sample Count Value
#define SOUND_SAMPLES                512
//#define FPBA_HZ                 12000000
#define TPA6130_TWI_MASTER_SPEED  100000

void dac_reload_callback(void);
void dac_overrun_callback(void);
void adc_underrun_callback(void);
void adc_reload_callback(void);

int16_t samples[SOUND_SAMPLES];

#define SAMPLE_OFFSET   0x80
#define SAMPLE_RATE     11025
#define SAMPLE_COUNT    SOUND_SAMPLES/2

 uint8_t* coordenas;
 uint8_t* adelante;
 uint8_t* atras;
 uint8_t* izquierda;
 uint8_t* derecha;

volatile avr32_tc_t *tc =  (&AVR32_TC);

__attribute__ ((__interrupt__)) void tecla_lrc_isr(void){//handler teclas left, right o center
	
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
		enter=1;
		
		if (running==1)//se interrumpio el movimiento
		{
			tc_stop(tc,1);
			running=0;
			gpio_set_gpio_pin(LED0_GPIO);//vehiculo detenido
			gpio_clr_gpio_pin(LED2_GPIO);
		}else{//se reanudo el movimiento
			tc_start(tc,1);
			running=1;
			gpio_clr_gpio_pin(LED0_GPIO);//vehiculo en movimiento
			gpio_set_gpio_pin(LED2_GPIO);
			
		}
		
	}
}


volatile U32 tc_tick = 0;


__attribute__((__interrupt__))
static void pdca_int_handler_USART(void){
	Disable_global_interrupt();
	pdca_disable_interrupt_transfer_complete(PDCA_CHANNEL_USART_EXAMPLE);
	pdca_disable(PDCA_CHANNEL_USART_EXAMPLE);
	end_of_transfer=1;
	Enable_global_interrupt();
	
}


__attribute__((__interrupt__))
static void pdca_int_handler_SD(void)
{
  // Disable all interrupts.
  Disable_global_interrupt();

  // Disable interrupt channel.
  pdca_disable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);

  sd_mmc_spi_read_close_PDCA();//unselects the SD/MMC memory.
  wait();
  // Disable unnecessary channel
  pdca_disable(AVR32_PDCA_CHANNEL_SPI_TX);
  pdca_disable(AVR32_PDCA_CHANNEL_SPI_RX);

  // Enable all interrupts.
  Enable_global_interrupt();

  end_of_transfer = 1;
}

static void tc_CTC(void)
{
uint16_t x,y;
static uint8_t i=0;
  // Increment the seconds counter
  tc_tick++;

  // Clear the interrupt flag. This is a side effect of reading the TC SR.
  tc_read_sr(tc, TC_CHANNEL);

  // Toggle a GPIO pin (this pin is used as a regular GPIO pin).
  if (tc_tick>=3)
  {		
	     tc_tick =0;
		 
		 spi_write(&AVR32_SPI0,0xFF);
		 spi_read(&AVR32_SPI0,&x);
		 spi_write(&AVR32_SPI0,0xFF);
		 spi_read(&AVR32_SPI0,&y);
		 
		 if (coordenas[i++]>=x)
		 {
			 sonido(atras);
		 }else{
			 sonido(adelante);
		 }
		 
		 if (coordenas[i++]>=y)
		 {
			 sonido(derecha);
			 }else{
			 sonido(izquierda);
		 }
		 
		 if (i>199)
		 {
			 i=0;
			 fin=1;
		 }
		 
		 
		 
		 
	 
  }
  

}

 
// Main function
int main(void)
{
  
  init_sys_clocks();

  gpio_enable_gpio_pin(LED0_GPIO);
  gpio_enable_gpio_pin(LED1_GPIO);
  gpio_enable_gpio_pin(LED2_GPIO);
  gpio_enable_gpio_pin(LED3_GPIO);
  //Interrupciones
  
  Disable_global_interrupt();

  INTC_init_interrupts();
  setUpGpioInterrupt();
  setupCTC();
  init_Usart_DMA_RX();
  init_SD_DMA_RX();

  Enable_global_interrupt();
  
  // Initialize SD/MMC driver resources: GPIO, SPI and SD/MMC.
  sd_mmc_resources_init();
//espera a que se inserte la SD
  while (!sd_mmc_spi_mem_check());
 coordenas = (uint8_t*)malloc(sizeof(uint8_t)*512);//2
 adelante = (uint8_t*)malloc(sizeof(uint8_t)*512);//3
 atras = (uint8_t*)malloc(sizeof(uint8_t)*512);//4
 izquierda = (uint8_t*)malloc(sizeof(uint8_t)*512);//5
 derecha = (uint8_t*)malloc(sizeof(uint8_t)*512);//6
 
 uint8_t* datos[]={coordenas,adelante,atras,izquierda,derecha};
 
 //Copia los datos de la SD a arreglos dentro de la RAM
 for(int j = 2; j <= 6; j++)
 {
	 // Configure the PDCA channel: the address of memory ram_buffer to receive the data at sector address j
	 pdca_load_channel( AVR32_PDCA_CHANNEL_SPI_RX,
	 &ram_buffer,//RAM
	 512);

	 pdca_load_channel( AVR32_PDCA_CHANNEL_SPI_TX,
	 (void *)&dummy_data,
	 512); //send dummy to activate the clock

	 end_of_transfer = 0;
	 // open sector number j
	 if(sd_mmc_spi_read_open_PDCA (j))
	 {
		 //spi_write(SD_MMC_SPI,0xFF); // Write a first dummy data to synchronize transfer
		 pdca_enable_interrupt_transfer_complete(AVR32_PDCA_CHANNEL_SPI_RX);
		 pdca_channelrx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_RX); // get the correct PDCA channel pointer
		 pdca_channeltx =(volatile avr32_pdca_channel_t*) pdca_get_handler(AVR32_PDCA_CHANNEL_SPI_TX); // get the correct PDCA channel pointer
		 pdca_channelrx->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
		 pdca_channeltx->cr = AVR32_PDCA_TEN_MASK; // and TX PDCA transfer
		  while(!end_of_transfer);
		  memcpy(ram_buffer,datos[j-2],512);
	 }
 }
 
 pdca_disable(AVR32_PDCA_CHANNEL_SPI_RX);
 pdca_disable(AVR32_PDCA_CHANNEL_SPI_TX);
 
 setupSPI(); //habilita SPI para el acelerometro
  // Configure TWI as master
  twi_init();
  // Initialize TPA6130
  tpa6130_init();
  // Initialize DAC that send audio to TPA6130
  tpa6130_dac_start(DEFAULT_DAC_SAMPLE_RATE_HZ,
  DEFAULT_DAC_NUM_CHANNELS,
  DEFAULT_DAC_BITS_PER_SAMPLE,
  DEFAULT_DAC_SWAP_CHANNELS,
  master_callback,
  AUDIO_DAC_OUT_OF_SAMPLE_CB
  | AUDIO_DAC_RELOAD_CB,
  FREQ);
  tpa6130_set_volume(0x2F);
  tpa6130_get_volume();
  
  gpio_clr_gpio_pin(LED2_GPIO);//vehiculo detenido
  gpio_set_gpio_pin(LED0_GPIO);

while (1)
{
	running=0;
	enter=0;
	fin=0;
	while(!enter);//espera que se presione center
	running=1;
	gpio_clr_gpio_pin(LED0_GPIO);//vehiculo en movimiento
	gpio_set_gpio_pin(LED2_GPIO);
	tc_start(tc, 1); //empieza el timer para interrupcion cada segundo
	while(!fin);//despues de que pasen las 100 coordenadas, se activa fin
	pdca_load_channel( PDCA_CHANNEL_USART_EXAMPLE,&coordenas[0],200);
	pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_USART_EXAMPLE);
	pdca_channel_usart =(volatile avr32_pdca_channel_t*) pdca_get_handler(PDCA_CHANNEL_USART_EXAMPLE); // get the correct PDCA channel pointer
	pdca_channel_usart->cr = AVR32_PDCA_TEN_MASK; // Enable TX PDCA transfer first
	end_of_transfer=0;
	while (!end_of_transfer);
	pdca_disable(PDCA_CHANNEL_USART_EXAMPLE);
}
  
  

  
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

static void sd_mmc_resources_init(void)
{
	// GPIO pins used for SD/MMC interface
	static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
	{
		{SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
		{SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
		{SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
		{SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};

	// SPI options.
	spi_options_t spiOptions =
	{
		.reg          = SD_MMC_SPI_NPCS,
		.baudrate     = SD_MMC_SPI_MASTER_SPEED,  // Defined in conf_sd_mmc_spi.h.
		.bits         = SD_MMC_SPI_BITS,          // Defined in conf_sd_mmc_spi.h.
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 1
	};

	// Assign I/Os to SPI.
	gpio_enable_module(SD_MMC_SPI_GPIO_MAP,
	sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

	// Initialize as master.
	spi_initMaster(SD_MMC_SPI, &spiOptions);

	// Set SPI selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode(SD_MMC_SPI, 0, 0, 0);

	// Enable SPI module.
	spi_enable(SD_MMC_SPI);

	// Initialize SD/MMC driver with SPI clock (PBA).
	sd_mmc_spi_init(spiOptions, FREQ);
}




void init_Usart_DMA_RX(void){
	
	const gpio_map_t usart_gpio_map = {
		{EXAMPLE_USART_RX_PIN, EXAMPLE_USART_RX_FUNCTION},
		{EXAMPLE_USART_TX_PIN, EXAMPLE_USART_TX_FUNCTION}
	};

	const usart_options_t usart_options = {
		.baudrate     = 57600,
		.charlength   = 8,
		.paritytype   = USART_NO_PARITY,
		.stopbits     = USART_1_STOPBIT,
		.channelmode  = USART_NORMAL_CHMODE,
	};

	const pdca_channel_options_t PDCA_OPTIONS = {
		/* Select peripheral - data is transmitted on USART TX line */
		.pid = AVR32_PDCA_PID_USART0_TX,
		/* Select size of the transfer */
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE,
		/* Memory address */
		.addr = (void*)coordenas,
		/* Transfer counter */
		.size = 200, //6 
		/* Next memory address */
		.r_addr = NULL,
		/* Next transfer counter */
		.r_size = 0,
	};

	/* Assign GPIO pins to USART. */
	gpio_enable_module(usart_gpio_map,
			sizeof(usart_gpio_map) / sizeof(usart_gpio_map[0]));
	/* Initialize the USART in RS232 mode. */
	usart_init_rs232(EXAMPLE_USART, &usart_options,FREQ);
	/* Initialize the PDCA channel with the requested options. */
	pdca_init_channel(PDCA_CHANNEL_USART_EXAMPLE, &PDCA_OPTIONS);
	INTC_register_interrupt(&pdca_int_handler_USART, AVR32_PDCA_IRQ_2,AVR32_INTC_INT3);
	/* Enable PDCA interrupt each time the reload counter reaches zero, i.e.
	 * each time half of the ASCII animation (either anim1 or anim2) is
	 * transferred. */
	//pdca_enable_interrupt_reload_counter_zero(PDCA_CHANNEL_USART_EXAMPLE);
	
	pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_USART_EXAMPLE);

}


void init_SD_DMA_RX(void)
{
	
  // this PDCA channel is used for data reception from the SPI
  pdca_channel_options_t pdca_options_SPI_RX ={ // pdca channel options

    .addr = ram_buffer,//RAM
    // memory address. We take here the address of the string dummy_data. This string is located in the file dummy.h

    .size = 512,                              // transfer counter: here the size of the string
    .r_addr = NULL,                           // next memory address after 1st transfer complete
    .r_size = 0,                              // next transfer counter not used here
    .pid = AVR32_PDCA_PID_SPI0_RX,        // select peripheral ID - data are on reception from SPI1 RX line
    .transfer_size = 8 // select size of the transfer: 8,16,32 bits
  };

  // this channel is used to activate the clock of the SPI by sending a dummy variables
  pdca_channel_options_t pdca_options_SPI_TX ={ // pdca channel options

    .addr = (void *)&dummy_data,              // memory address.
                                              // We take here the address of the string dummy_data.
                                              // This string is located in the file dummy.h
    .size = 512,                              // transfer counter: here the size of the string
    .r_addr = NULL,                           // next memory address after 1st transfer complete
    .r_size = 0,                              // next transfer counter not used here
    .pid = AVR32_PDCA_PID_SPI0_TX,        // select peripheral ID - data are on reception from SPI1 RX line
    .transfer_size = 8 // select size of the transfer: 8,16,32 bits
  };

  // Init PDCA transmission channel
  pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_TX, &pdca_options_SPI_TX);

  // Init PDCA Reception channel
  pdca_init_channel(AVR32_PDCA_CHANNEL_SPI_RX, &pdca_options_SPI_RX);

  //! \brief Enable pdca transfer interrupt when completed
  INTC_register_interrupt(&pdca_int_handler_SD, AVR32_PDCA_IRQ_0, AVR32_INTC_INT1);  // pdca_channel_spi1_RX = 0

}

// Software wait
void wait(void)
{
	volatile int i;
	for(i = 0 ; i < 5000; i++);
}

void setupCTC(void){
	// Options for waveform genration.
	static const tc_waveform_opt_t WAVEFORM_OPT2 =
	{
		.channel  = TC_CHANNEL,                        // Channel selection. 0

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
		.tcclks   = TC_CLOCK_SOURCE_TC1         // Internal source clock 1, connected to 32Khz
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

	//1s
	
	tc_write_rc(tc, TC_CHANNEL, 31999);            // Set RC value.

	tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

	// Start the timer/counter.
	//tc_start(tc, TC_CHANNEL1);                    // And start the timer/counter.

}

void init_sys_clocks(void)
{
	// Switch to OSC0 to speed up the booting
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

	// Start oscillator1
	pm_enable_osc1_crystal(&AVR32_PM, FOSC1);
	pm_enable_clk1(&AVR32_PM, OSC1_STARTUP);

	// Set PLL0 (fed from OSC1 = 11.2896 MHz) to 124.1856 MHz
	// We use OSC1 since we need a correct master clock for the SSC module to generate
	// the correct sample rate
	pm_pll_setup(&AVR32_PM, 0,  // pll.
	10,  // mul.
	1,   // div.
	1,   // osc.
	16); // lockcount.

	// Set PLL operating range and divider (fpll = fvco/2)
	// -> PLL0 output = 62.0928 MHz
	pm_pll_set_option(&AVR32_PM, 0, // pll.
	1,  // pll_freq.
	1,  // pll_div2.
	0); // pll_wbwdisable.

	// start PLL0 and wait for the lock
	pm_pll_enable(&AVR32_PM, 0);
	pm_wait_for_pll0_locked(&AVR32_PM);
	// Set all peripheral clocks torun at master clock rate
	pm_cksel(&AVR32_PM,
	0,   // pbadiv.
	0,   // pbasel.
	0,   // pbbdiv.
	0,   // pbbsel.
	0,   // hsbdiv.
	0);  // hsbsel.

	// Set one waitstate for the flash
	flashc_set_wait_state(1);

	// Switch to PLL0 as the master clock
	pm_switch_to_clock(&AVR32_PM, AVR32_PM_MCCTRL_MCSEL_PLL0);

	// Use 12MHz from OSC0 and generate 96 MHz
	pm_pll_setup(&AVR32_PM, 1,  // pll.
	7,   // mul.
	1,   // div.
	0,   // osc.
	16); // lockcount.

	pm_pll_set_option(&AVR32_PM, 1, // pll.
	1,  // pll_freq: choose the range 80-180MHz.
	1,  // pll_div2.
	0); // pll_wbwdisable.

	// start PLL1 and wait forl lock
	pm_pll_enable(&AVR32_PM, 1);

	// Wait for PLL1 locked.
	pm_wait_for_pll1_locked(&AVR32_PM);

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
		{AVR32_SPI0_NPCS_3_0_PIN, AVR32_SPI0_NPCS_3_0_FUNCTION}   // Chip Select NPCS.
	};

	// SPI options.
	spi_options_t spiOptions =
	{
		.reg          =  3,//seleccion chip select
		.baudrate     =  50000,
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

	spi_setupChipReg(&AVR32_SPI0,&spiOptions,FREQ);
	
}


void sonido(uint8_t* sonidos){
	int count = 0;
    int i=0;
    // Store sample from the sound_table array
    while(count < (SOUND_SAMPLES)){
      samples[count++] = ((uint8_t)sonidos[i]+0x80) << 8;
      samples[count++] = ((uint8_t)sonidos[i]+0x80) << 8;
      i++;
      if (i >= 256) i = 0;
    }
    tpa6130_dac_output((void *) samples,SOUND_SAMPLES/2);
    while(!tpa6130_dac_output(NULL, 0));
}

static void twi_init(void)
{
	const gpio_map_t TPA6130_TWI_GPIO_MAP =
	{
		{TPA6130_TWI_SCL_PIN, TPA6130_TWI_SCL_FUNCTION},
		{TPA6130_TWI_SDA_PIN, TPA6130_TWI_SDA_FUNCTION}
	};

	const twi_options_t TPA6130_TWI_OPTIONS =
	{
		.pba_hz = FREQ,
		.speed  = TPA6130_TWI_MASTER_SPEED,
		.chip   = TPA6130_TWI_ADDRESS
	};

	// Assign I/Os to SPI.
	gpio_enable_module(TPA6130_TWI_GPIO_MAP,
	sizeof(TPA6130_TWI_GPIO_MAP) / sizeof(TPA6130_TWI_GPIO_MAP[0]));

	// Initialize as master.
	twi_master_init(TPA6130_TWI, &TPA6130_TWI_OPTIONS);
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


void master_callback(uint32_t arg)
{
	if( arg == AUDIO_DAC_OUT_OF_SAMPLE_CB )
	{
		dac_overrun_callback();
	}

	else if( arg == AUDIO_DAC_RELOAD_CB )
	{
		dac_reload_callback();
	}

	else if( arg == AUDIO_ADC_OUT_OF_SAMPLE_CB )
	{
		adc_underrun_callback();;
	}

	else if( arg == AUDIO_ADC_RELOAD_CB )
	{
		adc_reload_callback();;
	}
}

void dac_reload_callback(void)
{
	// Nothing todo
}

void dac_overrun_callback(void)
{
	// Nothing todo
}


void adc_underrun_callback(void)
{
	// Nothing todo
}


void adc_reload_callback(void)
{
	// Nothing todo
}