/*

Equipo Laboratario Sistemas Embebidos

Rectangulo, Matrix, Triangle, Imagen , debounce2 and main.c by Ripagood //Elias Ventura
Ej1, Ej4, Ej7, Ej11 by Aldric
AjedrezRandom, AjedrezCrece by Juan Manuel


// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


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
#define PBA_HZ FOSC0
#define PRACTICA 3

volatile int enter=0;
volatile int end_of_transfer=0;

volatile char password[6];



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

#define  RGB(r,g,b) r<<11|g<<5|b 

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

#if BOARD == EVK1105
#include "pwm.h"
#include <string.h>
avr32_pwm_channel_t pwm_channel6 = {
/*
  .cmr = ((PWM_MODE_LEFT_ALIGNED << AVR32_PWM_CMR_CALG_OFFSET)
    | (PWM_POLARITY_HIGH << AVR32_PWM_CMR_CPOL_OFFSET)
    | (PWM_UPDATE_DUTY << AVR32_PWM_CMR_CPD_OFFSET)
    | AVR32_PWM_CMR_CPRE_MCK_DIV_2),
    */
  //.cdty = 0,
  .cdty = 0,
  .cprd = 100
};

__attribute__ ((__interrupt__)) void tecla_lrc_isr(void){//handler teclas left, right o center
	
	
	

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_LEFT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_LEFT);
		//LastRectX=RectX;
		#if	PRACTICA == 2
		
		if (actividad==0)
		{
			while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_LEFT)){
				et024006_DrawFilledRect(RectX , RectY, 30, 30, BLACK);//borra la posicion pasada
				RectX+=2;
				et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
				delay_ms(10);
				
			}
			
		}
		#endif
		
		
		
	}

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_RIGHT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_RIGHT);
		//LastRectX=RectX;
		if (actividad != 1)
		{
			actividad = 1; //te lleva al reloj
			CLR_disp();
		}
		
		
		
		#if PRACTICA == 2
		if (actividad==0)
		{
			while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_RIGHT)){
				et024006_DrawFilledRect(RectX , RectY, 30, 30, BLACK);//borra la posicion pasada
				RectX-=2;
				et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
				delay_ms(10);
			}
			
		}
		
		#endif
	}
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP))
	{
				gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP);
				
				
				if (actividad==2)//cronometro
				{
					resetTimer=1;
				}
				
				if (actividad != 2)
				{
					actividad = 2; //te lleva al cronometro
					CLR_disp();
				}
				
				
				
				#if PRACTICA == 2
				
				//LastRectY=RectY;
				if (actividad==0)
				{
					while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_UP)){
						et024006_DrawFilledRect(RectX , RectY, 30, 30, BLACK);//borra la posicion pasada
						RectY+=2;
						et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
						delay_ms(10);
						
						
					}
					
				}
				#endif
		     
			  
	}
	
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN))
	{
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN);
		if (actividad !=0)
		{
			actividad = 0; //te lleva al pong
			CLR_disp();
		}
		
		
		
		#if PRACTICA == 2
		//LastRectY=RectY;
		if (actividad==0)
		{
			while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_DOWN)){
				
				et024006_DrawFilledRect(RectX , RectY, 30, 30, BLACK);//borra la posicion pasada
				RectY-=2;
				et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
				delay_ms(10);
			}
			
		}
		#endif
		
	}
	
	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_ENTER))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER);
		//LastRectX=RectX;
		#if PRACTICA == 2
		enter=0;
		#endif
		if (actividad==2)
		{
			go^=(1<<0);//toggle al go del cronometro
		}
		
		if (actividad==1)
		{
			enter^=(1<<0);
		}
		
		
	}
	
	

}



static void tft_bl_init(void)
{

  pwm_opt_t opt = {
    .diva = 0,
    .divb = 0,
    .prea = 0,
    .preb = 0
  };
  /* MCK = OSC0 = 12MHz
   * Desired output 60kHz
   * Chosen MCK_DIV_2
   * CPRD = 12MHz / (60kHz * 2) = 100
   *
   * The duty cycle is 100% (CPRD = CDTY)
   * */
  pwm_init(&opt);
  pwm_channel6.CMR.calg = PWM_MODE_LEFT_ALIGNED;
  pwm_channel6.CMR.cpol = PWM_POLARITY_HIGH; //PWM_POLARITY_LOW;//PWM_POLARITY_HIGH;
  pwm_channel6.CMR.cpd = PWM_UPDATE_DUTY;
  pwm_channel6.CMR.cpre = AVR32_PWM_CMR_CPRE_MCK_DIV_2;

  pwm_channel_init(6, &pwm_channel6);
  pwm_start_channels(AVR32_PWM_ENA_CHID6_MASK);

}
#endif






volatile U32 tc_tick = 0;

volatile int segundos =0;
volatile int minutos =0;
volatile int horas=0;

volatile int gana2=1;
__attribute__((__interrupt__))
static void pdca_int_handler_USART(void){
	Disable_global_interrupt();
	
	usart_write_line(&AVR32_USART0,"recibido");
	
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
/*
__attribute__((__interrupt__))
static void uart_interrupt(void){
	#define imagen1 80000
	#define imagen2 80100
	int c;
	char cadena[20];
	//Disable_global_interrupt();
	
	usart_read_char((&AVR32_USART0),&c);
	
	if (c=='1')
	{
		usart_get_line((&AVR32_USART0),&cadena[0]);
		horas=atoi(cadena);
	}
	
	
	if (c=='2')
	{
		usart_get_line((&AVR32_USART0),&cadena[0]);
		minutos=atoi(cadena);
	}
	
	if (c=='3')
	{
		recibirImagen(imagen1);
	}
	
	if (c=='4')
	{
		displayImage(imagen1);
	}
	
	if (c=='5')
	{
		deleteImage(imagen1);
	}
	
	
	if (c=='6')
	{
		recibirImagen(imagen2);
	}
	
	if (c=='7')
	{
		displayImage(imagen2);
	}
	
	if (c=='8')
	{
		deleteImage(imagen2);
	}
	
	
	
	//usart_write_line((&AVR32_USART0),cadena);
	
	
	//Enable_global_interrupt();
	
} 
*/
__attribute__((__interrupt__))
static void tc_irq(void)
{
  // Increment the ms seconds counter
  tc_tick++;

  // Clear the interrupt flag. This is a side effect of reading the TC SR.
  tc_read_sr(EXAMPLE_TC, TC_CHANNEL);

  // Toggle a GPIO pin (this pin is used as a regular GPIO pin).
  if (tc_tick>=1000)
  {		
	   gpio_tgl_gpio_pin(LED0_GPIO);
		  tc_tick =0;
		  segundos++;
		  if (segundos>=60)
		  {
			  segundos=0;
			  minutos++;
			  gana1=0;
			  if (minutos>=60)
			  {
				  minutos=0;
				  horas++;
				  gana2=0;
				  if (horas>=24)
				  {
					  horas=0;
				  }
			  }
		  }  
	 
  }
  

}

 volatile avr32_tc_t *tc =  (&AVR32_TC);
// Main function
int main(void)
{
  U32 i;
  
    // Set CPU and PBA clock
    pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);
  /*
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
  //usart_write_line(EXAMPLE_USART, "Hello, this is the AVR UC3 MCU saying hello!\r\n");

  // Press enter to continue.
  //while (usart_get_echo_line(EXAMPLE_USART) == USART_FAILURE);  // Get and echo characters until end of line.

  //usart_write_line(EXAMPLE_USART, "Goodbye.\r\n");
  
  
  */
  
  //volatile avr32_tc_t *tc = EXAMPLE_TC;
  //volatile avr32_tc_t *tc =  (&AVR32_TC);


  // Options for waveform genration.
  static const tc_waveform_opt_t WAVEFORM_OPT =
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
	  .tcclks   = TC_CLOCK_SOURCE_TC3         // Internal source clock 3, connected to fPBA / 8.
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




  gpio_enable_gpio_pin(LED0_GPIO);
  gpio_enable_gpio_pin(LED1_GPIO);
  gpio_enable_gpio_pin(LED2_GPIO);
  gpio_enable_gpio_pin(LED3_GPIO);

  et024006_Init( FOSC0, FOSC0 );

#if BOARD == EVK1105
  /* PWM is fed by PBA bus clock which is by default the same
   * as the CPU speed. We set a 0 duty cycle and thus keep the
   * display black*/
  tft_bl_init();
#elif BOARD == EVK1104 || BOARD == UC3C_EK
  gpio_set_gpio_pin(ET024006DHU_BL_PIN);
#endif

  // Clear the display i.e. make it black
  et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
  
  //Interrupciones
  
  Disable_global_interrupt();

  INTC_init_interrupts();
  INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);
  init_Usart_DMA_RX();
  init_SD_DMA_RX();

  Enable_global_interrupt();
  
  tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.

  // Set the compare triggers.
  // Remember TC counter is 16-bits, so counting second is not possible with fPBA = 12 MHz.
  // We configure it to count ms.
  // We want: (1/(fPBA/8)) * RC = 0.001 s, hence RC = (fPBA/8) / 1000 = 1500 to get an interrupt every 1 ms.
  tc_write_rc(tc, TC_CHANNEL, 1500);            // Set RC value.

  tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

  // Start the timer/counter.
  tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.
  


  while(pwm_channel6.cdty < pwm_channel6.cprd)
  {
	  pwm_channel6.cdty++;
	  pwm_channel6.cupd = pwm_channel6.cdty;
	  //pwm_channel6.cdty--;
	  pwm_async_update_channel(AVR32_PWM_ENA_CHID6, &pwm_channel6);
	  delay_ms(10);
  }
  
  
  // Initialize SD/MMC driver resources: GPIO, SPI and SD/MMC.
  sd_mmc_resources_init();

  // Wait for a card to be inserted
  //while (!sd_mmc_spi_mem_check());
 
 
 uint8_t hayClave=0;
  uint32_t sector =10;
  char disp[50];
  char clave[7];
  usart_write_line(&AVR32_USART0,"Presione Left \n");
  usart_write_line(&AVR32_USART0,"Presione Right \n");
  usart_write_line(&AVR32_USART0,"Presione Enter \n");
  while (1)
  {
	  if (debounce2(QT1081_TOUCH_SENSOR_LEFT))
	  {
		  CLR_disp();
		  
		   pdca_load_channel( PDCA_CHANNEL_USART_EXAMPLE,
		   &password[0],//RAM
		   6);
		   pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_USART_EXAMPLE);
		   //pdca_enable(PDCA_CHANNEL_USART_EXAMPLE);
		  
		  pdca_channel_usart =(volatile avr32_pdca_channel_t*) pdca_get_handler(PDCA_CHANNEL_USART_EXAMPLE); // get the correct PDCA channel pointer
		  pdca_channel_usart->cr = AVR32_PDCA_TEN_MASK; // Enable RX PDCA transfer first
		  end_of_transfer=0;
		  et024006_PrintString("Recibiendo Clave :", (const unsigned char *)&FONT6x8, 80, 50, GREEN, -1);
		  while (!end_of_transfer)
		  {
		  }
		  et024006_PrintString("******", (const unsigned char *)&FONT6x8, 220, 50, GREEN, -1);
		  et024006_PrintString("Clave Recibida", (const unsigned char *)&FONT6x8, 80, 70, GREEN, -1);
		  hayClave=1;
		  
	  }
	  
	  if (debounce2(QT1081_TOUCH_SENSOR_RIGHT))
	  {
		  CLR_disp();
		  
			sprintf(disp,"%u",sector);
			end_of_transfer=0;
			
			
				if (!hayClave)
				{
					CLR_disp();
					et024006_PrintString(" ******ERROR no hay PASSWORD*******", (const unsigned char *)&FONT6x8, 80, 50, RED, -1);
					et024006_PrintString(" ****** Resetee la EVK pls *******", (const unsigned char *)&FONT6x8, 80, 90, RED, -1);
					
					while(1);
					
				}
			
			hayClave=0;
				  
				  et024006_PrintString("Almacenando Clave", (const unsigned char *)&FONT6x8, 80, 50, GREEN, -1);
				  et024006_PrintString(disp, (const unsigned char *)&FONT6x8, 80, 60, GREEN, -1);	  
					 if (!sd_mmc_spi_mem_check() )
					 {
						 CLR_disp();
						 et024006_PrintString(" ******ERROR no hay SD*******", (const unsigned char *)&FONT6x8, 80, 50, RED, -1);
						 et024006_PrintString(" ****** Resetee la EVK pls*******", (const unsigned char *)&FONT6x8, 80, 90, RED, -1);
						 while(1);
					 }
					 sd_mmc_spi_write_open(sector++);
					  sd_mmc_spi_write_sector_from_ram(&password[0]);
					  sd_mmc_spi_write_close();
				  
				  sd_mmc_spi_get_capacity();
				       int cap=capacity>>20;
				       sprintf(disp,"%d MB",cap);
				       et024006_PrintString(disp,(const unsigned char*)&FONT8x8,80,70,BLUE,-1);
				  if (sector>20)
				  {
					  sector=10;
				  }
			 
	  }
	  
	  if (debounce2(QT1081_TOUCH_SENSOR_ENTER))
	  {
		  CLR_disp();
		  et024006_PrintString("Claves", (const unsigned char *)&FONT6x8, 80, 50, GREEN, -1);
		  
		 
		  
		  // Read the first sectors number 1, 2, 3 of the card
		  for(int j = 10; j <= 20; j++)
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
					  getClave(&clave[0]);
					  if (clave[0]==0x00 && j==10)
					  {
						  CLR_disp();
						  et024006_PrintString(" ******ERROR no hay claves en la SD*******", (const unsigned char *)&FONT6x8, 80, 50, RED, -1);
						  et024006_PrintString(" ****** Resetee la EVK pls *******", (const unsigned char *)&FONT6x8, 80, 90, RED, -1);
						  while(1);
					  }
					  et024006_PrintString(clave, (const unsigned char *)&FONT6x8, 80, 60+j*8, GREEN, -1);
					  sprintf(disp,"%u",j);
					  et024006_PrintString(disp, (const unsigned char *)&FONT6x8, 130, 60+j*8, GREEN, -1);
	
			  }
		  
			}
	  }
	  
	  
	  if (debounce2(QT1081_TOUCH_SENSOR_DOWN))
	  {
		  uint32_t sector2=10;
		  char borrar[512]={0};
		  if (!sd_mmc_spi_mem_check() )
		  {
			  CLR_disp();
			  et024006_PrintString(" ******ERROR no hay SD*******", (const unsigned char *)&FONT6x8, 80, 50, RED, -1);
			  et024006_PrintString(" ****** Resetee la EVK pls*******", (const unsigned char *)&FONT6x8, 80, 90, RED, -1);
			  while(1);
		  }
		  for (; sector2<=20; sector2++)
		  {
			  sd_mmc_spi_write_open(sector2);
			  sd_mmc_spi_write_sector_from_ram(&borrar[0]);
			  sd_mmc_spi_write_close();
		  }
		  
		  
		  CLR_disp();
		  et024006_PrintString(" ****** SD CARD Borrada*******", (const unsigned char *)&FONT6x8, 80, 50, RED, -1);
		sector=10;
	  }
	  
	  
	  
	  
	  }
  
  }



void CLR_disp(void)
{
	// Clear the display i.e. make it black
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
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
	sd_mmc_spi_init(spiOptions, PBA_HZ);
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
		/* Select peripheral - data is transmitted on USART RX line */
		.pid = AVR32_PDCA_PID_USART0_RX,
		/* Select size of the transfer */
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE,

		/* Memory address */
		.addr = (void*)password,
		/* Transfer counter */
		.size = sizeof(password), //6 

		/* Next memory address */
		.r_addr = NULL,
		/* Next transfer counter */
		.r_size = 0,
	};

	/* Assign GPIO pins to USART. */
	gpio_enable_module(usart_gpio_map,
			sizeof(usart_gpio_map) / sizeof(usart_gpio_map[0]));

	/* Initialize the USART in RS232 mode. */
	usart_init_rs232(EXAMPLE_USART, &usart_options,FOSC0);

	//usart_write_line(EXAMPLE_USART, "PDCA Example.\r\n");

	/* Initialize the PDCA channel with the requested options. */
	pdca_init_channel(PDCA_CHANNEL_USART_EXAMPLE, &PDCA_OPTIONS);
	
	INTC_register_interrupt(&pdca_int_handler_USART, AVR32_PDCA_IRQ_2,
	AVR32_INTC_INT3);
    
	/* Enable PDCA interrupt each time the reload counter reaches zero, i.e.
	 * each time half of the ASCII animation (either anim1 or anim2) is
	 * transferred. */
	//pdca_enable_interrupt_reload_counter_zero(PDCA_CHANNEL_USART_EXAMPLE);
	
	//pdca_enable_interrupt_transfer_complete(PDCA_CHANNEL_USART_EXAMPLE);

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


void getClave(char* clave){
		for( int i = 0; i < 6; i++)
		{
			
			clave[i]= ( (U8)(*(ram_buffer + i)));
		}
		clave[6]='\0';
}

// Software wait
void wait(void)
{
	volatile int i;
	for(i = 0 ; i < 5000; i++);
}

