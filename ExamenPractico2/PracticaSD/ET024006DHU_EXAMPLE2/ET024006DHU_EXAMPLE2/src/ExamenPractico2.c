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
 
  uint32_t sector =10;
  char disp[10];
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
		  
	  }
	  
	  if (debounce2(QT1081_TOUCH_SENSOR_RIGHT))
	  {
		  CLR_disp();
		  
			sprintf(disp,"%u",sector);
			end_of_transfer=0;
			
				  
				  et024006_PrintString("Almacenando Clave", (const unsigned char *)&FONT6x8, 80, 50, GREEN, -1);
				  et024006_PrintString(disp, (const unsigned char *)&FONT6x8, 80, 60, GREEN, -1);	  
					  sd_mmc_spi_write_open(sector++);
					  sd_mmc_spi_write_sector_from_ram(&password[0]);
					  sd_mmc_spi_write_close();
				  
				
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
					  et024006_PrintString(clave, (const unsigned char *)&FONT6x8, 80, 60+j*7, GREEN, -1);
					  sprintf(disp,"%u",j);
					  et024006_PrintString(disp, (const unsigned char *)&FONT6x8, 120, 60+j*7, GREEN, -1);
	
			  }
		  
			}
			else{
				 et024006_PrintString("No hay SD insertada", (const unsigned char *)&FONT6x8, 80, 60, GREEN, -1);
			}
	  }
	  
	  }
  
  }



void Rectangulo(void){
	enter=1;
	// Rectangulo rojo movido por las teclas
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
	et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
	while(enter);//interrupcion de enter saca de aqui
}

void Matrix(void){
	#define MAX_COL 6  //numero de columnas a mostrar
	#define COL_INICIAL 40 //Posicion en X en la cual iniciarlas
	//Animacion de letras Matrix style
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
	enter=1;
	char* col;
	col= (char*)malloc(sizeof(char)*22*MAX_COL); 
	
	
	char* colPointers[MAX_COL];
	for (uint16_t i=0; i<MAX_COL; i++)
	{
		colPointers[i]=&col[i*22];
		
	}
	
	

  while(enter){
	  char add=0;
	  char add2=' ';
	  char add3= ' ';
	  char add4= ' ';
	  
	 
	  
	  
	  for (uint16_t i=1; i<MAX_COL+1 ;i++)
	  {
		   //caracteres visibles del ASCII
		  add2= (rand()%(126-32))+32;
		  add3= (rand()%(126-32))+32;
		   if (add)
		   {
			   add4=add2;
		   }else add4= add3;
		    if (rand()%2==1)
		    {
			    add ^= 1 << 0;//mas prbabilidad de que haya espacios
		    }else add4= ' ';
		   
		  et024006_DrawFilledRect(i*COL_INICIAL , 0, 30, ET024006_HEIGHT, BLACK );
		  addToArr(colPointers[i-1],add4,21);
		  verticalPrint(colPointers[i-1],21,i*COL_INICIAL,GREEN);
		  delay_ms(50);
		  
	  }
	 

}
	free(col);
}


void Imagen(void){
	CLR_disp();
	
	enter=1;
	 // Dibuja a Iggy Azalea
	 et024006_PutPixmap(Iggy, 320, 0, 0, 0, 0, 320, 240);
	 while (enter)
	 {
	 }
	
	
}


void addToArr(char * str, char add, size_t size){
	
	for (uint16_t i=size;i>0;i--)
	{
		str[i]=str[i-1];
		
	}
	str[0]=add;
		
}


void centerTriangle(uint16_t x, uint16_t y, uint16_t size, et024006_color_t color){
	
	
	drawTriangle(x-size/2,y+size*0.866/2,size,WHITE);
	
	
}

void Triangulos(void){
	enter=1;
	CLR_disp();
	//uint16_t j=20;
	/*
	for (uint16_t i=200; i>20;i-=10)
	{
		j-=10;
		drawTriangle(i, i-40,i/2,WHITE);
	}
	*/
	
	for (uint16_t i=10; i<130; i+=10)
	{
		
		centerTriangle(160,80,i,WHITE);
		
	}
	
	for (uint16_t i=10; i<100; i+=10)
	{
		
		centerTriangle(60,20,i,WHITE);
		
	}
	
	for (uint16_t i=10; i<100; i+=10)
	{
		
		centerTriangle(320-60,20,i,WHITE);
		
	}
	
	while (enter)
	{
	}
	
	
	
}





void AjedrezCrece(void){
	uint16_t size=1,j=1,k=1,rnd=0;
	
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
	enter=1;
	while(enter){
	if(debounce2(QT1081_TOUCH_SENSOR_UP)){
	size<<=1;
	size=(size>32)?(size>>1):size;
	}
	if(debounce2(QT1081_TOUCH_SENSOR_DOWN)){
	size>>=1;
	size=(size<1)?(1):size;
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
	}
	for( k=0; k<8; k++){
	for(j=0;j<8;j++){
	et024006_DrawFilledRect(k*size,j*size,size,size,(k+j)%2==0?BLACK:WHITE); //Ej 2
	
	}
	//et024006_DrawFilledRect(k*size,0*size,size,size,(k+0)%2==0?BLACK:WHITE);
	//et024006_DrawFilledRect(k*size,1*size,size,size,(k+1)%2==0?BLACK:WHITE);
	}
	//delay_s(2);
	}
	
	
	
	
}


void Caracol(void){
	enter=1;
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE );
	et024006_DrawFilledCircle(170,120,5,BLACK,TFT_QUADRANT3);et024006_DrawFilledCircle(170,120,2,WHITE,TFT_QUADRANT3);delay_ms(50);
	et024006_DrawFilledCircle(170,120,5,BLACK,TFT_QUADRANT0);et024006_DrawFilledCircle(170,120,2,WHITE,TFT_QUADRANT0);delay_ms(50);	
	et024006_DrawFilledCircle(170,125,10,BLACK,TFT_QUADRANT1);et024006_DrawFilledCircle(170,125,7,WHITE,TFT_QUADRANT1);delay_ms(50);
	et024006_DrawFilledCircle(175,125,15,BLACK,TFT_QUADRANT2);et024006_DrawFilledCircle(175,125,12,WHITE,TFT_QUADRANT2);
	delay_ms(50);
	et024006_DrawFilledCircle(175,115,25,BLACK,TFT_QUADRANT3);et024006_DrawFilledCircle(175,115,22,WHITE,TFT_QUADRANT3);
	delay_ms(50);
	et024006_DrawFilledCircle(160,115,40,BLACK,TFT_QUADRANT0);et024006_DrawFilledCircle(160,115,37,WHITE,TFT_QUADRANT0);
	delay_ms(50);
	et024006_DrawFilledCircle(160,140,65,BLACK,TFT_QUADRANT1);et024006_DrawFilledCircle(160,140,62,WHITE,TFT_QUADRANT1);
	delay_ms(50);
	et024006_DrawFilledCircle(200,140,105,BLACK,TFT_QUADRANT2);et024006_DrawFilledCircle(200,140,102,WHITE,TFT_QUADRANT2);
	delay_ms(50);
	et024006_DrawFilledCircle(200,75,170,BLACK,TFT_QUADRANT3);et024006_DrawFilledCircle(200,75,167,WHITE,TFT_QUADRANT3);
	delay_ms(50);
	
	while (enter)
	{
	}
	
}




void AjedrezRandom(void){

	
	uint16_t size=1,j=1,k=1,rnd=0;
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
	enter=1;
	while(enter){
	if(debounce2(QT1081_TOUCH_SENSOR_UP)){
	size<<=1;
	size=(size>32)?(size>>1):size;
	}
	if(debounce2(QT1081_TOUCH_SENSOR_DOWN)){
	size>>=1;
	size=(size<1)?(1):size;
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
	}
	for( k=0; k<8; k++){
	for(j=0;j<8;j++){
	//et024006_DrawFilledRect(k*size,j*size,size,size,(k+j)%2==0?BLACK:WHITE); //Ej 2
	rnd=255*rand();
	et024006_DrawFilledRect(k*size,j*size,size,size,(2*rnd)|((4*rnd)<<5)|((2*rnd)<<11)); //Ej 5
	}
	//et024006_DrawFilledRect(k*size,0*size,size,size,(k+0)%2==0?BLACK:WHITE);
	//et024006_DrawFilledRect(k*size,1*size,size,size,(k+1)%2==0?BLACK:WHITE);
	}
	//delay_s(2);
	
	
}
}






void verticalPrint(char *str,size_t size, uint16_t x, et024006_color_t color){
	//imprime una string en vertical
	char str2[]={' ','\0'};
	uint16_t i=0;
	
	for (uint16_t y=0;y<230;y+=10)
	{
		str2[0]=str[i++];
		et024006_PrintString(str2, (const unsigned char *)&FONT8x8, x, y, color, -1);
		if (i>=size)
		{
			break;
		}
	}
}






void CLR_disp(void)
{
	// Clear the display i.e. make it black
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, BLACK );
}

void Ej1(void)
{
	//320,240
	//Tablero 240,240
	//cuadros de 240/8=30 30x30
	enter=1;
	U32 i,j,c;
	c=WHITE;
	for(i=0; i<8; i++)
	{
		if(i%2) c=BLACK;
		else    c=WHITE;
		for(j=0;j<8;j++)
		{
			et024006_DrawFilledRect(j*30,i*30,30,30,c);
			if(c==BLACK) c=WHITE;
			else         c=BLACK;
			delay_ms(10);
		}
	}
	while(enter){};
}

void Ej4(void)
{
	enter=1;
	while(enter)
	{
		et024006_DrawPixel(rand()%ET024006_WIDTH,rand()%ET024006_HEIGHT,RGB(rand()%31, rand()%63 , rand()%31 ));
		//delay_ms(10);
	}
}

void Ej7(void)
{
	U32 i;
	enter=1;
	//320,240
	//Colores Arriba
	et024006_DrawFilledRect(0,0,45,150,WHITE);
	et024006_DrawFilledRect(45,0,45,150,RGB(31,63,0)); //amarillo
	et024006_DrawFilledRect(45*2,0,45,150,RGB(0,63,31));//azul 1
	et024006_DrawFilledRect(45*3,0,45,150,RGB(0,63,0));//verde
	et024006_DrawFilledRect(45*4,0,45,150,RGB(31,0,31));//violeta
	et024006_DrawFilledRect(45*5,0,45,150,RGB(31,0,0));//rojo
	et024006_DrawFilledRect(45*6,0,45,150,RGB(0,0,31));//azul 2
	//Colores Enmedio
	et024006_DrawFilledRect(0,150,45,20,RGB(0,0,31));//azul 2
	et024006_DrawFilledRect(45,150,45,20,RGB(31,0,31));//violeta
	et024006_DrawFilledRect(45*2,150,45,20,RGB(31,63,0)); //amarillo
	et024006_DrawFilledRect(45*3,150,45,20,RGB(31,0,0));//rojo
	et024006_DrawFilledRect(45*4,150,45,20,RGB(0,63,31));//azul 1
	et024006_DrawFilledRect(45*5,150,45,20,BLACK);
	et024006_DrawFilledRect(45*6,150,45,20,WHITE);
	//difuminados blanconegro
	for(i=0;i<190;i++)
	{
		et024006_DrawVertLine(i,170,30,RGB(i*31/190,i*63/190,i*31/190));
	}
	//difuminado colores
	for(i=0;i<10;i++) //rojo a amarillo
	{
		et024006_DrawVertLine(i,170,30,RGB(31,i*6,0));
	}
	for(i=0;i<10;i++) //amarillo a verde
	{
		et024006_DrawVertLine(i+10,170,30,RGB((10-i)*3,63,0));
	}
	for(i=0;i<10;i++) //verde a azul1
	{
		et024006_DrawVertLine(i+20,170,30,RGB(0,63,i));
	}
	for(i=0;i<10;i++) //azul 1 a azul2
	{
		et024006_DrawVertLine(i+30,170,30,RGB(0,(10-i)*6,31));
	}
	for(i=0;i<10;i++) // azul 2 a violeta
	{
		et024006_DrawVertLine(i+40,170,30,RGB(i*3,0,31));
	}
	//difuminado cuadros
	for(i=0;i<12;i++)
	{
		et024006_DrawFilledRect(i*15,200,15,40,RGB(i*31/12,i*63/12,i*31/12));
	}
	//cuadro negro
	et024006_DrawFilledRect(180,200,40,40,RGB(i*31/12,i*63/12,i*31/12));
	while (enter)
	{
	}
}

void Ej11(void)
{
	//320,240
	//et024006_DrawLine(x,y,x2,y2,c);
	// lineas apoyo
	enter=1;
	et024006_DrawLine(160,120,0,240,RGB(0,63,31));
	delay_ms(10);
	et024006_DrawLine(160,120,320,240,RGB(0,63,31));
	delay_ms(1);
	et024006_DrawLine(120,0,0,240,RGB(0,63,31));
	delay_ms(10);
	et024006_DrawLine(120,0,320,240,RGB(0,63,31));
	delay_ms(10);
	et024006_DrawLine(200,150,0,240,RGB(0,63,31));
	delay_ms(10);
	et024006_DrawLine(130,130,320,240,RGB(0,63,31));
	delay_ms(10);
	//Lineas negras
	et024006_DrawLine(160,120,130,130,WHITE);
	delay_ms(10);
	et024006_DrawLine(160,120,200,150,WHITE);
	delay_ms(10);
	et024006_DrawLine(120,0,130,20,WHITE);
	delay_ms(10);
	et024006_DrawLine(120,0,200,36,WHITE);
	delay_ms(10);
	et024006_DrawLine(200,150,180,159,WHITE);
	delay_ms(10);
	et024006_DrawLine(130,130,180,159,WHITE);
	delay_ms(10);
	//Lineas Verticales
	et024006_DrawLine(120,0,120,160,WHITE);
	delay_ms(10);
	et024006_DrawLine(130,20,130,130,WHITE);
	delay_ms(10);
	et024006_DrawLine(200,36,200,150,WHITE);
	delay_ms(10);
	while (enter)
	{
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

void reloj(void){
	//static int lastSegundos;
	char disp[20];
	sprintf(disp,"%d: %d: %d",horas,minutos,segundos);
	
	
	
	if (tc_tick == 500)//cada 500ms refresca la pantall
	{
		et024006_DrawFilledRect(80,50,80,40, BLACK);
		et024006_PrintString(disp, (const unsigned char *)&FONT8x16, 80, 50, BLUE, -1);
	}
	while(enter)
	{
		
		//gpio_disable_pin_interrupt(QT1081_TOUCH_SENSOR_ENTER);
		gpio_disable_pin_interrupt(QT1081_TOUCH_SENSOR_UP);
		gpio_disable_pin_interrupt(QT1081_TOUCH_SENSOR_DOWN);
		tc_stop(tc, TC_CHANNEL);  
		//Disable_global_interrupt();
		//delay_ms(100);
		
		
			
			if (debounce2(QT1081_TOUCH_SENSOR_UP))
			{
				horas++;
				if (horas>23)
				{
					horas=0;
				}
				sprintf(disp,"%d: %d: %d",horas,minutos,segundos);
				et024006_DrawFilledRect(80,50,80,40, BLACK);
				et024006_PrintString(disp, (const unsigned char *)&FONT8x16, 80, 50, BLUE, -1);
			}
			if (debounce2(QT1081_TOUCH_SENSOR_DOWN))
			{
				minutos++;
				if (minutos>59)
				{
					minutos=0;
				}
				sprintf(disp,"%d: %d: %d",horas,minutos,segundos);
				et024006_DrawFilledRect(80,50,80,40, BLACK);
				et024006_PrintString(disp, (const unsigned char *)&FONT8x16, 80, 50, BLUE, -1);
			}
			
			
		
		//Enable_global_interrupt();
		//enter=1;
		//enter ^= (1<<0);
	}
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_UP,GPIO_RISING_EDGE);
	gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_DOWN,GPIO_RISING_EDGE);
	tc_start(tc, TC_CHANNEL);  
	
	
	
}


void cronometro(void){
	
	static int mil=0;
	static int seg=0;
	static int minu=0;
	static int hor=0;
	
	#define POSX_CRON 120
	#define POSY_CRON 80
	
	char disp[20];
	sprintf(disp,"%d: %d: %d: %d",hor,minu,seg,mil);
	
	et024006_PrintString("C R O N O M E T R O \n\n     Press Enter \n       to Start", (const unsigned char *)&FONT8x16, 80, 5, BLUE, -1);
	
	et024006_PrintString("Press Enter \n to Stop \n\n Press Up \n to Reset", (const unsigned char *)&FONT8x16, 120, 120, BLUE, -1);
	
	if (resetTimer)
	{
		mil=0;
		seg=0;
		minu=0;
		hor=0;
		resetTimer=0;
		et024006_DrawFilledRect(POSX_CRON,POSY_CRON,120,40, BLACK);
		sprintf(disp,"%d: %d: %d: %d",hor,minu,seg,mil);
		et024006_PrintString(disp, (const unsigned char *)&FONT8x16, POSX_CRON, POSY_CRON, BLUE, BLACK);
	}
	
	
	
	if (tc_tick == 500)
	{
		et024006_DrawFilledRect(80,80,120,40, BLACK);
		et024006_PrintString(disp, (const unsigned char *)&FONT8x16, POSX_CRON, POSY_CRON, BLUE, -1);
	}
		

	while (go)
	{
		if ( tc_tick % 10 ==0 )//cada 10
		{
			mil++;
			if (mil>=100)
			{
				mil=0;
				seg++;
				if (seg>=60)
				{
					seg=0;
					minu++;
					if (minu>=60)
					{
						minu=0;
						hor++;
						if (hor>=24)
						{
							hor=0;
						}
					}
				}
			}
			et024006_DrawFilledRect(POSX_CRON,POSY_CRON,110,20, BLACK);
			sprintf(disp,"%d: %d: %d: %d",hor,minu,seg,mil);
			et024006_PrintString(disp, (const unsigned char *)&FONT8x16, POSX_CRON, POSY_CRON, BLUE, BLACK);
		}
		
	}
	
	
	
	
}

int puntuacionMin = 0;
int puntuacionHor = 0;
void PONG(void){
	#define WHOLE TFT_QUADRANT0 | TFT_QUADRANT1 | TFT_QUADRANT2 | TFT_QUADRANT3
	#define POSX_COUNTER1 280
	#define POSY_COUNTER1 20
	#define POSX_COUNTER2 40
	#define POSY_COUNTER2 20
	static uint32_t Barra1 = 10;
	static uint32_t Barra2 = 100;
	static uint32_t BolitaX= 60;
	static uint32_t BolitaY=60;
	static uint32_t VelX =1;
	static uint32_t VelY=1;
	//static int puntuacionMin = 0;
	//static int puntuacionHor = 0;
	int r=1;
	char disp[10];
	
	if (r==1)
	{
		puntuacionHor=horas;
		puntuacionMin=minutos;
		r=0;
	}
	
	
	et024006_DrawFilledCircle(BolitaX,BolitaY,5,BLACK,WHOLE);
	BolitaX+=VelX;
	BolitaY+=VelY;
	et024006_DrawFilledCircle(BolitaX,BolitaY,5,WHITE,WHOLE);
	delay_ms(3);
	
	et024006_DrawFilledRect(0,Barra1,10,40,BLACK);
	if ((BolitaX<160) & gana2)
	{
		Barra1 = BolitaY-20;
	}
	
	if (gana2==0)
	{
		Barra1 = Barra2;
	}
	
	et024006_DrawFilledRect(0,Barra1,10,40,WHITE);
	
	
	et024006_DrawFilledRect(310,Barra2,10,40,BLACK);
	if ((BolitaX>160) & (gana1))
	{
		Barra2 = BolitaY-20;
	}
	
	if (gana1==0)
	{
		Barra2= Barra1;
	}
	
	
	et024006_DrawFilledRect(310,Barra2,10,40,WHITE);
	
	
	
	if ((BolitaX==320-13) &&   (Barra2>= BolitaY-20) && (Barra2+20<=BolitaY) )
	{
		VelX=-1;
		
		
	}
	
	
	
	if (BolitaY==240-13)
	{
		
		VelY=-1;
		
		
		
		
	}
	
	if ((BolitaX==13) && (  (Barra1>=BolitaY-20) && (Barra1+20 <= BolitaY) ))
	{
		VelX=1;
	}
	
	if (BolitaY==5)
	{
		VelY=1;
	}
	
	if (BolitaX >= 310)
	{//si llega aqui, perdio el de minutos
		et024006_DrawFilledCircle(BolitaX,BolitaY,5,BLACK,WHOLE);
		BolitaX=16;
		gana1=1;
		verticalPrint("SCORE",5,160,WHITE);
		delay_ms(500);
		et024006_DrawFilledRect(160,0,10,50,BLACK);
		puntuacionMin++;
		/*sprintf(disp,"%d",puntuacionHor);
		et024006_DrawFilledRect(POSX_COUNTER2,POSY_COUNTER2,10,10, BLACK);
		et024006_PrintString(disp,(const unsigned char *)&FONT8x16, POSX_COUNTER2, POSY_COUNTER2, WHITE, BLACK);
		
		puntuacionMin++;
		
		sprintf(disp,"%d",puntuacionMin);
		et024006_DrawFilledRect(POSX_COUNTER1,POSY_COUNTER1,10,10, BLACK);
		et024006_PrintString(disp,(const unsigned char *)&FONT8x16, POSX_COUNTER1, POSY_COUNTER1, WHITE, BLACK);*/
	}
	
	if (BolitaX <= 12)
	{//si llega aqui, perdio el de horas
		et024006_DrawFilledCircle(BolitaX,BolitaY,5,BLACK,WHOLE);
		BolitaX=310;
		VelX = -1;
		gana2=1;
		puntuacionHor++;
		/*
		sprintf(disp,"%d",puntuacionHor);
		et024006_DrawFilledRect(POSX_COUNTER2,POSY_COUNTER2,10,10, BLACK);
		et024006_PrintString(disp,(const unsigned char *)&FONT8x16, POSX_COUNTER2, POSY_COUNTER2, WHITE, BLACK);
		
		
		
		sprintf(disp,"%d",puntuacionMin);
		et024006_DrawFilledRect(POSX_COUNTER1,POSY_COUNTER1,10,10, BLACK);
		et024006_PrintString(disp,(const unsigned char *)&FONT8x16, POSX_COUNTER1, POSY_COUNTER1, WHITE, BLACK);
		*/
	}
	
	
	
	//puntuacionHor = horas;
	//puntuacionMin = minutos;
	sprintf(disp,"%d",puntuacionHor);
	et024006_DrawFilledRect(POSX_COUNTER2,POSY_COUNTER2,10,10, BLACK);
	et024006_PrintString(disp,(const unsigned char *)&FONT8x16, POSX_COUNTER2, POSY_COUNTER2, WHITE, BLACK);
	
	
	
	sprintf(disp,"%d",puntuacionMin);
	et024006_DrawFilledRect(POSX_COUNTER1,POSY_COUNTER1,10,10, BLACK);
	et024006_PrintString(disp,(const unsigned char *)&FONT8x16, POSX_COUNTER1, POSY_COUNTER1, WHITE, BLACK);
}


void recibirImagen(uint32_t sector){
	
	#define image_X 80
	#define image_Y 80
	uint16_t imagen[6400];
		uint32_t i,j;
		//usart_write_line(EXAMPLE_USART, "Listo para recibir imagen\n");
		uint16_t r,g,b;
		uint16_t color_bw;
		uint16_t color_gray;
		uint16_t color;
		uint16_t d=0;

		for(i=0; i<image_Y; i++)
		{
			for(j=0; j<image_X;j++)
			{
				color=usart_getchar(EXAMPLE_USART)<<8;
				color+=usart_getchar(EXAMPLE_USART);
				et024006_DrawPixel(j,i,color);
				imagen[d++]=color;
				r=color & 0x001F;
				g=(color>>6) & 0x001F;
				b=(color>>11) & 0x001F;
				color_gray=(   (((r+g+b)/3)<<11) + (((r+g+b)/3)<<6) + ((r+g+b)/3));
				et024006_DrawPixel(j,i+image_Y,color_gray);
				if(((r+g+b)/3)>15)
				{
					color_bw=WHITE;
				}
				else
				{
					color_bw=BLACK;
				}
				et024006_DrawPixel(j,i+2*image_Y,color_bw);
				/*
				if(i==0 && j==0)
				usart_write_line(EXAMPLE_USART, "Recibiendo imagen...\n");*/
				
			}
		
			
		}
		usart_write_line(EXAMPLE_USART, "Escribiendo a SD\n");
		saveImage(&imagen[0],sector);
		usart_write_line(EXAMPLE_USART, "Imagen recibida\n");
}




void loadImage(uint8_t* buffer,uint32_t sector){
	
	
	
	for(int i =0;i<25;i++){
	sd_mmc_spi_write_open(sector+i);
	sd_mmc_spi_read_sector_to_ram(buffer+512*i);
	sd_mmc_spi_read_close();
	}
	
}

void displayImage(uint32_t sector){
	#define image_X 80
	#define image_Y 80
	
		uint32_t i,j;
		//usart_write_line(EXAMPLE_USART, "Listo para recibir imagen\n");
		uint16_t r,g,b;
		uint16_t color_bw;
		uint16_t color_gray;
		uint16_t color;
		
		uint8_t imagen[12800];
		uint16_t d=0;
		
		loadImage(&imagen[0],sector);

		for(i=0; i<image_Y; i++)
		{
			for(j=0; j<image_X;j++)
			{
				color=imagen[d++];
				color+=imagen[d++]<<8;
				
				
				
		
				
				
				et024006_DrawPixel(j,i,color);
				
				r=color & 0x001F;
				g=(color>>6) & 0x001F;
				b=(color>>11) & 0x001F;
				color_gray=(   (((r+g+b)/3)<<11) + (((r+g+b)/3)<<6) + ((r+g+b)/3));
				et024006_DrawPixel(j,i+image_Y,color_gray);
				if(((r+g+b)/3)>15)
				{
					color_bw=WHITE;
				}
				else
				{
					color_bw=BLACK;
				}
				et024006_DrawPixel(j,i+2*image_Y,color_bw);
				/*
				if(i==0 && j==0)
				usart_write_line(EXAMPLE_USART, "Recibiendo imagen...\n");*/
				
			}
		
			
		}
}


void deleteImage(uint32_t sector){
	uint8_t buffer2[512];
	uint32_t inicio = sector +25;
	
	
	memset(&buffer2[0],0xFF,512);
	
	for (;sector<inicio;sector++)
	{
			
		sd_mmc_spi_write_open(sector);
		sd_mmc_spi_write_sector_from_ram(&buffer2[0]);
		sd_mmc_spi_write_close();
	}

	
}


void saveImage(uint16_t* buffer,uint32_t sector){
	uint8_t buffer2[512];
	uint32_t inicio = sector +25;
	uint32_t j=0;
	for (;sector<inicio;sector++)
	{
		
		
		for (int i=0; i<512;)
		{
			buffer2[i++]=(uint8_t)buffer[j];
			buffer2[i++]=(uint8_t)(buffer[j]>>8);
			j++;
		}
		
		
		sd_mmc_spi_write_open(sector);
		sd_mmc_spi_write_sector_from_ram(&buffer2[0]);
		sd_mmc_spi_write_close();
	}

	
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
		clave[7]='\0';
}

// Software wait
void wait(void)
{
	volatile int i;
	for(i = 0 ; i < 5000; i++);
}

