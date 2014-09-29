
#include <asf.h>

#include "conf_gpio_peripheral_bus_example.h"
void display(volatile int32_t numero);
void inicializa_PM(void);
uint32_t debounce2( uint32_t GPIO_PIN );

void Lab1(void);
void Lab3(void);
void displayPrimo(void);
void displayA1(void);
void displayOnOff(void);
void Actividad1(void);
void Actividad2(void);
void tecla_lrc_isr(void);
void Actividad3(void);
void touch_button_isr(void);

#define UP 54
#define DOWN 55
#define LEFT 56
#define RIGHT 57
#define CENTER 58

#define GPIO_LED0 59
#define GPIO_LED1 60
#define GPIO_LED2 5
#define GPIO_LED3 6
volatile uint32_t center;
volatile uint32_t vel=2;




__attribute__ ((__interrupt__)) void touch_button_isr(void){
	
	eic_clear_interrupt_line(&AVR32_EIC, QT1081_EIC_EXTINT_INT);
	// UP
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_UP))
	{
		//gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP);
		//gpio_tgl_gpio_pin(LED0_GPIO);
		pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
			   pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
			   
			   pm_pll_setup(&AVR32_PM,0,vel++,1,0,16); // lockcount in main clock for the PLL wait lock

			   //_______________________________________________________________________________
			   // Establece la frecuencia de salida del PLL
			   pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
			   //_______________________________________________________________________________
			   //Habilita el PLL 0
			   pm_pll_enable(&AVR32_PM,0);
			   //_______________________________________________________________________________
			   //Espera a que se establesca el PLL
			   pm_wait_for_pll0_locked(&AVR32_PM) ;
			   //_______________________________________________________________________________
			   // Set one wait-state (WS) for flash controller
			   flashc_set_wait_state(1);

			   //habilita la salida del PLL0 con 2 y el OSC0 con 1
			   pm_switch_to_clock(&AVR32_PM, 2);
			   if (vel>=6)
			   {
				   vel=6;
			   }
			   
			   
	}
	// DOWN
	if(gpio_get_pin_value(QT1081_TOUCH_SENSOR_DOWN))
	{
		
		//gpio_tgl_gpio_pin(LED1_GPIO);
		pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
			   pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
			   
			   pm_pll_setup(&AVR32_PM,0,vel--,1,0,16); // lockcount in main clock for the PLL wait lock

			   //_______________________________________________________________________________
			   // Establece la frecuencia de salida del PLL
			   pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
			   //_______________________________________________________________________________
			   //Habilita el PLL 0
			   pm_pll_enable(&AVR32_PM,0);
			   //_______________________________________________________________________________
			   //Espera a que se establesca el PLL
			   pm_wait_for_pll0_locked(&AVR32_PM) ;
			   //_______________________________________________________________________________
			   // Set one wait-state (WS) for flash controller
			   flashc_set_wait_state(1);

			   //habilita la salida del PLL0 con 2 y el OSC0 con 1
			   pm_switch_to_clock(&AVR32_PM, 2);
			   
			  if (vel<=2)
			  {
				  vel=2;
			  }
	}
}




__attribute__ ((__interrupt__)) void tecla_lrc_isr(void){//handler teclas left, right o center
	
	
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER))
	{
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER);
	}
	

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_LEFT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_LEFT);
		pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); 
		
		while(gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER)==0){
			
			gpio_tgl_gpio_pin(LED3_GPIO);
			delay_s(50);
			
		}
		display(0);
	}

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_RIGHT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_RIGHT);
		pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); 
		while(gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER)==0){
			displayPrimo();
			delay_s(50);
			
		}
	}
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP))
	{
				gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP);
		       pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
			   pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
			   
			   pm_pll_setup(&AVR32_PM,0,2,1,0,16); // lockcount in main clock for the PLL wait lock

			   //_______________________________________________________________________________
			   // Establece la frecuencia de salida del PLL
			   pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
			   //_______________________________________________________________________________
			   //Habilita el PLL 0
			   pm_pll_enable(&AVR32_PM,0);
			   //_______________________________________________________________________________
			   //Espera a que se establesca el PLL
			   pm_wait_for_pll0_locked(&AVR32_PM) ;
			   //_______________________________________________________________________________
			   // Set one wait-state (WS) for flash controller
			   flashc_set_wait_state(1);

			   //habilita la salida del PLL0 con 2 y el OSC0 con 1
			   pm_switch_to_clock(&AVR32_PM, 2);
			   
			   while (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER)==0)
			   {
				   
				   displayA1();
				    /*for (uint32_t j =0; j<18000; j++)//300ms 12Mhz
				    {
						delay_ms(10);
				    }*/
					delay_s(50);
				   
			   }
			   display(0);
			   
		
	}
	
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN))
	{
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN);
		pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
		pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
		
		pm_pll_setup(&AVR32_PM,0,6,1,0,16); // lockcount in main clock for the PLL wait lock

		//_______________________________________________________________________________
		// Establece la frecuencia de salida del PLL
		pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
		//_______________________________________________________________________________
		//Habilita el PLL 0
		pm_pll_enable(&AVR32_PM,0);
		//_______________________________________________________________________________
		//Espera a que se establesca el PLL
		pm_wait_for_pll0_locked(&AVR32_PM) ;
		//_______________________________________________________________________________
		// Set one wait-state (WS) for flash controller
		flashc_set_wait_state(1);

		//habilita la salida del PLL0 con 2 y el OSC0 con 1
		pm_switch_to_clock(&AVR32_PM, 2);
		
		while (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER)==0)
		{
			displayOnOff();
			
		}
		display(0);
		
		
	}
	
	
	
	

}






int main(void)
{
	 
	
	//Actividad2();
	Actividad1();
	//Actividad3();
	//Lab3();
	//Lab1();
	
	}

		
	
	
	void display(volatile int32_t numero){//muestra numero en los leds
	int8_t numerox = (int8_t)numero;
	if (numerox & (1 << 0)) //checa el bit0
	{
		gpio_clr_gpio_pin(LED3_GPIO);
	}else{
		gpio_set_gpio_pin(LED3_GPIO);
		
	}
	if (numerox & (1 << 1)) //checa el bit0
	{
		gpio_clr_gpio_pin(LED2_GPIO);
		}else{
		gpio_set_gpio_pin(LED2_GPIO);
		
	}
	if (numerox & (1<<2)) //checa el bit0
	{
		gpio_clr_gpio_pin(LED1_GPIO);
		}else{
		gpio_set_gpio_pin(LED1_GPIO);
		
	}
	if (numerox & (1<<3)) //checa el bit0
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		}else{
		gpio_set_gpio_pin(LED0_GPIO);
		
	}
	
	
	
	}

void displayOnOff(void){
	static uint32_t j=0;
	static int32_t SEL =4;
			
			
				   display(15);
				   delay_s(100);
				   display(0);
				   delay_s(100);
				   if (j>6) {
				   pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
				   pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
				   
				   pm_pll_setup(&AVR32_PM,0,SEL,1,0,16); // lockcount in main clock for the PLL wait lock

				   //_______________________________________________________________________________
				   // Establece la frecuencia de salida del PLL
				   pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
				   //_______________________________________________________________________________
				   //Habilita el PLL 0
				   pm_pll_enable(&AVR32_PM,0);
				   //_______________________________________________________________________________
				   //Espera a que se establesca el PLL
				   pm_wait_for_pll0_locked(&AVR32_PM) ;
				   //_______________________________________________________________________________
				   // Set one wait-state (WS) for flash controller
				   flashc_set_wait_state(1);

				   //habilita la salida del PLL0 con 2 y el OSC0 con 1
				   pm_switch_to_clock(&AVR32_PM, 2);
				   SEL-=2;
				   if (SEL<2)
				   {
					   SEL=6;
				   }
				j=0;
			}
			j++;
		
	
	
}





void Lab1(void){
	int32_t numero=0,numero1=8,numero2 = 0;
	
	while (1) {
		switch (numero) {
		case 0:

			/* Access with GPIO driver gpio.c with clear and set
			 * access. */
			//for (uint32_t i = 0; i < 1000; i += 4) {	
				if (debounce2(QT1081_TOUCH_SENSOR_RIGHT)) {
					//if(arrVal[3]){
					//delay_ms(500);
					//gpio_clr_gpio_pin(GPIO_PIN_EXAMPLE_2);
					numero1++;
					
					if (numero1>16)
					{
						numero1=15;
					}
					}
			//}
			//for (uint32_t i = 0; i < 1000; i += 4) {
				
				
				if (debounce2(QT1081_TOUCH_SENSOR_LEFT)) {
					//delay_ms(500);
					//gpio_clr_gpio_pin(GPIO_PIN_EXAMPLE_2);
					//if(arrVal[2]){
					numero1--;
					
					if (numero1<=0)
					{
						numero1=0;
					}
					
					
					
			}
			display(numero1);
			
			break;

		case 1:
			if (debounce2(QT1081_TOUCH_SENSOR_RIGHT)) {
				//delay_ms(500);
				//gpio_clr_gpio_pin(GPIO_PIN_EXAMPLE_2);
				numero2++;
				
				if (numero2>16)
				{
					numero2=15;
				}
			}
			//}
			//for (uint32_t i = 0; i < 1000; i += 4) {
			
			
			if (debounce2(QT1081_TOUCH_SENSOR_LEFT)) {
				//delay_ms(500);
				//gpio_clr_gpio_pin(GPIO_PIN_EXAMPLE_2);
				numero2--;
				
				if (numero2<=0)
				{
					numero2=0;
				}
				
				
				
			}
			display(numero2);
			
			
			break;
		default:
			//gpio_tgl_gpio_pin(GPIO_PIN_EXAMPLE_1);
			
			break;
		}
		
		if (debounce2(QT1081_TOUCH_SENSOR_ENTER))
		// (arrVal[0])
		{
			numero++;
			if (numero>1)
			{
				numero=0;
			}
		}
		
		if (gpio_get_pin_value(QT1081_TOUCH_SENSOR_UP) == 1)
		{
			delay_ms(100);
			while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_UP) == 1)
			{
				display(numero1&numero2);
			}
		}
		
		if (gpio_get_pin_value(QT1081_TOUCH_SENSOR_DOWN) == 1)
		{
			delay_ms(100);
			while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_DOWN) == 1)
			{
				display(numero1|numero2);
			}
		}
		
		
		

		
		}
	
	
	
}


void Lab3(void){
	
	pm_switch_to_osc0(&AVR32_PM,FOSC0,OSC0_STARTUP);
	
	int32_t cuenta =0;
	while(1){
	if (gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER))
	{
		while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_DOWN)==0)
		{
			display(cuenta++);
			if (cuenta>9)
			{
				cuenta=0;
			}
			delay_s(100);
		}
		//ya se presiono la tecla down
		
		while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER)==0)
		{
			
			display(cuenta--);
			if (cuenta<0)
			{
				cuenta=9;
			}
			delay_s(100);
			
		}
		
		
		
	}
	}
}

void displayA1(void){
	static uint32_t i=0;
	static uint32_t j=0;
	static uint32_t SEL =4;
	uint32_t arr[]={0,6,15,6};
		
		display(arr[i]);
		i++;
		if (i>3)
		{
			i=0;
			
			
			
			if (j>6)
			{/*
				pm_cksel(&AVR32_PM,//CAMBIA FREQ DEL CPU
				0,//PBA DIV
				0,//PBA SEL
				0,//PBB DIV
				0,//PBB SEL
				1,//CPU DIV
				SEL);//CPU SEL 2^(SEL+1)
				SEL++;
				if (SEL>2)
				{
					SEL=0;
				}*/
			
			pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
			pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
			
			pm_pll_setup(&AVR32_PM,0,SEL,1,0,16); // lockcount in main clock for the PLL wait lock

			//_______________________________________________________________________________
			// Establece la frecuencia de salida del PLL
			pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
			//_______________________________________________________________________________
			//Habilita el PLL 0
			pm_pll_enable(&AVR32_PM,0);
			//_______________________________________________________________________________
			//Espera a que se establesca el PLL
			pm_wait_for_pll0_locked(&AVR32_PM) ;
			//_______________________________________________________________________________
			// Set one wait-state (WS) for flash controller
			flashc_set_wait_state(1);

			//habilita la salida del PLL0 con 2 y el OSC0 con 1
			pm_switch_to_clock(&AVR32_PM, 2);
			SEL+=2;
			if (SEL>=8)
			{
				SEL=2;
			}
			
			
			
			
				j=0;
			}
			j++;
			
			

			
		}
	
}







void displayPrimo(void){
	static uint32_t i;
	uint32_t  arr[]={1,3,5,7,11,13};
		
		
		display(arr[i]);
		i++;
		if (i>5)
		{
			i=0;
		}
}

	
	
void Actividad2(void){
		
		
		pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); //osc0 a 12Mhz
		center=0;
		//Interrupciones
		
		Disable_global_interrupt();

		INTC_init_interrupts();

		INTC_register_interrupt(&tecla_lrc_isr, 71, 0);
		INTC_register_interrupt(&tecla_lrc_isr, 70,0);
		gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_LEFT,GPIO_RISING_EDGE);
		gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_RIGHT,GPIO_RISING_EDGE);
		gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_ENTER,GPIO_RISING_EDGE);
		gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_UP,GPIO_RISING_EDGE);
		gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_DOWN,GPIO_RISING_EDGE);


		Enable_global_interrupt();
		
		
		
		
		while (1){}
	}

	
void Actividad3(void){
	
pm_switch_to_osc0(&AVR32_PM,FOSC0,OSC0_STARTUP);

Disable_global_interrupt();
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


INTC_init_interrupts();
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

 Enable_global_interrupt();
 
 while (1){
	 gpio_tgl_gpio_pin(LED0_GPIO);
	 gpio_tgl_gpio_pin(LED1_GPIO);
	 gpio_tgl_gpio_pin(LED2_GPIO);
	 gpio_tgl_gpio_pin(LED3_GPIO);
	 delay_s(100);

 }
 

}


	
	void Actividad1(void){
		pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); //osc0 a 12Mhz
	   while (1)
	   {
		   
		  if (debounce2(QT1081_TOUCH_SENSOR_LEFT))
		   {
			   pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); 
			  
			
			 
			 while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER)==0)
			 {
				
			 gpio_tgl_gpio_pin(LED3_GPIO);
			 
				delay_s(50);			 
				   
			   }
			   
			   display(0);
			   
		  }
		   
		   
		   if (debounce2(QT1081_TOUCH_SENSOR_RIGHT))
		   {
			   pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP); 
			   
			   while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER)==0)
			   {
				   
				   displayPrimo();
				   delay_s(50);
				   
			   }
			   
		   }
		   
		   if (debounce2(QT1081_TOUCH_SENSOR_UP))
		   {
			   pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
			   pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
			   
			   pm_pll_setup(&AVR32_PM,0,2,1,0,16); // lockcount in main clock for the PLL wait lock

			   //_______________________________________________________________________________
			   // Establece la frecuencia de salida del PLL
			   pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
			   //_______________________________________________________________________________
			   //Habilita el PLL 0
			   pm_pll_enable(&AVR32_PM,0);
			   //_______________________________________________________________________________
			   //Espera a que se establesca el PLL
			   pm_wait_for_pll0_locked(&AVR32_PM) ;
			   //_______________________________________________________________________________
			   // Set one wait-state (WS) for flash controller
			   flashc_set_wait_state(1);

			   //habilita la salida del PLL0 con 2 y el OSC0 con 1
			   pm_switch_to_clock(&AVR32_PM, 2);
			   
			   while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER)==0)
			   {
				   
				   displayA1();
					delay_s(50);
				   
			   }
			   display(0);
			   
		   }
		   
		   if (debounce2(QT1081_TOUCH_SENSOR_DOWN))
		   {
			   pm_switch_to_clock(&AVR32_PM,0); //cambia al RC
			   pm_pll_disable(&AVR32_PM,0); // deshabilita el el PLL 0
			   
			   pm_pll_setup(&AVR32_PM,0,6,1,0,16); // lockcount in main clock for the PLL wait lock

			   //_______________________________________________________________________________
			   // Establece la frecuencia de salida del PLL
			   pm_pll_set_option(&AVR32_PM,0,1,0,0);//1 Star-up faster, Start-up normal
			   //_______________________________________________________________________________
			   //Habilita el PLL 0
			   pm_pll_enable(&AVR32_PM,0);
			   //_______________________________________________________________________________
			   //Espera a que se establesca el PLL
			   pm_wait_for_pll0_locked(&AVR32_PM) ;
			   //_______________________________________________________________________________
			   // Set one wait-state (WS) for flash controller
			   flashc_set_wait_state(1);

			   //habilita la salida del PLL0 con 2 y el OSC0 con 1
			   pm_switch_to_clock(&AVR32_PM, 2);
			   	
			   while (gpio_get_pin_value(QT1081_TOUCH_SENSOR_ENTER)==0)
			   {
				   displayOnOff();
				   
			   }
			   display(0);
			   
		   }
	  
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

	void inicializa_PM(void)
	{

		// Activa el oscilador Osc0
		pm_switch_to_osc0(&AVR32_PM,12000000,6);

		//_______________________________________________________________________________
		//Establece la frecuencia  del VCO
		//Si DIV>1           Fvco=((MUL+1)/DIV)*Fosc
		//Si DIV=0           Fvco=2*(MUL+1)*Fosc

		pm_pll_setup(&AVR32_PM,0,7,0,0,16); // lockcount in main clock for the PLL wait lock

		//_______________________________________________________________________________
		// Establece la frecuencia de salida del PLL
		pm_pll_set_option(&AVR32_PM,0,1,1,0);//1 Star-up faster, Start-up normal
		//_______________________________________________________________________________
		//Habilita el PLL 0
		pm_pll_enable(&AVR32_PM,0);
		//_______________________________________________________________________________
		//Espera a que se establesca el PLL
		pm_wait_for_pll0_locked(&AVR32_PM) ;
		//_______________________________________________________________________________
		// Set one wait-state (WS) for flash controller
		flashc_set_wait_state(1);

		//habilita la salida del PLL0 con 2 y el OSC0 con 1
		pm_switch_to_clock(&AVR32_PM, 2);
		



	}

	