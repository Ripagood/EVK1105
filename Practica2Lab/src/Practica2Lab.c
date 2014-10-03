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

#define  RGB(r,g,b) r<<11|g<<5|b 

volatile uint16_t RectX = 160;
volatile uint16_t RectY = 120;
volatile uint32_t enter= 1;
volatile uint16_t actividad = 0;
void tecla_lrc_isr(void);
void Matrix(void);
void Rectangulo(void);
void Imagen(void);
void verticalPrint(char *str,size_t size, uint16_t x, et024006_color_t color);
void scrollVertical(char* str,size_t size, uint16_t x);
void addToArr(char * str, char add, size_t size);
void Triangulos(void);
void centerTriangle(uint16_t x, uint16_t y, uint16_t size, et024006_color_t color);



void Ej1(void);
void Ej4(void);
void Ej7(void);
void Ej11(void);

void AjedrezCrece(void);
void AjedrezRandom(void);
uint32_t debounce2( uint32_t GPIO_PIN );

void CLR_disp(void);




#if BOARD == EVK1105
#include "pwm.h"
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
		if (actividad==0)
		{
			while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_LEFT)){
				et024006_DrawFilledRect(RectX , RectY, 30, 30, BLACK);//borra la posicion pasada
				RectX+=2;
				et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
				delay_ms(10);
				
			}
			
		}
		
		
	}

	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_RIGHT))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_RIGHT);
		//LastRectX=RectX;
		if (actividad==0)
		{
			while(gpio_get_pin_value(QT1081_TOUCH_SENSOR_RIGHT)){
				et024006_DrawFilledRect(RectX , RectY, 30, 30, BLACK);//borra la posicion pasada
				RectX-=2;
				et024006_DrawFilledRect(RectX , RectY, 30, 30, RED);
				delay_ms(10);
			}
			
		}
		
	}
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP))
	{
				gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_UP);
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
		     
			  
	}
	
	if (gpio_get_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN))
	{
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_DOWN);
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
		
	}
	
	if (gpio_get_pin_interrupt_flag (QT1081_TOUCH_SENSOR_ENTER))
	{
		
		gpio_clear_pin_interrupt_flag(QT1081_TOUCH_SENSOR_ENTER);
		//LastRectX=RectX;
		enter=0;
		
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


// Main function
int main(void)
{
  U32 i;

  // Set CPU and PBA clock
  pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

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

  INTC_register_interrupt(&tecla_lrc_isr, 71, 0);
  INTC_register_interrupt(&tecla_lrc_isr, 70,0);
  gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_LEFT,GPIO_RISING_EDGE);
  gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_RIGHT,GPIO_RISING_EDGE);
  gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_ENTER,GPIO_RISING_EDGE);
  gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_UP,GPIO_RISING_EDGE);
  gpio_enable_pin_interrupt(QT1081_TOUCH_SENSOR_DOWN,GPIO_RISING_EDGE);


  Enable_global_interrupt();

  while(pwm_channel6.cdty < pwm_channel6.cprd)
  {
	  pwm_channel6.cdty++;
	  pwm_channel6.cupd = pwm_channel6.cdty;
	  //pwm_channel6.cdty--;
	  pwm_async_update_channel(AVR32_PWM_ENA_CHID6, &pwm_channel6);
	  delay_ms(10);
  }
 
  while(1){
	  switch (actividad){
		case 0:
			Rectangulo();
			actividad++;
			break;
		case 1:
			Matrix();
			actividad++;
			break;
		case 2:
			Imagen();
			actividad++;
			break;
		case 3:
		    CLR_disp();
			Ej1();
			actividad++;
			break;
		case 4:
			CLR_disp();
			Ej11();
			actividad++;
			break;
		case 5:
			CLR_disp();
			Ej7();
			actividad++;
			break;
		case 6:
			CLR_disp();
			Ej4();
			actividad++;
			break;
		case 7:
			Triangulos();
			actividad++;
			break;
		case 8:
			AjedrezCrece();
			actividad++;
			break;
		case 9: 
			AjedrezRandom();
			actividad=0;
			break;
			
	  }

	 }
  
  
  
  


  // Display lines of colored squares.
  for( i=0 ; i<16 ; i++ )
  {
    // From black to white.
    et024006_DrawFilledRect(20*i,   0, 20, 20, (2*i)/*B:5*/ | ((4*i)<<5)/*G:6*/ | ((2*i)<<11)/*R:5*/ );
    // From black to blue.
    et024006_DrawFilledRect(20*i,  20, 20, 20, (2*i) /*B:5*/);
    // From black to green
    et024006_DrawFilledRect(20*i, 200, 20, 20, ((4*i)<<5) /*G:6*/);
    // From black to red
    et024006_DrawFilledRect(20*i, 220, 20, 20, ((2*i)<<11) /*R:5*/);
  }


#if BOARD == EVK1105
  /* Lets do a nice fade in by increasing the duty cycle */
  while(pwm_channel6.cdty < pwm_channel6.cprd)
  {
    pwm_channel6.cdty++;
    pwm_channel6.cupd = pwm_channel6.cdty;
    //pwm_channel6.cdty--;
    pwm_async_update_channel(AVR32_PWM_ENA_CHID6, &pwm_channel6);
    delay_ms(10);
  }
#endif

  // Display text.
  et024006_PrintString("AVR UC3", (const unsigned char *)&FONT6x8, 80, 50, BLUE, -1);
  et024006_PrintString("AVR UC3", (const unsigned char *)&FONT8x8, 80, 60, BLACK, -1);

  // Draw a crossed square.
  et024006_DrawHorizLine(10, 50, 20, BLACK);
  et024006_DrawVertLine(10, 50, 20, BLACK);
  et024006_DrawHorizLine(10, 70, 20, BLACK);
  et024006_DrawVertLine(30, 50, 20, BLACK);
  et024006_DrawLine(10, 50, 30, 70, BLACK);
  et024006_DrawLine(30, 50, 10, 70, BLACK);

  while(true);
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
/*
void Piet(void){
	
	
	et024006_DrawFilledRect(0 , 0, ET024006_WIDTH, ET024006_HEIGHT, WHITE );
	
	
	et024006_DrawFilledRect(0 , 50, ET024006_WIDTH, 20, BLACK );
	
	et024006_DrawFilledRect(0, 100, ET024006_WIDTH, 20, BLACK );
	
	et024006_DrawFilledRect(50 , 0, 20, ET024006_HEIGHT, BLACK);
	
	et024006_DrawFilledRect(100 , 0, 20, ET024006_HEIGHT, BLACK);
	
	et024006_DrawFilledRect(5 , 70, 20, ET024006_HEIGHT, BLACK);
	
	
	
	
	
	
	
// }
// */


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
