/*
 * Practica4Lab.h
 *
 * Created: 10/22/2014 7:50:12 PM
 *  Author: Admin
 */ 


#ifndef PRACTICA4LAB_H_
#define PRACTICA4LAB_H_

void tecla_lrc_isr(void);
void Matrix(void);
void Rectangulo(void);
void Imagen(void);
//void verticalPrint(char *str,size_t size, uint16_t x, et024006_color_t color);
void scrollVertical(char* str,size_t size, uint16_t x);
void addToArr(char * str, char add, size_t size);
void Triangulos(void);
//void centerTriangle(uint16_t x, uint16_t y, uint16_t size, et024006_color_t color);
void Caracol(void);


void reloj(void);
void PONG(void);
void cronometro(void);
volatile uint32_t gana1=1;


void Ej1(void);
void Ej4(void);
void Ej7(void);
void Ej11(void);

void AjedrezCrece(void);
void AjedrezRandom(void);
uint32_t debounce2( uint32_t GPIO_PIN );

void CLR_disp(void);


#define TFT_QUADRANT0 ((1 << 1) | (1 << 0))
#define TFT_QUADRANT1 ((1 << 3) | (1 << 2))
#define TFT_QUADRANT2 ((1 << 5) | (1 << 4))
#define TFT_QUADRANT3 ((1 << 7) | (1 << 6))


void recibirImagen(uint32_t sector);
static void sd_mmc_resources_init(void);
void saveImage(uint16_t* buffer,uint32_t sector);
void displayImage(uint32_t sector);
void loadImage(uint8_t* buffer,uint32_t sector);
void deleteImage(uint32_t sector);
void init_SD_DMA_RX(void);
void init_Usart_DMA_RX(void);
void getClave(char* clave);
void wait(void);
volatile uint16_t RectX = 160;
volatile uint16_t RectY = 120;
#if PRACTICA == 2
volatile uint32_t enter = 1;
#endif
#if PRACTICA == 3
volatile uint32_t enter = 0;
#endif

volatile uint16_t actividad = 0;

volatile uint32_t go=0;
volatile uint32_t resetTimer=0;


#include "compiler.h"
#include "board.h"
#include "pm.h"
#include "gpio.h"
#include "tc.h"
//#include "usart.h"
#define FPBA          FOSC0
#define TC_CHANNEL    0

#define EXAMPLE_TC (&AVR32_TC)
#define FALSE 0
#define AVR32_TC_IRQ0 448
#define EXAMPLE_TC_IRQ AVR32_TC_IRQ0

#endif /* PRACTICA4LAB_H_ */