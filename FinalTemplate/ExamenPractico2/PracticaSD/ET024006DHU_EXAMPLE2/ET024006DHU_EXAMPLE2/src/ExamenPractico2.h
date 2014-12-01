/*
 * ExamenPractico2.h
 *
 * Created: 11/28/2014 9:20:22 PM
 *  Author: Admin
 */ 


#ifndef EXAMENPRACTICO2_H_
#define EXAMENPRACTICO2_H_

void master_callback(uint32_t arg);
static void twi_init(void);
void sonido(uint8_t* sonidos);
void setupSPI(void);
void init_sys_clocks(void);
void setupCTC(void);
void setUpGpioInterrupt(void);
#endif /* EXAMENPRACTICO2_H_ */
