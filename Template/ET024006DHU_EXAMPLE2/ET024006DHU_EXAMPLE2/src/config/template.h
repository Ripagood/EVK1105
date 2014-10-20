/*
 * template.h
 *
 * Created: 10/18/2014 5:14:39 PM
 *  Author: Admin
 */ 


#ifndef TEMPLATE_H_
#define TEMPLATE_H_





uint32_t debounce2( uint32_t GPIO_PIN );
void setUpGpioInterrupt(void);
void setupCTC(void);
void setupPWM(void);
void initUSART(void);
void setupEIC(void);
void initClocks (void);
void setupSPI(void);
#endif /* TEMPLATE_H_ */
