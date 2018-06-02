
#include <asf.h>


#define LED_VERDE 1<<20
#define LED_AZUL 1<<19
#define SEN 1<<3
#define BTN 1<<3
#define analogico 1<<7


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

sysclk_init();

board_init();


	/* Insert application code here, after the board has been initialized. */

		PIOA->PIO_PER |= LED_VERDE |LED_AZUL;
		PIOA->PIO_OER |= LED_VERDE |LED_AZUL;
		//PIOA->PIO_CODR |= LED_VERDE |LED_AZUL;
		
		PIOB->PIO_PER |= SEN;
		PIOB->PIO_ODR |= SEN;
		
	//	PIOA->PIO_PER |= analogico;
		//PIOA->PIO_ODR |= analogico;

		

		while(1){



			while(!(PIOB->PIO_PDSR & SEN)){

				PIOA->PIO_SODR |= LED_AZUL;
				
				PIOA->PIO_CODR |= LED_VERDE;
				
			//	delay_s(5);
			
			}
			
			
			while((PIOB->PIO_PDSR & SEN)){
				PIOA->PIO_CODR |= LED_AZUL;
				PIOA->PIO_SODR |= LED_VERDE;
				
				
			}

/*int valor;

		valor = ioport_get_pin_level(analogico);
		sprintf ("%i", valor);

*/
		}




}
