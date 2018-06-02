#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

#define PortaH PIO_PB0     

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       4096
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** Reference voltage for ADC,in mv. */
#define VOLT_REF        (3300)
/* Tracking Time*/
#define TRACKING_TIME    15
/* Transfer Period */
#define TRANSFER_PERIOD  2
/* Startup Time*/
#define STARTUP_TIME ADC_STARTUP_TIME_4
/** The maximal digital value */
#define MAX_DIGITAL     (4096)

#define ADC_CHANNEL 5

#define TC			TC0 //Define o timer do ADC
#define CHANNEL		0 // Define o canal do timer do ADC
#define ID_TC		ID_TC0
#define TC_Handler  TC0_Handler
#define TC_IRQn     TC0_IRQn



#define STRING_EOL    "\r"
#define STRING_HEADER "-- PWM LED Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;


 uint32_t  adc_duty = 0;
//Variável global do potênciometro(ADC - Analógico - Digital) UINT_32 = UNSIGNED INT

/**
 * \brief Interrupt handler for the PWM controller.
 */
void PWM_Handler(void)
{
	static uint32_t ul_count = 0;  /* PWM counter value */
	static uint32_t ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */
	static uint8_t fade_in = 1;  /* LED fade in flag */

//#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
	//uint32_t events = pwm_channel_get_interrupt_status(PWM0);
//#else
	uint32_t events = pwm_channel_get_interrupt_status(PWM);
//#endif

	/* Interrupt on PIN_PWM_LED0_CHANNEL */
	if ((events & (1 << PIN_PWM_LED0_CHANNEL)) ==
	(1 << PIN_PWM_LED0_CHANNEL)) {
		ul_count++;

		uint16_t result;
		result = adc_get_latest_value(ADC); //Resultado do potenciômetro
		adc_duty = result*100/4096;


		//ul_duty = adc_duty; // Ciclo de trabalho do pwm recebe o ciclo do ADC, no caso, do potenciômetro
	g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
	 //canal habilitado pro pwm recebe o sinal ja padrao da atmal de pwm no led
		pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty); // Atualiza o ciclo de trabaho do pwm3





		/* Fade in/out 
		if (ul_count == (PWM_FREQUENCY / (PERIOD_VALUE - INIT_DUTY_VALUE))) {
			/* Fade in 
			if (fade_in) {
				ul_duty++;
				if (ul_duty == PERIOD_VALUE) {
					fade_in = 0;
				}
				} else {
				/* Fade out 
				ul_duty--;
				if (ul_duty == INIT_DUTY_VALUE) {
					fade_in = 1;
				}
			}

			/* Set new duty cycle 
			ul_count = 0;
			g_pwm_channel_led.channel = PIN_PWM_LED0_CHANNEL;
			#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, ul_duty);
			#else
			pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
			#endif
			g_pwm_channel_led.channel = PIN_PWM_LED1_CHANNEL;
			#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, ul_duty);
			#else
			pwm_channel_update_duty(PWM, &g_pwm_channel_led, ul_duty);
			#endif


			*/
				}
		
			}



/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

void ADC_Handler(void)
{
	uint16_t result;

	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	{

		result = adc_get_latest_value(ADC); //Resultado do potenciômetro

		PWM->PWM_CH_NUM[0].PWM_CDTY = result;
		//adc_duty = result*100/4096; // O ciclo de trabalho do potênciometro foi feito por uma regra de três onde:
		/*
		de 0-100 é um valor x, no caso adc_duty
		de 0-4096 é o valor result então x = result * (100/4095)
		*/
	}
}

void TC_Handler(void)
{
	tc_get_status(TC,CHANNEL);
	adc_start(ADC);
}


//Timer config
static void tc_config(uint32_t freq_desejada)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t counts;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC);
	
	tc_find_mck_divisor( freq_desejada, ul_sysclk, &ul_div, &ul_tcclks,	BOARD_MCK);
	
	tc_init(TC, CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	
	counts = (ul_sysclk/ul_div)/freq_desejada;
	
	tc_write_rc(TC, CHANNEL, counts);

	NVIC_ClearPendingIRQ(TC_IRQn);
	NVIC_SetPriority(TC_IRQn, 4);
	NVIC_EnableIRQ(TC_IRQn);
	
	// Enable interrupts for this TC, and start the TC.
	tc_enable_interrupt(TC,	CHANNEL, TC_IER_CPCS);
	tc_start(TC, CHANNEL);
}


//ADC config
void configure_adc(void)
{
	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);
	
	/* Initialize ADC. */
	/*
	* Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
	* For example, MCK = 64MHZ, PRESCAL = 4, then:
	* ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
	*/
	/* Formula:
	*     Startup  Time = startup value / ADCClock
	*     Startup time = 64 / 6.4MHz = 10 us
	*/
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, STARTUP_TIME);
	
	/* Formula:
	*     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
	*     Tracking Time = (TRACKTIM + 1) / ADCClock
	*     Settling Time = settling value / ADCClock
	*
	*     Transfer Time = (1 * 2 + 3) / 6.4MHz = 781 ns
	*     Tracking Time = (1 + 1) / 6.4MHz = 312 ns
	*     Settling Time = 3 / 6.4MHz = 469 ns
	*/
	adc_configure_timing(ADC, TRACKING_TIME	, ADC_SETTLING_TIME_3, TRANSFER_PERIOD);

	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);

	/* Enable channel for potentiometer. */
	adc_enable_channel(ADC, ADC_CHANNEL);

	NVIC_SetPriority(ADC_IRQn, 5);
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Enable ADC channel interrupt. */
	adc_enable_interrupt(ADC, ADC_IER_DRDY);



	//adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_12);
}

/**
 * \brief Application entry point for PWM with LED example.
 * Output PWM waves on LEDs to make them fade in and out.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	


	/* Configure the console uart for debug information */
	//configure_console();

	/* Output example information */
	//puts(STRING_HEADER);
	
	// disable the PIO (peripheral controls the pin)
	PIOA->PIO_PDR = PIO_PDR_P19;
	// select alternate function B (PWML0) for pin PA19
	PIOA->PIO_ABCDSR[0] |= PIO_ABCDSR_P19;
	PIOA->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P19;
	// Enable the PWM peripheral from the Power Manger
	PMC->PMC_PCER0 = (1 << ID_PWM);
	// Select the Clock to run at the MCK (4MHz)
	PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_MCK;
	// select the period 10msec
	PWM->PWM_CH_NUM[0].PWM_CPRD = 4096;// freq em khz
	// select the duty cycle
	PWM->PWM_CH_NUM[0].PWM_CDTY = 3500;
	// enable the channel
	PWM->PWM_ENA = PWM_ENA_CHID0;


	configure_adc(); //inicializacao do potenciometro
	tc_config(10);


	/* Infinite loop */
	while (1) {
		
		
		pio_set_output(PIOB, PortaH, LOW, DISABLE, ENABLE);
	}
}