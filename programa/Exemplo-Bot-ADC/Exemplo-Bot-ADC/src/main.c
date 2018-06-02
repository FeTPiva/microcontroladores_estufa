#include <asf.h>


#define LED_VERDE 1<<20
#define LED_AZUL 1<<19
#define SEN 1<<3
#define BTN 1<<3
#define analogico 1<<1


/** Chip select number to be set */
#define ILI93XX_LCD_CS      1

/** Reference voltage for ADC,in mv. */
#define VOLT_REF        (5000)
/* Tracking Time*/
#define TRACKING_TIME    15 //n precisa mudar
/* Transfer Period */
#define TRANSFER_PERIOD  2 //tb n precisa mudar
/* Startup Time*/
#define STARTUP_TIME ADC_STARTUP_TIME_4
/** The maximal digital value */
#define MAX_DIGITAL     (4095)

#define ADC_CHANNEL 5

struct ili93xx_opt_t g_ili93xx_display_opt;


//PARTE DO TIMER
#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_Handler  TC0_Handler
#define TC_IRQn     TC0_IRQn

#define CONF_UART              UART0
#define CONF_UART_BAUDRATE     9600
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT
#define CONF_UART_PARITY       US_MR_PAR_NO
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT

/************************************************************************/
/* HANDLERS                                                            */
/************************************************************************/

uint16_t valor;

void inicializacao_UART (){
	
	static usart_serial_options_t usart_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS
	};
	
	usart_serial_init(CONF_UART, &usart_options);
	
	stdio_serial_init((Usart *)CONF_UART, &usart_options);
}

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



static void push_button_handle(uint32_t id, uint32_t mask)
{
	adc_start(ADC);
}



/**
* \brief ADC interrupt handler.
*/



//quando ocorrer interrupcao do adc chama esse codigo
/* void ADC_Handler(void)
{
	uint16_t result;

	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	{
		// Recupera o último valor da conversão
		result = adc_get_latest_value(ADC);
		
		char buffer[10];
		sprintf (buffer, "%d", result); //pega caracter por caracter pra printa - buffer
		
		// Desenha retângulo
		ili93xx_set_foreground_color(COLOR_CYAN);
		ili93xx_draw_filled_rectangle(135, 175, 240, 200);
		
		// Escreve uma String no LCD na posição 140, 180
		ili93xx_set_foreground_color(COLOR_GOLD);
		ili93xx_draw_string(140, 180, (uint8_t*) buffer); //uint8 = char
	}
}
*/

void TC_Handler(void)
{
	tc_get_status(TC,CHANNEL); //obrigatorio, usa pra dar reset
	LED_Toggle(LED0_GPIO);
	adc_start(ADC);

	uint16_t result;


	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	{
		// Recupera o último valor da conversão
		result = adc_get_latest_value(ADC);
		
				
		char buffer[10];
		sprintf (buffer, "%d", result); //pega caracter por caracter pra printa - buffer
		
		// Desenha retângulo
		ili93xx_set_foreground_color(COLOR_CYAN);
		ili93xx_draw_filled_rectangle(135, 175, 240, 200);
		
		// Escreve uma String no LCD na posição 140, 180
		ili93xx_set_foreground_color(COLOR_GOLD);
		ili93xx_draw_string(140, 180, (uint8_t*) buffer); //uint8 = char
	}
}

/************************************************************************/
/* CONFIGs                                                              */
/************************************************************************/
void configure_lcd()
{
	/** Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, ILI93XX_LCD_CS, SMC_SETUP_NWE_SETUP(2)
	| SMC_SETUP_NCS_WR_SETUP(2)
	| SMC_SETUP_NRD_SETUP(2)
	| SMC_SETUP_NCS_RD_SETUP(2));
	
	smc_set_pulse_timing(SMC, ILI93XX_LCD_CS, SMC_PULSE_NWE_PULSE(4)
	| SMC_PULSE_NCS_WR_PULSE(4)
	| SMC_PULSE_NRD_PULSE(10)
	| SMC_PULSE_NCS_RD_PULSE(10));
	
	smc_set_cycle_timing(SMC, ILI93XX_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
	| SMC_CYCLE_NRD_CYCLE(22));
	
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE);

	/** Initialize display parameter */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_WHITE;

	/** Switch off backlight */
	aat31xx_disable_backlight();

	/** Initialize LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/** Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,ILI93XX_LCD_HEIGHT);

	/** Turn on LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
}

void configure_botao(void)
{
	pmc_enable_periph_clk(ID_PIOB);
	
	pio_set_input(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, PIN_PUSHBUTTON_1_ATTR);
	pio_set_debounce_filter(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, 10); //filtra pra nao parecer q clicou varias vezes
	pio_handler_set(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_ID,PIN_PUSHBUTTON_1_MASK, PIN_PUSHBUTTON_1_ATTR , push_button_handle);
	pio_enable_interrupt(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK);
	NVIC_SetPriority( PIN_PUSHBUTTON_1_ID, 5);
	NVIC_EnableIRQ( PIN_PUSHBUTTON_1_ID);
}


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

	adc_configure_trigger(ADC, ADC_TRIG_SW, 0); //trigger de software, 0- free running mode

	/* Enable channel for potentiometer. */
	adc_enable_channel(ADC, ADC_CHANNEL); //habilitanto o potenciometro

	NVIC_SetPriority(ADC_IRQn, 5);
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Enable ADC channel interrupt. */
	adc_enable_interrupt(ADC, ADC_IER_DRDY); //interrupcao data ready
}


/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	sysclk_init();
	board_init();

	configure_lcd();
	configure_adc();
	//configure_botao();


	inicializacao_UART();
	tc_config(100);


PIOA->PIO_PER |= LED_VERDE |LED_AZUL;
PIOA->PIO_OER |= LED_VERDE |LED_AZUL;
//PIOA->PIO_CODR |= LED_VERDE |LED_AZUL;

PIOB->PIO_PER |= SEN;
PIOB->PIO_ODR |= SEN;

PIOA->PIO_PER |= analogico;
PIOA->PIO_ODR |= analogico;





	//Configura os PIOs de saida (LEDs)
	//pio_set_output(PIOA, PIO_PA19, LOW, DISABLE, ENABLE);
	//pio_set_output(PIOA, PIO_PA20, LOW, DISABLE, ENABLE);

	/** Draw text on the LCD */
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10, 20, (uint8_t *)"Aula - ADC");



	


	while (1) {
		
		
		
		while(!(PIOB->PIO_PDSR & SEN)){

			PIOA->PIO_SODR |= LED_AZUL;
			
			PIOA->PIO_CODR |= LED_VERDE;
			
			delay_s(5);
			
		}
		
		
		while((PIOB->PIO_PDSR & SEN)){
			PIOA->PIO_CODR |= LED_AZUL;
			PIOA->PIO_SODR |= LED_VERDE;
		}


		//valor = ioport_get_pin_level(analogico);
		//sprintf ("%i", valor);

	}
}
