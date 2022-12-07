#include <asf.h>
#include "conf_board.h"
#include <string.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

volatile char flag_tc = 0;
QueueHandle_t xQueueButId;
/************************************************************************/
/* Defines dos leds e botoes                                            */
/************************************************************************/
// Led Embutido
#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX)
// LED 1 OLED XPlained Pro
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)
// LED 2 OLED XPlained Pro
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)
// LED 3 OLED XPlained Pro
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)


//BUZZER
// LED 3 OLED XPlained Pro
#define BUZZER_PIO PIOB
#define BUZZER_PIO_ID ID_PIOB
#define BUZZER_PIO_IDX 4
#define BUZZER_PIO_IDX_MASK (1 << BUZZER_PIO_IDX)


//definindo os botoes da placa EXT1
//botao da placa
#define BUT_PIO     PIOC    //perifferico que controla o botao
#define BUT_PIO_ID  ID_PIOC //ID do periferico PIOA
#define BUT_PIO_IDX 13	    //ID do botao do PIO
#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) //mascara para lermos o botao
//Botao 1
#define BUT1_PIO     PIOD  //perifferico que controla o botao1
#define BUT1_PIO_ID  ID_PIOD //ID do periferico PIOD
#define BUT1_PIO_IDX 28	    //ID do botao1 no PIO
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) //mascara para lermos o botao1

//Botao 2
#define BUT2_PIO     PIOC    //perifferico que controla o botao2
#define BUT2_PIO_ID  ID_PIOC //ID do periferico PIOC
#define BUT2_PIO_IDX 31	    //ID do botao2 no PIO
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX) //mascara para lermos o botao2
//Botao 3
#define BUT3_PIO     PIOA    //perifferico que controla o botao3
#define BUT3_PIO_ID  ID_PIOA //ID do periferico PIOA
#define BUT3_PIO_IDX 19	    //ID do botao3 no PIO
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX) //mascara para lermos o botao3



/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/
int genius_get_sequence(int level, int *sequence);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) ;
void init();
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}



void init(void)
{
	//initialize board clock
	sysclk_init();
	
	//desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	//Ativa o PIO que o LED esta conectado
	pmc_enable_periph_clk(LED_PIO_ID);
	
	//Inicializacao PC8 como saidoa
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	
	
	//Ativa o PIO que o botao esta conectado
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	//Inicializacao do PA11 como entrada
	pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	
	//ativacao do pull-up, o resistor alimentado o vcc p/ o botao funcionar corretamente
	pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, PIO_PULLUP);
	
	
	//pinos da placa EXT1
	
	//LED1
	//Ativa o PIO que o LED1 esta conectado
	pmc_enable_periph_clk(LED1_PIO_ID);
	
	//Inicializacao PA0 como saida
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	
	//LED2
	//Ativa o PIO que o LED2 esta conectado
	pmc_enable_periph_clk(LED2_PIO_ID);
	
	//Inicializacao PC30 como saida
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	
	//LED3
	//Ativa o PIO que o LED3 esta conectado
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	//Inicializacao PB2 como saida
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	//BUZZER
	//Ativa o PIO que o BUZZER esta conectado
	pmc_enable_periph_clk(BUZZER_PIO_ID);
		
	//Inicializacao PB4 como saida
	pio_set_output(BUZZER_PIO, BUZZER_PIO_IDX_MASK, 0, 0, 0);
	
	//Botoes
	
	//botao 1
	// Configura Pinos
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	//debounce filter
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callback);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	//botao 2
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4

	//botao 3
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but3_callback);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4

}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(BUZZER_PIO, BUZZER_PIO_IDX_MASK); 
}

void but1_callback (void) {
	int id = 0;
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		} else {
		// PINO == 0 --> Borda de descida
	}
	
	xQueueSendFromISR(xQueueButId, &id, 0);
	
}

void but2_callback (void) {
	int id = 1;
	if (pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		} else {
		// PINO == 0 --> Borda de descida
	}
	
	xQueueSendFromISR(xQueueButId, &id, 0);
	
}

void but3_callback (void) {
	int id = 2; 
	if (pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
		// PINO == 1 --> Borda de subida
		} else {
		// PINO == 0 --> Borda de descida
	}
	
	xQueueSendFromISR(xQueueButId, &id, 0);
	
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_game(void *pvParameters) {
	gfx_mono_ssd1306_init();

	
	int *sequence[512];
	int level = 0;
	int size;
	size = genius_get_sequence(level, sequence);
	//garante que os leds comecam apagados
	pio_set(LED_PIO, LED_PIO_IDX_MASK);
	pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	
	int complete_flag = 0;
	int fail_flag = 0;
	int flag_read = 0;
	
	
	char text[20];
	sprintf(text, "Level: %d", level);
	gfx_mono_draw_string(text, 0, 0, &sysfont);
	



	for (;;)  {
	char text[20];
	sprintf(text, "Level: %d", level);
	gfx_mono_draw_string(text, 0, 0, &sysfont);
		
		//if(!flag_read){
			//lendo a sequencia e printando nos leds
			for(int i = 0; i < size; i++){
				printf("%d", sequence[i]);
			
				if(sequence[i] == 0){
					TC_init(TC0, ID_TC1, 1, 1000);
					tc_start(TC0, 1);
					pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
					vTaskDelay(200);
					pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
					tc_stop(TC0, 1);
				}
			
				if(sequence[i] == 1){
					TC_init(TC0, ID_TC1, 1, 1500);
					tc_start(TC0, 1);
					pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
					vTaskDelay(200);
					pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
					tc_stop(TC0, 1);
				}
			
				if(sequence[i] == 2){
					TC_init(TC0, ID_TC1, 1, 2000);
					tc_start(TC0, 1);		
					pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
					vTaskDelay(200);
					pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
					tc_stop(TC0, 1);
				}
				int id;
				if( xQueueReceive( xQueueButId, &id, ( TickType_t ) 50 )){
					gfx_mono_draw_string("FALHOU", 0, 0, &sysfont);
					vTaskDelay(1000);
					gfx_mono_draw_string("ESPERE ACABAR", 0, 0, &sysfont);
					vTaskDelay(1000);
					gfx_mono_draw_string("                 ", 0, 0, &sysfont);
					fail_flag = 1;
					//limpa a fila
					while( xQueueReceive( xQueueButId, &id, ( TickType_t ) 10 )){
					}

					break;
				}
				
			}

		int index = 0;
		while(!fail_flag){
			int id;
			if( xQueueReceive( xQueueButId, &id, ( TickType_t ) 2000 )){
				printf("Botao %d apetado", id);
				
					
				if(id == 0 ){
					TC_init(TC0, ID_TC1, 1, 1000);
					tc_start(TC0, 1);
					pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
					vTaskDelay(200);
					pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
					tc_stop(TC0, 1);
				}
				
				if(id == 1){
					TC_init(TC0, ID_TC1, 1, 1500);
					tc_start(TC0, 1);
					pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
					vTaskDelay(200);
					pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
					tc_stop(TC0, 1);
				}
				
				if(id == 2){
					TC_init(TC0, ID_TC1, 1, 2000);
					tc_start(TC0, 1);
					pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
					vTaskDelay(200);
					pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
					tc_stop(TC0, 1);
				}
				
				if(id == sequence[index]){
					index++;
					
				}else{
					fail_flag = 1;
					break;
				}
				
				if(index == (size)){
					complete_flag = 1;
					break;
				}
			}
			else{
				gfx_mono_draw_string("TIME OUT", 0, 0, &sysfont);
				
				vTaskDelay(1000);
				level = 0;
				size = genius_get_sequence(level, sequence);
				
				char text[20];
				sprintf(text, "Level: %d", level);
				gfx_mono_draw_string(text, 0, 0, &sysfont);
				
				complete_flag = 0;
				flag_read = 0;
				//limpa a fila
				while( xQueueReceive( xQueueButId, &id, ( TickType_t ) 10 )){
				}
				
				break;
			}	
		}
		
		if(complete_flag){
			gfx_mono_draw_string("PASSOU          ", 0, 0, &sysfont);
			vTaskDelay(1000);
			level++;
			size = genius_get_sequence(level, sequence);
			
			char text[20];
			sprintf(text, "Level: %d", level);
			gfx_mono_draw_string(text, 0, 0, &sysfont);
			
			complete_flag = 0;
			flag_read = 0;
			
			//limpa a fila 
			/*
			int id;
			while( xQueueReceive( xQueueButId, &id, ( TickType_t ) 10 )){
			}
			*/
		}
		if(fail_flag){
			gfx_mono_draw_string("FALHOU", 0, 0, &sysfont);
			vTaskDelay(1000);
			level = 0;
			size = genius_get_sequence(level, sequence);
			
			char text[20];
			sprintf(text, "Level: %d", level);
			gfx_mono_draw_string(text, 0, 0, &sysfont);
			
			complete_flag = 0;
			flag_read = 0;
			fail_flag = 0;
			
			int id;
			//limpa a fila
			while( xQueueReceive( xQueueButId, &id, ( TickType_t ) 10 )){
			}
			
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
int genius_get_sequence(int level, int *sequence){
	int n = level + 3;

	for (int i=0; i< n ; i++) {
		*(sequence + i) = rand() % 3;
	}

	return n;
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_game, "game", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}
	
	xQueueButId = xQueueCreate(32, sizeof(int) );
	// verifica se fila foi criada corretamente
	if (xQueueButId == NULL){
	  printf("falha em criar a fila \n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
