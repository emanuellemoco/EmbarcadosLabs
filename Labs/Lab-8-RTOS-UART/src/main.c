/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

// Configuracoes dos LEDs
#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        12                   // ID do perif�rico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED


#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

#define LED2_PIO			PIOC
#define LED2_PIO_ID			ID_PIOC
#define LED2_PIO_IDX		30
#define LED2_PIO_IDX_MASK	(1 << LED2_PIO_IDX)

#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_IDX		2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         16
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_LED1_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_UARTRX_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_UARTRX_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_EXECUTE_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_EXECUTE_STACK_PRIORITY        (tskIDLE_PRIORITY)

/** Semaforo a ser usado pela task led 
    tem que ser var global! */
SemaphoreHandle_t xSemaphore;

QueueHandle_t xQueueChar;
QueueHandle_t xQueueCommand;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**                                                               
* callback do botao                                               
* libera semaforo: xSemaphore                                    
*/
void but1_callback(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    printf("but_callback \n");
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    printf("semafaro tx \n");
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

void led_toggle(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask) {

    if (pio_get(pio,ul_type,ul_mask)) pio_clear(pio, ul_mask);
    else pio_set(pio,ul_mask);
                
}


extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u\n\r", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(3000);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init botão */
  pmc_enable_periph_clk(BUT1_PIO_ID);
  pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
  pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
  pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT1_PIO_ID);
  NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4

  if (xSemaphore == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE ){
      led_toggle(LED_PIO,LED_PIO_ID, LED_PIO_IDX_MASK);
    }
  }
}
static void task_led1(void *pvParameters)
{
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	/* Block for 2000ms. = 2s */ 
    const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;  //3s
	const TickType_t xDelayLed = 200 / portTICK_PERIOD_MS;

	for (;;) {
		for (int i = 0; i < 5; i++)
		{
			pio_clear(LED1_PIO, LED1_PIO_IDX_MASK) ; // liga o led
			vTaskDelay(xDelayLed);
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK); // apaga o led
			vTaskDelay(xDelayLed);
		}
		vTaskDelay(xDelay);
	}
}

void led_init(void){
	//Ativa o PIO na qual o LED foi conectado
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	//Inicializa cada PIO como saida
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
}


/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}


static void USART1_init(void){
  /* Configura USART1 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  sysclk_enable_peripheral_clock(ID_PIOA);
  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
      .baudrate       = 115200,
      .char_length    = US_MR_CHRL_8_BIT,
      .parity_type    = US_MR_PAR_NO,
      .stop_bits    = US_MR_NBSTOP_1_BIT    ,
      .channel_mode   = US_MR_CHMODE_NORMAL
  };

  /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(ID_USART1);

  stdio_serial_init(CONF_UART, &usart_settings);

  /* Enable the receiver and transmitter. */
  usart_enable_tx(USART1);
  usart_enable_rx(USART1);

  /* map printf to usart */
  ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
  ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

  /* ativando interrupcao */
  usart_enable_interrupt(USART1, US_IER_RXRDY);
  NVIC_SetPriority(ID_USART1, 4);
  NVIC_EnableIRQ(ID_USART1);
}

void USART1_Handler(void){
//Irá receber um caracter da uart e colocar ele na fila xQueueChar
  uint32_t ret = usart_get_status(USART1);

  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  char c;
  printf("entrou em USART1_HANDLER \n");

  // Verifica por qual motivo entrou na interrupçcao?
  // RXRDY ou TXRDY

  //  Dados disponível para leitura
  if(ret & US_IER_RXRDY){
      usart_serial_getchar(USART1, &c); //pega um char da serial e salva em c
      printf("pegou:%c", c);
	  xQueueSendFromISR(xQueueChar, &c, 0);

  // -  Transmissoa finalizada
  } else if(ret & US_IER_TXRDY){

  }
}


void task_uartRX(void){
	printf("ENTROU NESSA PORRA \n ");
	char letter;
	char word[30];
	int i =0;
	while(1){ 
		if (xQueueReceive( xQueueChar, &letter, ( TickType_t )  100 / portTICK_PERIOD_MS)){
			if (letter== 0) {
				word[i] = 0;
				xQueueSend(xQueueCommand, &word, 0);
				i = 0;
			} else {
				word[i] = letter;
				i ++;
			}
			printf("mandando char %c", letter);
		}
	}
}

static void task_execute(void *pvParameters){
	char command[30];
	while(1){
		if (xQueueReceive( xQueueCommand, &command, ( TickType_t )  100 / portTICK_PERIOD_MS)){
			printf("ENTROU EM TASK EXECUTE \n ");
			//strsdr
			if (strcmp(command, "led 0") == 0) led_toggle(LED_PIO,LED_PIO_ID, LED_PIO_IDX_MASK);
			else if (strcmp(command, "led 1") == 0) led_toggle(LED1_PIO,LED1_PIO_ID, LED1_PIO_IDX_MASK);
			else if (strcmp(command, "led 2") == 0) led_toggle(LED2_PIO,LED2_PIO_ID, LED2_PIO_IDX_MASK);
			else if (strcmp(command, "led 3") == 0) led_toggle(LED3_PIO,LED3_PIO_ID, LED3_PIO_IDX_MASK);
		}
	}
}


uint32_t usart1_puts(uint8_t *pstring){
    uint32_t i ;

    while(*(pstring + i))
        if(uart_is_tx_empty(USART1))
            usart_serial_putchar(USART1, *(pstring+i++));
}



int main(void)
{
	// Criar a fila aqui!
	xQueueChar = xQueueCreate (30, sizeof(char));
	xQueueCommand = xQueueCreate (5, sizeof(char)*30);

	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	led_init();
	

	/* Initialize the console uart */
	//configure_console();
	USART1_init();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	if (xTaskCreate(task_led1, "Led", TASK_LED1_STACK_SIZE, NULL,
			TASK_LED1_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}	

	if (xTaskCreate(task_execute, "Led", TASK_EXECUTE_STACK_SIZE, NULL,
			TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led execute\r\n");
	}

	if (xTaskCreate(task_uartRX, "Led", TASK_UARTRX_STACK_SIZE, NULL,
		TASK_UARTRX_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test task uartRX\r\n");
	}


	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
