/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)


#define TASK_LED1_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_UARTRX_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_UARTRX_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_EXECUTE_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_EXECUTE_STACK_PRIORITY (tskIDLE_PRIORITY)

// Config LED
#define LED_PIO PIOC						// periferico que controla o LED
#define LED_PIO_ID 12						// ID do perif�rico PIOC (controla LED)
#define LED_PIO_IDX 8						// ID do LED no PIO
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX) // Mascara para CONTROLARMOS o LED

#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)

#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

#define BUT3_PIO PIOA
#define BUT3_PIO_ID 10
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

#define BUT2_PIO PIOC
#define BUT2_PIO_ID 12
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT1_PIO            PIOD
#define BUT1_PIO_ID         16
#define BUT1_PIO_IDX        28
#define BUT1_PIO_IDX_MASK   (1u << BUT1_PIO_IDX)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore2;
SemaphoreHandle_t xSemaphore3;

QueueHandle_t xQueueChar;
QueueHandle_t xQueueCommand;


/**                                                               
* callback do botao                                               
* libera semaforo: xSemaphore                                    
*/
void but1_callback(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    printf("but1_callback \n");
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    printf("semafaro tx \n");
}
void but2_callback(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    printf("but2_callback \n");
    xSemaphoreGiveFromISR(xSemaphore2, &xHigherPriorityTaskWoken);
    printf("semafaro tx \n");
}
void but3_callback(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    printf("but3_callback \n");
    xSemaphoreGiveFromISR(xSemaphore3, &xHigherPriorityTaskWoken);
    printf("semafaro tx \n");
}

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
void led_toggle(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask) {

	if (pio_get(pio,ul_type,ul_mask)) pio_clear(pio, ul_mask);
	else pio_set(pio,ul_mask);
				
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);
    const TickType_t xDelay = 1500 / portTICK_PERIOD_MS;

	for (;;) {
		//printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		//printf(szList);
		vTaskDelay(xDelay);
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
        led_toggle(LED_PIO,LED_PIO_ID,LED_PIO_IDX_MASK);
    }
  }
}

static void task_led2(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore2 = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init botão */
  pmc_enable_periph_clk(BUT2_PIO_ID);
  pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP);
  pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
  pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT2_PIO_ID);
  NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4

  if (xSemaphore2 == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore2, ( TickType_t ) 500) == pdTRUE ){
        led_toggle(LED2_PIO,LED2_PIO_ID,LED2_PIO_IDX_MASK);
    }
  }
}
static void task_led3(void *pvParameters) {
  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemaphore3 = xSemaphoreCreateBinary();

  /* devemos iniciar a interrupcao no pino somente apos termos alocado
  os recursos (no caso semaforo), nessa funcao inicializamos 
  o botao e seu callback*/
  /* init botão */
  pmc_enable_periph_clk(BUT3_PIO_ID);
  pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP);
  pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback);
  pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
  NVIC_EnableIRQ(BUT3_PIO_ID);
  NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4

  if (xSemaphore3 == NULL)
    printf("falha em criar o semaforo \n");

  for (;;) {
    if( xSemaphoreTake(xSemaphore3, ( TickType_t ) 500) == pdTRUE ){
        led_toggle(LED3_PIO,LED3_PIO_ID,LED3_PIO_IDX_MASK);
    }
  }
}
static void task_led1(void *pvParameters){


	const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;
	const TickType_t xDelayLed = 200 / portTICK_PERIOD_MS;
	while(1){
		for (uint i=0; i<5; i++){
			pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
			vTaskDelay(xDelayLed);
			pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			vTaskDelay(xDelayLed);
		}
		vTaskDelay(xDelay);
	}


}


led_init(){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);

	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);

	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);

}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */

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

static void task_uartRX(void *pvParameters){
	char letter;
	char word[32];
	int i = 0;
	while(1){
		if (xQueueReceive( xQueueChar, &letter, ( TickType_t )  100 / portTICK_PERIOD_MS)){
			if (letter == '\n'){
				//acabou a palavra
				printf("enviou o comando\n");
				word[i] = 0;
				xQueueSend(xQueueCommand, &word, 0);
				i = 0;
			}
			else {
				word[i] = letter;
				i++;
			} 
			//printf("Comando: %c\n",word[i]);
		}

	}
}

static void task_execute(void *pvParameters){
	char command[32];
	while(1){
		if (xQueueReceive (xQueueCommand, &command, ( TickType_t )  100 / portTICK_PERIOD_MS)){
			printf("recebeu comando \n");
			if (strcmp(command, "led 0 toggle")==0) led_toggle(LED_PIO, LED_PIO_ID, LED_PIO_IDX_MASK);
			else if (strcmp(command, "led 1 toggle")==0) led_toggle(LED1_PIO, LED1_PIO_ID, LED1_PIO_IDX_MASK);
			else if (strcmp(command, "led 2 toggle")==0) led_toggle(LED2_PIO, LED2_PIO_ID, LED2_PIO_IDX_MASK);
			else if (strcmp(command, "led 3 toggle")==0) led_toggle(LED3_PIO, LED3_PIO_ID, LED3_PIO_IDX_MASK);
			else if (strcmp(command, "led 0 off")==0) pio_set(LED_PIO, LED_PIO_IDX_MASK);
			else if (strcmp(command, "led 0 on")==0) pio_clear(LED_PIO, LED_PIO_IDX_MASK);
			else if (strcmp(command, "led 1 off")==0) pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			else if (strcmp(command, "led 1 on")==0) pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
			else if (strcmp(command, "led 2 off")==0) pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			else if (strcmp(command, "led 2 on")==0) pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
			else if (strcmp(command, "led 3 off")==0) pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			else if (strcmp(command, "led 3 on")==0) pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
			
}
	}
}



void USART1_Handler(void){
  uint32_t ret = usart_get_status(USART1);

  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  char c;

  // Verifica por qual motivo entrou na interrupçcao?
  // RXRDY ou TXRDY

  //  Dados disponível para leitura
  if(ret & US_IER_RXRDY){
	  usart_serial_getchar(USART1, &c); //pega um char da serial e salva em c
	  xQueueSendFromISR(xQueueChar, &c, 0);


  // -  Transmissoa finalizada
  } else if(ret & US_IER_TXRDY){

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

	// Criar a fila!
	xQueueChar = xQueueCreate (30, sizeof(char));
	xQueueCommand = xQueueCreate (10, sizeof(char)*30);

	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	led_init();
	/* Initialize the console uart */
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
	if (xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL, TASK_LED1_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led1 task\r\n");
	}
	if (xTaskCreate(task_led2, "Led2", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led2 task\r\n");
	}
	if (xTaskCreate(task_led3, "Led3", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led3 task\r\n");
	}
		if (xTaskCreate(task_uartRX, "task_uartRX", TASK_UARTRX_STACK_SIZE, NULL, TASK_UARTRX_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create task_uartRX\r\n");
	}
		if (xTaskCreate(task_execute, "task_execut", TASK_EXECUTE_STACK_SIZE, NULL, TASK_EXECUTE_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create task_execut\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}