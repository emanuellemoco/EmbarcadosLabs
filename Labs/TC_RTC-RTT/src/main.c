#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configuracoes do led same70
#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        12                  // ID do perif�rico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Configuracoes dos LEDs
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

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t seccond;
} calendar;


/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile char flag_tc = 0; //TC
volatile Bool f_rtt_alarme = false; //RTT
volatile char flag_rtc = 0; // RTC
volatile char flag_rtc2 = 0; //RTC2

volatile char flag_tc2 = 0; //TC2


void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pisca_led_RTC(int n, int t);


void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc = 1;
}

void TC4_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc2 = 1;
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		//ENTROU AQUI
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		flag_rtc2 = 1;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    	flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	

}

/*
 * Pisca LED placa
 */
void pisca_led(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);

		
	}
}

void pisca_led2(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_PIO, LED_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PIO, LED_PIO_IDX_MASK);
		delay_ms(t);
	}
}

void pisca_led_RTC(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
		delay_ms(t);
	}
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/**
*Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED1_PIO);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, estado, 0, 0);
	pmc_enable_periph_clk(LED2_PIO);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);

	pmc_enable_periph_clk(LED3_PIO);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, estado, 0, 0 );
};

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);

}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter � meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	/* Interrup��o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}
void get_time (char hora[3], char minuto[3], char segundo[3], char horario[9]){
	horario[0] = hora[0];
	horario[1] = hora[1];
	horario[2] = ':';
	horario[3] = minuto[0];
	horario[4] = minuto[1];
	horario[5] = ':';
	horario[6] = segundo[0];
	horario[7] = segundo[1];

}

void show_time(calendar rtc_initial){
	//Exiba a hora no formato (HH:MM:SS) no display OLED
	char hora[3];
	char minuto[3];
	char segundo[3];

	sprintf(hora,"%lu", rtc_initial.hour);
	sprintf(minuto,"%lu", rtc_initial.minute);
	sprintf(segundo,"%lu", rtc_initial.seccond);

	char teste [] = {segundo[0]};
	char horario[] = { hora[0],hora[1], ':', minuto[0], minuto[1], ':', segundo[0], segundo[1]};
	//gfx_mono_draw_filled_circle(104, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_string(horario, 1,16, &sysfont);
	gfx_mono_draw_filled_circle(95, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);

}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();

  // Init OLED
	gfx_mono_ssd1306_init();
  
  // Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	//gfx_mono_draw_string("Mundo", 50,16, &sysfont);
  
	/* Configura Leds */
	LED_init(0);	

	
	 /** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC1, ID_TC4, 1, 10);
	
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt_alarme = true;
	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	

	
//	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN );
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);

	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	
	gfx_mono_draw_string("          ", 1,16, &sysfont);
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		
	//TC:
	if(flag_tc){
		pisca_led(1,10);
		flag_tc = 0;
	}
		
	//TC2:
	if(flag_tc2){
		pisca_led2(1,10);
		flag_tc2 = 0;
	}
		

	//RTT:
	if (f_rtt_alarme){
	
	/*
	* IRQ apos 4s -> 8*0.5
	*/
	uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
	uint32_t irqRTTvalue = 8;
	
	// reinicia RTT para gerar um novo IRQ
	RTT_init(pllPreScale, irqRTTvalue);         
	
	f_rtt_alarme = false;
}
	
	//RTC:
	if(flag_rtc){
		pisca_led_RTC(5, 200);
		flag_rtc = 0;
	}

	if(flag_rtc2){
		rtc_get_time(RTC, &rtc_initial.hour, &rtc_initial.minute, &rtc_initial.seccond);
		flag_rtc2 = 0;
		show_time(rtc_initial);

	}
	
	
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
