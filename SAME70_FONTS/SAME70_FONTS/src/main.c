/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "rtt.h"
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"

#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

/**
* Botão
*/
#define BUT_PIO_ID			  ID_PIOA
#define BUT_PIO				  PIOA
#define BUT_PIN				  11
#define BUT_PIN_MASK			  (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

struct ili9488_opt_t g_ili9488_display_opt;

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile uint8_t pulse = 0;
volatile Bool f_rtt_alarme = false;
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

static void Button1_Handler(uint32_t id, uint32_t mask)
{
	pulse +=1;
}

void RTC_Handler()
{
	uint32_t minute;
	uint32_t hour;
	uint32_t second;
	rtc_get_time(RTC, &hour, &minute, &second);
	uint32_t ul_status = rtc_get_status(RTC);

	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			if (second == 60){
				second = 0;
				minute += 1;
			}
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);

	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		if(pio_get(BUT_PIO, PIO_INPUT, BUT_PIN_MASK)){
			pulse +=1;  
		f_rtt_alarme = true;                  
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void){
	/* led */
	pmc_enable_periph_clk(BUT_PIO);
	pio_configure(BUT_PIO, PIO_OUTPUT_0, BUT_PIN_MASK, PIO_DEFAULT);
}


static void RTT_init((int) (((float) 32768) / 2.0), 4)
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


void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}



void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}


int main(void) {
	 
	 // Desliga watchdog
	 WDT->WDT_MR = WDT_MR_WDDIS;
	 
	 sysclk_init();
	 io_init();
	 
	 // Inicializa RTT com IRQ no alarme.
	 f_rtt_alarme = true;
	
	RTC_init();
		
	board_init();
	configure_lcd();
	char buffer[32];
	sprintf(buffer, "%d", pulse);
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	
	while(1){
			rtc_get_time(RTC, &hour, &minute, &second);
			char buffer[32];
			sprintf(buffer, "%d", second);
			
			font_draw_text(&sourcecodepro_28, buffer, 50, 50, 1);
			font_draw_text(&calibri_36, "Oi Mundo! #$!@", 50, 100, 1);
			font_draw_text(&arial_72, "102456", 50, 200, 2);
			
			
			float tempo;
			float vel_ang = pulse/tempo;
			float vel_real = vel_ang*(0.65); //distancia em metros
			
	}
}