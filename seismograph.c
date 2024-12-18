/*
* Bibliotecas necesarias de C
*/

#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

/*
 * Bibliotecas de libopencm3
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/usart.h>

/*
 * Bibliotecas de libopencm3-examples:
 * adc-dac-printf
 * lcd-serial
 * spi
 */

#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"

#define MEMS_RNW			(1 << 7)  
#define MEMS_MNS			(1 << 6) 
#define MEMS_WHO_AM_I		0x0F
#define MEMS_OUT_TEMP		0x26
#define MEMS_STATUS_REG		0x27
#define MEMS_CTRL_REG1		0x20
#define MEMS_CTRL_REG1_PD	(1 << 3)
#define MEMS_CTRL_REG1_XEN	(1 << 1)
#define MEMS_CTRL_REG1_YEN	(1 << 0)
#define MEMS_CTRL_REG1_ZEN	(1 << 2)
#define MEMS_CTRL_REG1_BW_SHIFT	4
#define MEMS_CTRL_REG4		0x23
#define MEMS_CTRL_REG4_FS_SHIFT	4

////////////////////////////////////////////////////////////////
/**
 * adc-dac-printf.c
 */

//#define LED_DISCO_GREEN_PORT GPIOG
//#define LED_DISCO_GREEN_PIN GPIO13

//#define USART_CONSOLE USART1


//int _write(int file, char *ptr, int len);



//static void usart_setup(void)
//{
	/* Setup GPIO pins for USART1 transmit. */
//	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
//	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
/*
	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);
*/
	/* Finally enable the USART. */
//	usart_enable(USART_CONSOLE);
//}

/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
//int _write(int file, char *ptr, int len)
//{
//	int i;

//	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
//		for (i = 0; i < len; i++) {
//			if (ptr[i] == '\n') {
//				usart_send_blocking(USART_CONSOLE, '\r');
//			}
//			usart_send_blocking(USART_CONSOLE, ptr[i]);
//		}
//		return i;
//	}
//	errno = EIO;
//	return -1;
//}

//static void adc_setup(void)
//{
//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
//
//	adc_power_off(ADC1);
//	adc_disable_scan_mode(ADC1);
//	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
//
//	adc_power_on(ADC1);
//
//}
/*
static void dac_setup(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);
	dac_disable(DAC1, DAC_CHANNEL2);
	dac_disable_waveform_generation(DAC1, DAC_CHANNEL2);
	dac_enable(DAC1, DAC_CHANNEL2);
	dac_set_trigger_source(DAC1, DAC_CR_TSEL2_SW);
}

static uint16_t read_adc_naiive(uint8_t channel)
{
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t reg16 = adc_read_regular(ADC1);
	return reg16;
}
*/
////////////////////////////////////////////////////////////////
/**
 * lcd-serial.c
 */

#define d2r(d) ((d) * 6.2831853 / 360.0)

////////////////////////////////////////////////////////////////
/**
 * spi-mems.c
 */

/*
 * Functions defined for accessing the SPI port 8 bits at a time
 */
uint16_t read_reg(int reg);
void write_reg(uint8_t reg, uint8_t value);
uint8_t read_xyz(int16_t vecs[3]);
void spi_init(void);

/*
 * Chart of the various SPI ports (1 - 6) and where their pins can be:
 *
 *	 NSS		  SCK			MISO		MOSI
 *	 --------------   -------------------   -------------   ---------------
 * SPI1  PA4, PA15	  PA5, PB3		PA6, PB4	PA7, PB5
 * SPI2  PB9, PB12, PI0   PB10, PB13, PD3, PI1  PB14, PC2, PI2  PB15, PC3, PI3
 * SPI3  PA15*, PA4*	  PB3*, PC10*		PB4*, PC11*	PB5*, PD6, PC12*
 * SPI4  PE4,PE11	  PE2, PE12		PE5, PE13	PE6, PE14
 * SPI5  PF6, PH5	  PF7, PH6		PF8		PF9, PF11, PH7
 * SPI6  PG8		  PG13			PG12		PG14
 *
 * Pin name with * is alternate function 6 otherwise use alternate function 5.
 *
 * MEMS uses SPI5 - SCK (PF7), MISO (PF8), MOSI (PF9),
 * MEMS CS* (PC1)  -- GPIO
 * MEMS INT1 = PA1, MEMS INT2 = PA2
 */

void put_status(char *m);

/*
 * put_status(char *)
 *
 * This is a helper function I wrote to watch the status register
 * it decodes the bits and prints them on the console. Sometimes
 * the SPI port comes up with the MODF flag set, you have to re-read
 * the status port and re-write the control register to clear that.
 */
void put_status(char *m)
{
	uint16_t stmp;

	console_puts(m);
	console_puts(" Status: ");
	stmp = SPI_SR(SPI5);
	if (stmp & SPI_SR_TXE) {
		console_puts("TXE, ");
	}
	if (stmp & SPI_SR_RXNE) {
		console_puts("RXNE, ");
	}
	if (stmp & SPI_SR_BSY) {
		console_puts("BUSY, ");
	}
	if (stmp & SPI_SR_OVR) {
		console_puts("OVERRUN, ");
	}
	if (stmp & SPI_SR_MODF) {
		console_puts("MODE FAULT, ");
	}
	if (stmp & SPI_SR_CRCERR) {
		console_puts("CRC, ");
	}
	if (stmp & SPI_SR_UDR) {
		console_puts("UNDERRUN, ");
	}
	console_puts("\n");
}

/*
 * read_reg(int reg)
 *
 * This reads the MEMs registers. The chip registers are 16 bits
 * wide, but I read it as two 8 bit bytes. Originally I tried
 * swapping between an 8 bit and 16 bit wide bus but that confused
 * both my code and the chip after a while so this was found to
 * be a more stable solution.
 */
uint16_t
read_reg(int reg)
{
	uint16_t d1, d2;

	d1 = 0x80 | (reg & 0x3f); /* Read operation */
	/* Nominallly a register read is a 16 bit operation */
	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, d1);
	d2 = spi_read(SPI5);
	d2 <<= 8;
	/*
	 * You have to send as many bits as you want to read
	 * so we send another 8 bits to get the rest of the
	 * register.
	 */
	spi_send(SPI5, 0);
	d2 |= spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);
	return d2;
}

/*
 * uint8_t status = read_xyz(int16_t [])
 *
 * This function exploits the fact that you can do a read +
 * auto increment of the SPI registers. It starts at the
 * address of the X register and reads 6 bytes.
 *
 * Then the status register is read and returned.
 */
uint8_t
read_xyz(int16_t vecs[3])
{
   
	uint8_t	 buf[7];
	int		 i;

	gpio_clear(GPIOC, GPIO1); /* CS* select */
	spi_send(SPI5, 0xc0 | 0x28);
	(void) spi_read(SPI5);
	for (i = 0; i < 6; i++) {
		spi_send(SPI5, 0);
		buf[i] = spi_read(SPI5);
	}
	gpio_set(GPIOC, GPIO1); /* CS* deselect */
	vecs[0] = (buf[1] << 8 | buf[0]);
	vecs[1] = (buf[3] << 8 | buf[2]);
	vecs[2] = (buf[5] << 8 | buf[4]);
	return read_reg(0x27); /* Status register */
}

/*
 * void write_reg(uint8_t register, uint8_t value)
 *
 * This code then writes into a register on the chip first
 * selecting it and then writing to it.
 */
void
write_reg(uint8_t reg, uint8_t value)
{
	gpio_clear(GPIOC, GPIO1); /* CS* select */
	spi_send(SPI5, reg);
	(void) spi_read(SPI5);
	spi_send(SPI5, value);
	(void) spi_read(SPI5);
	gpio_set(GPIOC, GPIO1); /* CS* deselect */
	return;
}

int print_decimal(int);

/*
 * int len = print_decimal(int value)
 *
 * Very simple routine to print an integer as a decimal
 * number on the console.
 */
int
print_decimal(int num)
{
	int		ndx = 0;
	char	buf[10];
	int		len = 0;
	char	is_signed = 0;

	if (num < 0) {
		is_signed++;
		num = 0 - num;
	}
	buf[ndx++] = '\000';
	do {
		buf[ndx++] = (num % 10) + '0';
		num = num / 10;
	} while (num != 0);
	ndx--;
	if (is_signed != 0) {
		console_putc('-');
		len++;
	}
	while (buf[ndx] != '\000') {
		console_putc(buf[ndx--]);
		len++;
	}
	return len; /* number of characters printed */
}

char *axes[] = { "X: ", "Y: ", "Z: " };

void spi_init(void){
	rcc_periph_clock_enable(RCC_SPI5);

	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOF);

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_set(GPIOC, GPIO1);

	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9);   
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

	/*Configuracion correcta de SPI (faltante en el ejemplo spi-mems.c) */
	spi_set_master_mode(SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI5);
	spi_set_clock_phase_0(SPI5);
	spi_set_full_duplex_mode(SPI5);
	spi_set_unidirectional_mode(SPI5);
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);
	spi_enable(SPI5);


	/*GPIOG clock habilitado. */
	rcc_periph_clock_enable(RCC_GPIOG);

	/*GPIO13 configurado (en GPIO port G) como 'output push-pull'.*/
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13 | GPIO14);

    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, MEMS_CTRL_REG1); 
	spi_read(SPI5);
	spi_send(SPI5, MEMS_CTRL_REG1_PD | MEMS_CTRL_REG1_XEN |
			MEMS_CTRL_REG1_YEN | MEMS_CTRL_REG1_ZEN |
			(3 << MEMS_CTRL_REG1_BW_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1); 

    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, MEMS_CTRL_REG4);
	spi_read(SPI5);
	spi_send(SPI5, (1 << MEMS_CTRL_REG4_FS_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);
}

/***
 * Main, contiene declaraciones de los tres archivos
 */
int main(void)
{
	/* Adicion de cadena de caracteres para desplegar en pantalla LCD */
	char int_to_str[7];
	char lcd_out[3];
	
	/*Codigo tomado de funcion main() de spi-mems.c*/
	int16_t vecs[3];
	int16_t baseline[3];
	int tmp, i;
	int count = 0;
	uint32_t temp;
//	int cursor = 36;

	/*Codigo tomado de funcion main() de lcd-serial.c */
//	int p1, p2, p3;
	clock_setup();
	console_setup(115200);
	spi_init();
	sdram_init();
	lcd_spi_init();

	console_puts("LCD Initialized\n");
	console_puts("Should have a checker pattern, press any key to proceed\n");
	msleep(2000);
	/*	(void) console_getc(1); */
	gfx_init(lcd_draw_pixel, 240, 320);
	gfx_fillScreen(LCD_GREY);
	gfx_fillRoundRect(10, 10, 220, 220, 5, LCD_WHITE);
	gfx_drawRoundRect(10, 10, 220, 220, 5, LCD_RED);
	gfx_fillCircle(20, 250, 10, LCD_RED);
	gfx_fillCircle(120, 250, 10, LCD_GREEN);
	gfx_fillCircle(220, 250, 10, LCD_BLUE);
	gfx_setTextSize(2);
	gfx_setCursor(15, 25);
	gfx_puts("Seismograph");
	gfx_setTextSize(1);
	gfx_setCursor(15, 49);
	gfx_puts("IE0624");
	gfx_setCursor(15, 60);
	gfx_puts("Lab de Microcontroladores");
	lcd_show_frame();
	console_puts("Now it has a bit of structured graphics.\n");
	console_puts("Press a key for some simple animation.\n");
	msleep(2000);
	/*	(void) console_getc(1); */
	gfx_setTextColor(LCD_YELLOW, LCD_BLACK);
	gfx_setTextSize(1);
//	p1 = 0;
//	p2 = 45;
//	p3 = 90;

	/*Codigo tomado de funcion main() de spi-mems.c*/
	baseline[0] = 0;
	baseline[1] = 0;
	baseline[2] = 0;
/*	console_puts("MEMS demo (new version):\n");
	console_puts("Press a key to read the registers\n");
	console_getc(1);
	tmp = read_reg(0xf);
	if (tmp != 0xD4) {
		console_puts("Maybe this isn't a MEMSoscope.\n");
	}*/
	/*
	 * These parameters are sort of random, clearly I need
	 * set something. Based on the app note I reset the 'baseline'
	 * values after 100 samples. But don't see a lot of change
	 * when I move the board around. Z doesn't move at all but the
	 * temperature reading is correct and the ID code returned is
	 * as expected so the SPI code at least is working.
	 */
/*	write_reg(0x20, 0xcf);  // Normal mode 
	write_reg(0x21, 0x07);  // standard filters 
	write_reg(0x23, 0xb0);  // 250 dps 
	tmp = (int) read_reg(0x26);
	console_puts("Temperature: ");
	print_decimal(tmp);
	console_puts(" C\n");
*/
	while (1) {

		//Codigo tomado de funcion main() de spi-mems.c
		tmp = read_xyz(vecs);
		for (i = 0; i < 3; i++) {
			int pad;
		//	console_puts(axes[i]);
			tmp = vecs[i] - baseline[i];
			pad = print_decimal(tmp);
			pad = 15 - pad;
		//	while (pad--) {
		//		console_puts(" ");
		//	}
/*			
			gfx_setCursor(15,cursor);
			gfx_puts(lcd_out);
			sprintf(lcd_out, "%s", axes[i]);
			sprintf(int_to_str, "%d", vecs[i]);
			strcat(lcd_out, int_to_str);
			cursor = cursor + 54;*/ 
		}

//		cursor = 36;
//		console_putc('\r');
		if (count == 100) {
			baseline[0] = vecs[0];
			baseline[1] = vecs[1];
			baseline[2] = vecs[2];
		} else {
			count++;
		}
		msleep(100);
		
		temp = read_reg(0x26);

	//	gfx_fillScreen(LCD_BLACK);
	//	gfx_setCursor(15, 36);
	//	gfx_puts("Seismograph");
	//	gfx_fillCircle(120, 160, 40, LCD_YELLOW);
	//	gfx_drawCircle(120, 160, 55, LCD_GREY);
	//	gfx_drawCircle(120, 160, 75, LCD_GREY);
	//	gfx_drawCircle(120, 160, 100, LCD_GREY);

	//	gfx_fillCircle(120 + (sin(d2r(p1)) * 55),
	//		       160 + (cos(d2r(p1)) * 55), 5, LCD_RED);
	//	gfx_fillCircle(120 + (sin(d2r(p2)) * 75),
	//		       160 + (cos(d2r(p2)) * 75), 10, LCD_WHITE);
	//	gfx_fillCircle(120 + (sin(d2r(p3)) * 100),
	//		       160 + (cos(d2r(p3)) * 100), 8, LCD_BLUE);
	//	p1 = (p1 + 3) % 360;
	//	p2 = (p2 + 2) % 360;
	//	p3 = (p3 + 1) % 360;

		gfx_fillScreen(LCD_BLACK);
		gfx_setCursor(15, 36);
		gfx_puts("Seismograph");
		
		
		
		sprintf(lcd_out, "%s", axes[0]);
		sprintf(int_to_str, "%d", vecs[0]);
		strcat(lcd_out, int_to_str);
		gfx_setCursor(15,90);
		gfx_puts(lcd_out);


		sprintf(lcd_out, "%s", axes[1]);
		sprintf(int_to_str, "%d", vecs[1]);
		strcat(lcd_out, int_to_str);
		gfx_setCursor(15,144);
		gfx_puts(lcd_out);

		sprintf(lcd_out, "%s", axes[2]);
		sprintf(int_to_str, "%d", vecs[2]);
		strcat(lcd_out, int_to_str);
		gfx_setCursor(15,198);
		gfx_puts(lcd_out);

		sprintf(lcd_out, "%s", "Temperature:");
		sprintf(int_to_str, "%d", temp);
		strcat(lcd_out, int_to_str);
		gfx_setCursor(15, 232);
		gfx_puts(lcd_out);
		
		lcd_show_frame();

	}








}