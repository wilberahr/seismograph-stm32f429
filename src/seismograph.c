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

////////////////////////////////////////////////////////////////
/**
 * adc-dac-printf.c
 */

#define LED_DISCO_GREEN_PORT GPIOG
#define LED_DISCO_GREEN_PIN GPIO13

#define USART_CONSOLE USART1


int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOG);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART2 and dac */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_DAC);

	/* And ADC*/
	rcc_periph_clock_enable(RCC_ADC1);
}

static void usart_setup(void)
{
	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

static void adc_setup(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);

	adc_power_on(ADC1);

}

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

/***
 * Main, contiene declaraciones de los tres archivos
 */
int main(void)
{










}