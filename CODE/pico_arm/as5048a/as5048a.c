
#include "as5048a.h"

static void as5048a_cs_select(uint8_t device_id) {
	asm volatile("nop \n nop \n nop");
	switch (device_id) {
	case 0:
		gpio_put(SPI_0_CS_GPIO_PIN, 0);
		break;
	case 1:
		gpio_put(SPI_1_CS_GPIO_PIN, 0);
		break;
	default:
		break;
	}
	asm volatile("nop \n nop \n nop");
}

static void as5048a_cs_deselect(uint8_t device_id) {
	asm volatile("nop \n nop \n nop");
	switch (device_id) {
	case 0:
		gpio_put(SPI_0_CS_GPIO_PIN, 1);
		break;
	case 1:
		gpio_put(SPI_1_CS_GPIO_PIN, 1);
		break;
	default:
		break;
	}
	asm volatile("nop \n nop \n nop");
}

static uint8_t spi_calc_even_parity(uint16_t value) {
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < 16; i++) {
		if (value & 0x1) {
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

void as5048a_init_device() {

	spi_init(SPI_PORT_0, 1000 * 1000);
	spi_init(SPI_PORT_1, 1000 * 1000);
	
	spi_set_format(SPI_PORT_0, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
	spi_set_format(SPI_PORT_1, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(SPI_0_MISO_GPIO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_0_SCK_GPIO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_0_MOSI_GPIO_PIN, GPIO_FUNC_SPI);

    gpio_set_function(SPI_1_MISO_GPIO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_1_SCK_GPIO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_1_MOSI_GPIO_PIN, GPIO_FUNC_SPI);

    bi_decl(bi_3pins_with_func(SPI_0_MISO_GPIO_PIN, SPI_0_MOSI_GPIO_PIN, SPI_0_SCK_GPIO_PIN, GPIO_FUNC_SPI));
    bi_decl(bi_3pins_with_func(SPI_1_MISO_GPIO_PIN, SPI_1_MOSI_GPIO_PIN, SPI_1_SCK_GPIO_PIN, GPIO_FUNC_SPI));

	gpio_init(SPI_0_CS_GPIO_PIN);
	gpio_set_dir(SPI_0_CS_GPIO_PIN, GPIO_OUT);
	gpio_put(SPI_0_CS_GPIO_PIN, 1);

	gpio_init(SPI_1_CS_GPIO_PIN);
	gpio_set_dir(SPI_1_CS_GPIO_PIN, GPIO_OUT);
	gpio_put(SPI_1_CS_GPIO_PIN, 1);

}

static uint16_t as5048a_read16(uint8_t device_id, uint16_t reg) {
	// PAR=0 R/W=R
	uint16_t command = 0b0100000000000000;
	command = command | reg;

	// Add a parity bit on the the MSB
	command |= ((uint16_t)spi_calc_even_parity(command) << 15);

	uint16_t response;

	as5048a_cs_select(device_id);
	switch (device_id) {
	case 0:
		spi_write16_read16_blocking(SPI_PORT_0, &command, &response, 1);
		break;
	case 1:
		spi_write16_read16_blocking(SPI_PORT_1, &command, &response, 1);
		break;
	default:
		break;
	}
	as5048a_cs_deselect(device_id);	

	if (response & 0x4000) {
		if (AS5048_DEBUG) {
			printf("Error: Setting error bit\n");
		}
	}
	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

uint16_t as5048a_get_raw_rotation(uint8_t device_id) {
	return as5048a_read16(device_id, AS5048A_ANGLE);
}

double as5048a_get_angle_in_rad(uint8_t device_id) {
	return ( (double)as5048a_get_raw_rotation(device_id) / AS5048A_RESOLUTION ) * M_PI * 2;
}

uint16_t as5048a_get_simple_avr_raw_rotation(uint8_t device_id, uint16_t n) {
	uint32_t sum = 0;
	for (uint i = 0; i < n; i++) {
		sum = sum + as5048a_read16(device_id, AS5048A_ANGLE);
	}
	return (uint16_t)(sum / n);
}
