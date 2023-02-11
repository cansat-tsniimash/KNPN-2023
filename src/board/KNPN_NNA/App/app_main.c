/*
 * app_main.c
 *
 *  Created on: Dec 17, 2022
 *      Author: Install
 */
#include "BME280/DriverForBME280.h"
#include "LSM6DS3/DLSM.h"
#include "LIS3MDL/DLIS3.h"
#include <stm32f4xx_hal.h>
#include "nRF24L01_PL/nrf24_upper_api.h"
#include "nRF24L01_PL/nrf24_lower_api_stm32.h"

/*typedef struct {
	SPI_HandleTypeDef *bus; // Хэндлер шины SPI
	GPIO_TypeDef *latch_port; // Порт Latch-а, например, GPIOA, GPIOB, etc
	uint16_t latch_pin; // Маска Latch-а, например, GPIO_Pin_1, GPIO_Pin_2, etc
	GPIO_TypeDef *oe_port; // Порт OE, например, GPIOA, GPIOB, etc
	uint16_t oe_pin; // Маска OE, например, GPIO_Pin_1, GPIO_Pin_2, etc
	uint16_t value; // Текущее состояние сдвигового регистра
} shift_reg_t;
	return o;
}*/

extern SPI_HandleTypeDef hspi2;

float lsm_gyro[3];
float lsm_temp;
float lsm_accel[3];
float lis[3];
float lis_temp;
struct bme280_data bme_data;
struct bme280_dev bme;
uint8_t buf[32] = {1, 2, 3};
nrf24_fifo_status_t  rx_status;
nrf24_fifo_status_t  tx_status;

int _write(int file, char *ptr, int len)
{
	extern UART_HandleTypeDef huart1;
	HAL_UART_Transmit(
			&huart1,
			(uint8_t*)ptr, len,
			HAL_MAX_DELAY
	);
	return len;
}

int app_main(){

	// Настройка сдвигового регистра IMU
	shift_reg_t imu_sr;
	imu_sr.latch_port = GPIOC;
	imu_sr.latch_pin = GPIO_PIN_1;
	imu_sr.oe_port = GPIOC;
	imu_sr.oe_pin = GPIO_PIN_13;
	imu_sr.value = 0;
	imu_sr.bus = &hspi2;
	shift_reg_init(&imu_sr);
	shift_reg_write_16(&imu_sr, 0xFFFF);

	// Настройка сдвигового регистра NRF
	shift_reg_t nrf_sr;
	nrf_sr.latch_port = GPIOC;
	nrf_sr.latch_pin = GPIO_PIN_4;
	nrf_sr.oe_port = GPIOC;
	nrf_sr.oe_pin = GPIO_PIN_5;
	nrf_sr.value = 0;
	nrf_sr.bus = &hspi2;
	shift_reg_init(&nrf_sr);
	shift_reg_write_8(&nrf_sr, 0xFF);

	// Настройка nrf
	nrf24_spi_pins_sr_t rf_sr;
	rf_sr.pos_CE = 0;
	rf_sr.pos_CS = 1;
	rf_sr.this =  &nrf_sr;
	nrf24_lower_api_config_t nrf;

	nrf24_spi_init_sr(&nrf, &hspi2 ,&rf_sr );

	nrf24_rf_config_t nrf_config;
	nrf24_protocol_config_t nrf_protocol_config;

	nrf_config.data_rate = NRF24_DATARATE_1000_KBIT;
	nrf_config.rf_channel = 36;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf24_setup_rf(&nrf, &nrf_config);

	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.auto_retransmit_count = 10;
	nrf_protocol_config.auto_retransmit_delay = 1;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.en_ack_payload = false;
	nrf_protocol_config.en_dyn_ack = false;
	nrf_protocol_config.en_dyn_payload_size = true;
	nrf24_setup_protocol(&nrf, &nrf_protocol_config);


	nrf24_pipe_set_tx_addr(&nrf,0x1234);
	nrf24_mode_power_down(&nrf);
	nrf24_mode_standby(&nrf);







	// Настройка Lis
	stmdev_ctx_t lis_ctx;
	lis_spi_intf_sr lis_sr;
	lis_sr.spi = &hspi2;
	lis_sr.sr = &imu_sr;
	lis_sr.sr_pin = 3;

	lisset_sr(&lis_ctx, &lis_sr);

	// Настройка Lsm
	stmdev_ctx_t stm_ctx;
	lsm_spi_intf_sr lsm_sr;
	lsm_sr.spi = &hspi2;
	lsm_sr.sr = &imu_sr;
	lsm_sr.sr_pin = 4;

	lsmset_sr(&stm_ctx, &lsm_sr);

	// Настройка bme
	bme_spi_intf_sr bme_sr;
	bme_sr.spi = &hspi2;
	bme_sr.sr = &imu_sr;
	bme_sr.sr_pin = 2;

	bme_init_default_sr(&bme, &bme_sr);


	//  Вывод в консоль

	while(1){
		nrf24_fifo_status(&nrf,&rx_status, &tx_status);
		nrf24_fifo_write(&nrf, buf, 32,false);
if(tx_status == NRF24_FIFO_FULL){

}

		lsmread(&stm_ctx, &lsm_temp, &lsm_accel, &lsm_gyro);
		lisread(&lis_ctx, &lis_temp, &lis);
		bme_data = bme_read_data(&bme);
		printf("АКСЕЛЕРОМЕТР X %f\n АКСЕЛЕРОМЕТР Y %f\n АКСЕЛЕРОМЕТР Z %f\n", lsm_accel[0], lsm_accel[1], lsm_accel[2]);
		printf("ГИРОСКОП X %f\n ГИРОСКОП Y %f\n ГИРОСКОП Z %f\n", lsm_gyro[0], lsm_gyro[1], lsm_gyro[2]);
		printf("Температура %f\n\n\n\n", lsm_temp);
		printf("МАГНИТОМЕТР X %f\n МАГНИТОМЕТР Y %f\n МАГНИТОМЕТР Z %f\n", lis[0], lis[1], lis[2]);
		printf("Давление  %lf \n Температура %lf \n", bme_data.pressure, bme_data.temperature);

	}

	return 0;
}
