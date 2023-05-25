/*
 * cfg.c
 *
 *  Created on: 24 мая 2023 г.
 *      Author: Install
 */

#include "BME280/DriverForBME280.h"
#include "LSM6DS3/DLSM.h"
#include "LIS3MDL/DLIS3.h"
#include <stm32f4xx_hal.h>
#include "nRF24L01_PL/nrf24_upper_api.h"
#include "nRF24L01_PL/nrf24_lower_api_stm32.h"
#include "Photorezistor/photorezistor.h"
#include "1Wire_DS18B20/one_wire.h"
#include "ina219/inc/ina219_helper.h"
#include "I2C_crutch/i2c-crutch.h"
#include "fatfs.h"
#include "string.h"
#include "ATGM336H/nmea_gps.h"
#include "AD5593/ad5593.h"
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart6;

float power;
char str_buf[300];
uint16_t num_written;
float current;
float bus_voltage;
float shunt_voltage;
float lsm_gyro[3];
float lsm_temp;
float lsm_accel[3];
float lis[3];
float lis_temp;
struct bme280_data bme_data;
struct bme280_dev bme;
struct ina219_t ina219;


void init_sr(shift_reg_t *imu_sr)
{
	// Настройка сдвигового регистра IMU

	imu_sr->latch_port = GPIOC;
	imu_sr->latch_pin = GPIO_PIN_1;
	imu_sr->oe_port = GPIOC;
	imu_sr->oe_pin = GPIO_PIN_13;
	imu_sr->value = 0;
	imu_sr->bus = &hspi2;
	shift_reg_init(imu_sr);
	shift_reg_write_16(imu_sr, 0xFFFF);
}

void init_dop_sr(shift_reg_t *dop_sr)
{
	// Настройка сдвигового регистра доп
	dop_sr->latch_port = GPIOB;
	dop_sr->latch_pin = GPIO_PIN_12;
	dop_sr->oe_port = GPIOB;
	dop_sr->oe_pin = GPIO_PIN_12;
	dop_sr->value = 0;
	dop_sr->bus = &hspi2;
	shift_reg_init(dop_sr);
	shift_reg_write_16(dop_sr, 0x0000);
}

void init_sr_radio(shift_reg_t *nrf_sr)
{
	// Настройка сдвигового регистра NRF
	nrf_sr->latch_port = GPIOC;
	nrf_sr->latch_pin = GPIO_PIN_4;
	nrf_sr->oe_port = GPIOC;
	nrf_sr->oe_pin = GPIO_PIN_5;
	nrf_sr->value = 0;
	nrf_sr->bus = &hspi2;
	shift_reg_init(nrf_sr);
	shift_reg_write_8(nrf_sr, 0xFF);
}

void init_radio(nrf24_lower_api_config_t* nrf, shift_reg_t *nrf_sr)
{
	// Настройка nrf
	nrf24_spi_pins_sr_t nrf_cfg;
	nrf_cfg.pos_CE = 0;
	nrf_cfg.pos_CS = 1;
	nrf_cfg.this = nrf_sr;

	nrf24_spi_init_sr(nrf, &hspi2, &nrf_cfg);

	nrf24_rf_config_t nrf_config;
	nrf24_protocol_config_t nrf_protocol_config;

	nrf24_mode_power_down(&nrf);

	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.rf_channel = 101;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_18_DBM;
	nrf24_setup_rf(&nrf, &nrf_config);

	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.auto_retransmit_count = 0;
	nrf_protocol_config.auto_retransmit_delay = 0;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.en_ack_payload = true;
	nrf_protocol_config.en_dyn_ack = true;
	nrf_protocol_config.en_dyn_payload_size = true;
	nrf24_setup_protocol(&nrf, &nrf_protocol_config);


	nrf24_pipe_set_tx_addr(&nrf,0xacacacacac);

	//настройка пайпа(штука , чтобы принимать)
	nrf24_pipe_config_t pipe_config;
	for (int i = 1; i < 6; i++)
	{
		pipe_config.address = 0xacacacacac;
		pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
		pipe_config.enable_auto_ack = false;
		pipe_config.payload_size = -1;
		nrf24_pipe_rx_start(&nrf, i, &pipe_config);
	}

	pipe_config.address = 0xafafafaf01;
	pipe_config.enable_auto_ack = false;
	pipe_config.payload_size = -1;

	nrf24_pipe_rx_start(&nrf, 0, &pipe_config);

	nrf24_mode_standby(&nrf);
	nrf24_mode_tx(&nrf);
}

void init_lsm(stmdev_ctx_t *lsm_ctx, shift_reg_t *imu_sr)
{
	// Настройка Lsm
	lsm_spi_intf_sr lsm_cfg;
	lsm_cfg.spi = &hspi2;
	lsm_cfg.sr = imu_sr;
	lsm_cfg.sr_pin = 4;

	lsmset_sr(lsm_ctx, &lsm_cfg);
}

void init_lis(	stmdev_ctx_t *lis_ctx, shift_reg_t *imu_sr)
{
	// Настройка Lis
	lis_spi_intf_sr lis_cfg;
	lis_cfg.spi = &hspi2;
	lis_cfg.sr = imu_sr;
	lis_cfg.sr_pin = 3;

	lisset_sr(lis_ctx, &lis_cfg);
}
void init_bme(	bme_spi_intf_sr *bme_cfg, shift_reg_t *imu_sr)
{
	// Настройка bme
	bme_cfg->spi = &hspi2;
	bme_cfg->sr = imu_sr;
	bme_cfg->sr_pin = 2;

	bme_init_default_sr(&bme, bme_cfg);
}

void init_ds(	ds18b20_t *ds_cfg)
{
	// Настройка ds
	ds_cfg->onewire_pin = One_Wire_Pin;
	ds_cfg->onewire_port = One_Wire_GPIO_Port;
}

void init_photor(	photorezistor_t *phor_cfg)
{
	// Настройка фоторезистора
	phor_cfg->hadc = &hadc1;
	phor_cfg->resist = 2000;
}
