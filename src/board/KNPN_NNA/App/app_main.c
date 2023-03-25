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
#include "Photorezistor/photorezistor.h"
#include "1Wire_DS18B20/one_wire.h"
#include "ina219/inc/ina219_helper.h"
#include "I2C_crutch/i2c-crutch.h"

extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;


#pragma pack(push, 1)
typedef struct paket_3{
	uint8_t flag;
	uint32_t time_pak;
	uint16_t n;
	int16_t temp;
	float latitude;
	float longitude;
	int16_t height;
	uint32_t time_s;
	uint32_t time_us;
	uint8_t fix;
	uint16_t crc;
}paket_3;

typedef struct paket_1{
	uint8_t flag;
	uint32_t time_pak;
	uint16_t n;
	int16_t temp_bme;
	uint32_t press;
	float current;
	int16_t bus_voltage;
	int8_t state;
	uint32_t photor;
	uint16_t crc;
}paket_1;

typedef struct paket_2{
	uint8_t flag;
	uint32_t time_pak;
	uint16_t n;
	int16_t lis_x;
	int16_t lis_y;
	int16_t lis_z;
	int16_t lsm_a_x;
	int16_t ism_a_y;
	int16_t lsm_a_z;
	int16_t lsm_g_x;
	int16_t lsm_g_y;
	int16_t lsm_g_z;
	uint16_t crc;
}paket_2;

#pragma pack(pop)

typedef struct INA219_DATA{
	uint16_t power;
	uint16_t current;
	uint16_t voltage;
	uint16_t shunt_voltage;
}INA219_DATA;

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

uint16_t Crc16(uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    uint8_t i;
    while (len--) {
        crc ^= *buf++ << 8;
        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

typedef enum
{
	NRF_PACK_12,
	NRF_PACK_3,
	NRF_WAIT,
}nrf_state_t;


int app_main(){

	float power;
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
	float photor;
	uint16_t ds_temp;
	nrf24_fifo_status_t  rx_status;
	nrf24_fifo_status_t  tx_status;
	nrf_state_t nrf_state = NRF_PACK_12;
	int mission_state = 0;
	int a = 0;
	//GPIO_PinState IA = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	//GPIO_PinState IB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	int IAB = 0;
	bool storona_levo = 0;
	bool storona_pravo = 0;
	uint32_t nrf_start_time;
	uint32_t ds_start_time;
	int32_t prevCounter = 0;



	//memset(&ina219,0,sizeof(ina219));

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

	// Настройка ds
	ds18b20_t ds_data;
	ds_data.onewire_pin = One_Wire_Pin;
	ds_data.onewire_port = One_Wire_GPIO_Port;

	// Настройка фоторезистора
	photorezistor_t phor_sr;
	phor_sr.hadc = &hadc1;
	phor_sr.resist = 2000;


	ina219_primary_data_t primary_data;

	ina219_secondary_data_t secondary_data;

	ina219_init_default(&ina219,&hi2c1,INA219_I2CADDR_A1_GND_A0_GND, HAL_MAX_DELAY);

	int comp;

	ds18b20_start_conversion(&ds_data);
	ds_start_time = HAL_GetTick();

	paket_1 p1_sr;
	paket_2 p2_sr;
	paket_3 p3_sr;

	p1_sr.n = 0;
	p2_sr.n = 0;
	p3_sr.n = 0;



	while(1){

		/*
		if(IA == 1 & IB == 0){
			storona_levo = !storona_levo;
			IAB ++;
		}
		if(IA == 0 & IB == 1){
			storona_pravo = !storona_pravo;
			IAB --;
		}
		if(IAB >= 64){
			IAB = 0;
			printf("ееее 360 ");
			// и мы вроде наверное прошли 360 градусов
			// 1 это 5,625 градусов (прям как у пива)
		}
		*/


	    // так можно проставить начальное значение счетчика:
		// __HAL_TIM_SET_COUNTER(&htim1, 32760);

		//включаем таймер
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

		int currCounter = __HAL_TIM_GET_COUNTER(&htim3);
		currCounter = 32767 - ((currCounter-1) & 0xFFFF);
		if(currCounter != prevCounter) {
			char buff[16];

			// выводим куда-то currCounter
			// snprintf(buff, sizeof(buff), "%06d", currCounter);

			prevCounter = currCounter;
		}





		lsmread(&stm_ctx, &lsm_temp, &lsm_accel, &lsm_gyro);
		lisread(&lis_ctx, &lis_temp, &lis);
		bme_data = bme_read_data(&bme);
		photor = photorezistor_get_lux(phor_sr);


		if (HAL_GetTick() - ds_start_time > 750)
		{
			ds18b20_read_raw_temperature(&ds_data, &ds_temp,0);
			ds18b20_start_conversion(&ds_data);
			ds_start_time = HAL_GetTick();
		}



		int res = ina219_read_primary(&ina219,&primary_data);
		if (res == 2)
		{
			I2C_ClearBusyFlagErratum(&hi2c1, 20);
			reset_i2c_1();
		}
		res = ina219_read_secondary(&ina219,&secondary_data);
		if (res == 2)
		{
			I2C_ClearBusyFlagErratum(&hi2c1, 20);
			reset_i2c_1();
		}


		power = ina219_power_convert(&ina219, secondary_data.power);

		current = ina219_current_convert(&ina219, secondary_data.current);

		bus_voltage = ina219_bus_voltage_convert(&ina219, primary_data.busv);

		shunt_voltage = ina219_shunt_voltage_convert(&ina219, primary_data.shuntv);

		p1_sr.flag = 0xAA;
		p1_sr.press = bme_data.pressure;
		p1_sr.photor = photor * 100;
		p1_sr.temp_bme = bme_data.temperature * 100;
		p1_sr.current = current * 1000;
		p1_sr.bus_voltage = bus_voltage * 1000;
		p1_sr.state = mission_state;

		p2_sr.flag =0xBB;
		p2_sr.lis_x = lis[0] * 1000;
		p2_sr.lis_y = lis[1] * 1000;
		p2_sr.lis_z = lis[2] * 1000;
		p2_sr.lsm_a_x = lsm_accel[0] * 1000;
		p2_sr.ism_a_y = lsm_accel[1] * 1000;
		p2_sr.lsm_a_z = lsm_accel[2] * 1000;
		p2_sr.lsm_g_x = lsm_gyro[0] * 1000;
		p2_sr.lsm_g_y = lsm_gyro[1] * 1000;
		p2_sr.lsm_g_z = lsm_gyro[2] * 1000;

		p3_sr.flag = 0xCC;
		p3_sr.temp = ds_temp;
		p3_sr.latitude = 3;
		p3_sr.longitude = 4;
		p3_sr.time_s = 7;
		p3_sr.time_us = 4325;
		p3_sr.fix = 5;
		p3_sr.height = 1;

		switch(nrf_state) {
			case NRF_PACK_12:
				p1_sr.n++;
				p2_sr.n++;
				p1_sr.time_pak = HAL_GetTick();
				p2_sr.time_pak = HAL_GetTick();
				p1_sr.crc = Crc16((uint8_t *)&p1_sr, sizeof(p1_sr) - 2);
				p2_sr.crc = Crc16((uint8_t *)&p2_sr, sizeof(p2_sr) - 2);
				nrf24_fifo_write(&nrf, (uint8_t *)&p1_sr,sizeof(p1_sr),false);
				nrf24_fifo_write(&nrf, (uint8_t *)&p2_sr,sizeof(p2_sr),false);
				a++;
				nrf_state = NRF_WAIT;
				nrf_start_time = HAL_GetTick();
			break;
			case NRF_PACK_3:
				p3_sr.n++;
				p3_sr.time_pak = HAL_GetTick();
				p3_sr.crc = Crc16((uint8_t *)&p3_sr, sizeof(p3_sr) - 2);
				nrf24_fifo_write(&nrf, (uint8_t *)&p3_sr,sizeof(p3_sr),false);
				a = 0;
				nrf_state = NRF_WAIT;
				nrf_start_time = HAL_GetTick();
			break;
			case NRF_WAIT:
				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET){
					nrf24_irq_get(&nrf, &comp);
					nrf24_irq_clear(&nrf, comp);
					nrf24_fifo_status(&nrf,&rx_status,&tx_status);

					if (tx_status == NRF24_FIFO_EMPTY)
					{
						if (a == 4)
						{
							nrf_state = NRF_PACK_3;
						}
						else
						{
							nrf_state = NRF_PACK_12;
						}
						break;
					}
				}
				if (HAL_GetTick() - nrf_start_time > 100)
				{
					nrf24_fifo_flush_tx(&nrf);
					if (a == 4)
					{
						nrf_state = NRF_PACK_3;
					}
					else
					{
						nrf_state = NRF_PACK_12;
					}
					break;

				}
				break;
			}

/*
		printf("АКСЕЛЕРОМЕТР X %f\n АКСЕЛЕРОМЕТР Y %f\n АКСЕЛЕРОМЕТР Z %f\n", lsm_accel[0], lsm_accel[1], lsm_accel[2]);
		printf("ГИРОСКОП X %f\n ГИРОСКОП Y %f\n ГИРОСКОП Z %f\n", lsm_gyro[0], lsm_gyro[1], lsm_gyro[2]);
		printf("Температура %f\n\n\n\n", lsm_temp);
		printf("МАГНИТОМЕТР X %f\n МАГНИТОМЕТР Y %f\n МАГНИТОМЕТР Z %f\n", lis[0], lis[1], lis[2]);
		printf("Давление  %lf \n Температура %lf \n", bme_data.pressure, bme_data.temperature);
		*/
		printf("power %f\n current %f\n bus_voltage %f\n shunt_voltage %f\n" , power, current,bus_voltage,shunt_voltage);
	}


	return 0;
}

