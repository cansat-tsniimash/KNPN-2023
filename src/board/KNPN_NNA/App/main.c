#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "ad5593.h"


int main()
{
    // Инициализация
    ad5593_t dev;
    int rc = ad5593_ctor(&dev, AD5593_ADDR_A0_HIGH, "/dev/i2c-1");
    if (0 != rc)
    {
        perror("unable to ctor");
        return EXIT_FAILURE;
    }

    // Сброс прибора
    rc = ad5593_sw_reset(&dev);
    // Ошибка здесь ожидается из-за свинства датчика
    perror("reset error");
    rc = 0;
    // Подождем пока он ребутится
    usleep(300); // 250 мкс по даташиту

    // Включаем питание на чем хотим
    printf("power config\n");
    rc = ad5593_power_config(&dev, AD5593_POWER_DAC_REF, 0x00);
    if (0 != rc)
    {
        perror("unable to power config");
        return EXIT_FAILURE;
    }

    // Аналоговая конфигурация
    printf("analog config\n");
    ad5593_an_config_t an_config;
    an_config.adc_buffer_enable = false;
    an_config.adc_buffer_precharge = false;
    an_config.adc_range = AD5593_RANGE_1REF;
    rc = ad5593_an_config(&dev, &an_config);
    if (0 != rc)
    {
        perror("unable to an config");
        return EXIT_FAILURE;
    }

    // Настройка пинов
    printf("pin config\n");
    rc = ad5593_pin_config(&dev, 0xFF, AD5593_PINMODE_ADC);
    if (0 != rc)
    {
        perror("unable to setup pins");
        return EXIT_FAILURE;
    }

    printf("adc auto config\n");
    const int channel_mask = 
        (1 << AD5593_ADC_0) | 
        (1 << AD5593_ADC_1) | 
        (1 << AD5593_ADC_2) |
        (1 << AD5593_ADC_TEMP)
    ;
    rc = ad5593_adc_auto_setup(&dev, channel_mask, true);
    if (0 != rc)
    {
        perror("unable to setup auto");
        return EXIT_FAILURE;
    }

    for (int i = 0; ; i++)
    {
        uint16_t raw;
        ad5593_channel_id_t channel = AD5593_ADC_1;
        //rc = ad5593_adc_read(&dev, channel, &raw);
        rc = ad5593_adc_auto_read(&dev, &channel, &raw);
        if (0 != rc)
            perror("bad read");

        printf("%d, %d, %d, 0x%04X\n", i, rc, channel, raw);
        usleep(100*1000);
    }

    return 0;
}
