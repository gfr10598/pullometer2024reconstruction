/*
Just collecting the data is likely to require about 30% of one CPU,
because the CPU is busy when reading the data over I2C.
This will likely mean power consumption of at least 20mA just to read
the data.  We can hopefully use the other processor to concurrently
write the data to the flash.

We can download 32 records at a time, which is 16 samples, which is
about 8 msec.  So, we might want to download on a timer, say about
every 5 msec, to keep the FIFO from overflowing.

This could allow us to wake up every 5 msec, read the data, post it
to the other processor through a queue, and go back
into light sleep.
Periodically, the other processor will write the data to flash.
Apparently the SD card write speed is about 200kB/sec, so a 4kB
write will take about 20 msec.  But the incoming data rate, before
expanding to base64, is about 2*2*7byte/ms = 28kB/sec.  So the
processor will only require about 20% duty cycle to keep up.

*/

#include "Arduino.h"
#include <stdio.h>
#include "LSM6DSV16XSensor.h"
#include <string>
#include "base64_encode.hpp"

#define FIFO_SAMPLE_THRESHOLD 20
#define FLASH_BUFF_LEN 8192
// If we send the raw data to the other processor for output to WiFi,
// we can likely keep up with 3840 samples/sec.
#define SENSOR_ODR 1920

LSM6DSV16XSensor init_lsm()
{
    // Initialize i2c.
    // We need to read roughly 7*2*2khz = 28k bytes per second from the LSM6DSV16X.
    // Because we are reading many bytes at a time, 1MHz I2C can provide perhaps
    // 100k bytes/sec.  So we will be running around 30% duty cycle just reading the data.
    Wire.begin(3, 4, 1000000);
    printf("I2C initialized\n");
    LSM6DSV16XSensor LSM(&Wire);
    printf("LSM created\n");
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(13, LOW);
        delay(100);
        digitalWrite(13, HIGH);
        delay(100);
    }

    if (LSM6DSV16X_OK != LSM.begin())
    {
        printf("LSM.begin() Error\n");
    }
    int32_t status = 0;

    // This doesn't seem to do anything - not seeing any compressed data.
    status |= LSM.FIFO_Set_Compression_Algo(LSM6DSV16X_CMP_16_TO_1);
    status |= LSM.Set_G_FS(1000); // Need minimum of 600 dps
    status |= LSM.Set_X_FS(16);   // To handle large impulses from clapper.
    status |= LSM.Set_X_ODR(SENSOR_ODR);
    status |= LSM.Set_G_ODR(SENSOR_ODR);
    status |= LSM.Set_Temp_ODR(LSM6DSV16X_TEMP_BATCHED_AT_1Hz875);

    // Set FIFO to timestamp data at 20 Hz
    status |= LSM.FIFO_Enable_Timestamp();
    status |= LSM.FIFO_Set_Timestamp_Decimation(LSM6DSV16X_TMSTMP_DEC_32);
    status |= LSM.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);

    // Configure FIFO BDR for acc and gyro
    status |= LSM.FIFO_Set_X_BDR(SENSOR_ODR);
    status |= LSM.FIFO_Set_G_BDR(SENSOR_ODR);

    // Set FIFO in Continuous mode
    status |= LSM.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

    status |= LSM.Enable_G();
    status |= LSM.Enable_X();

    if (status != LSM6DSV16X_OK)
    {
        printf("LSM6DSV16X Sensor failed to configure FIFO\n");
        while (1)
            ;
    }
    else
        printf("LSM enabled\n");
    return LSM;
}

struct Tag
{
    unsigned int _unused : 1;
    unsigned int cnt : 2;
    unsigned int tag : 5;
};

int read_many(LSM6DSV16XSensor &LSM, uint16_t avail)
{
    uint16_t actual;
    uint8_t records[32 * 7];
    if (LSM6DSV16X_OK != LSM.Read_FIFO_Data(avail, (void *)records, &actual))
    {
        printf("LSM6DSV16X Sensor failed to read FIFO data\n");
        while (1)
            ;
    }

    unsigned char output[64 * 7];
    encode_base64((unsigned char *)records, actual * 7, output);

    printf("%s\n", output);
    return actual;

    for (int i = 0; i < actual; i++)
    {
        uint8_t tag = records[i * 7];
        // Find the first record that has tag=1 and cnt = 0, and the next.
        if (true || (tag & 0xFE) == 0x08)
        {
            int16_t *d = (int16_t *)&records[i * 7 + 1];
            printf("OPT1 %01X %02X %04hX %04hX %04hX  ", (tag >> 1) & 0x03, tag >> 3, d[0], d[1], d[2]);
            tag = records[i * 7 + 7];
            d = (int16_t *)&records[i * 7 + 8];
            printf("%01X %02X %04hX %04hX %04hX\n", (tag >> 1) & 0x03, tag >> 3, d[0], d[1], d[2]);
            break;

            // int16_t *d = (int16_t *)&records[i * 7 + 1];
            // printf("OPT1 %01X %02X %6d %6d %6d  ", (tag >> 1) & 0x03, tag >> 3, d[0], d[1], d[2]);
            // tag = records[i * 7 + 7];
            // d = (int16_t *)&records[i * 7 + 8];
            // printf("%01X %02X %6d %6d %6d  ", (tag >> 1) & 0x03, tag >> 3, d[0], d[1], d[2]);
            // break;
        }
    }
    return actual;
}

extern "C" void app_main()
{
    initArduino();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    LSM6DSV16XSensor LSM = init_lsm();
    printf("LSM initialized\n");

    int led = HIGH;

    int n = 2 * SENSOR_ODR;
    uint64_t gap_start = esp_timer_get_time();
    while (1)
    {
        // Wait until there are FIFO_SAMPLE_THRESHOLD records in the FIFO.
        uint16_t avail = 0;
        for (int status = LSM.FIFO_Get_Num_Samples(&avail);
             (status == LSM6DSV16X_OK) && (avail < FIFO_SAMPLE_THRESHOLD);
             status = LSM.FIFO_Get_Num_Samples(&avail))
        {
        }
        int64_t gap_end = esp_timer_get_time();
        printf("Gap was %5lld ", gap_end - gap_start);

        int actual = read_many(LSM, avail);

        gap_start = esp_timer_get_time();
        printf("  Read time: %5lld  for  %2d  ", gap_start - gap_end, actual);

        n -= actual;
        if (n > 0)
        {
            continue;
        }
        led = !led;
        digitalWrite(13, led);
        // printf("*************************************************************************************** %6lld  1920 samples\n", esp_timer_get_time() / 1000);
        n += 2 * SENSOR_ODR;
    }
}
