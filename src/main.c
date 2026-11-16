
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "vl53l0x.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* KONFIGURASI KALIBRASI */
#define JUMLAH_TITIK  5
#define JUMLAH_SAMPLE 50
#define MAX_RETRY     150

/* Jarak referensi dalam mm */
static const float jarak_referensi[JUMLAH_TITIK] =
{
    100.0f, 200.0f, 300.0f, 400.0f, 500.0f
};

/* Variabel kalibrasi */
static float data_sensor[JUMLAH_TITIK];
static float offset = 0.0f;
static float gain = 1.0f;
static uint8_t kalibrasi_sukses = 0;

/* PROTOTYPE */
void SystemClock_Config(void);
void Error_Handler(void);
int Kalibrasi_Sensor(void);
void Hitung_Koreksi(void);
void Baca_Sensor(void);

/* OUTPUT UART */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,
                      HAL_MAX_DELAY);
    return ch;
}

/* PENGAMBILAN DATA SENSOR */
int Ambil_Sampel(float *hasil)
{
    uint32_t total = 0;
    uint16_t nilai;
    uint16_t valid = 0;
    uint16_t retry = 0;

    while (valid < JUMLAH_SAMPLE && retry < MAX_RETRY)
    {
        retry++;

        nilai = VL53L0X_ReadRangeSingleMM();

        if (!VL53L0X_TimeoutOccurred() && nilai < 8190)
        {
            if (nilai > 0)
            {
                total += nilai;
                valid++;
            }
        }

        HAL_Delay(50);
    }

    if (valid < JUMLAH_SAMPLE)
        return 0;

    *hasil = (float)total / (float)valid;

    return 1;
}

/* PROSES KALIBRASI 5 TITIK */
int Kalibrasi_Sensor(void)
{
    printf("\r\n");
    printf("================================\r\n");
    printf("     KALIBRASI SENSOR VL53L0X\r\n");
    printf("================================\r\n");

    for (int i = 0; i < JUMLAH_TITIK; i++)
    {
        printf("\r\nTitik kalibrasi %d dari %d\r\n",
               i + 1, JUMLAH_TITIK);

        printf("Letakkan objek pada jarak %.0f mm\r\n",
               jarak_referensi[i]);

        printf("Pengukuran dimulai dalam 5 detik...\r\n");

        HAL_Delay(5000);

        printf("Mengambil %d sampel...\r\n",
               JUMLAH_SAMPLE);

        if (!Ambil_Sampel(&data_sensor[i]))
        {
            printf("GAGAL mengambil data!\r\n");
            return 0;
        }

        printf("Jarak referensi : %.2f mm\r\n",
               jarak_referensi[i]);

        printf("Rata-rata sensor: %.2f mm\r\n",
               data_sensor[i]);

        printf("Selisih         : %.2f mm\r\n",
               jarak_referensi[i] - data_sensor[i]);

        printf("Titik %d berhasil!\r\n", i + 1);
    }

    return 1;
}

/* PERHITUNGAN REGRESI LINEAR */
void Hitung_Koreksi(void)
{
    float sum_x = 0;
    float sum_y = 0;
    float sum_xy = 0;
    float sum_x2 = 0;

    float n = (float)JUMLAH_TITIK;

    for (int i = 0; i < JUMLAH_TITIK; i++)
    {
        float x = data_sensor[i];
        float y = jarak_referensi[i];

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    float penyebut = (n * sum_x2) - (sum_x * sum_x);

    if (fabsf(penyebut) < 0.001f)
    {
        kalibrasi_sukses = 0;
        return;
    }

    gain = ((n * sum_xy) - (sum_x * sum_y))
           / penyebut;

    offset = (sum_y - (gain * sum_x)) / n;

    if (gain <= 0.0f)
    {
        kalibrasi_sukses = 0;
        return;
    }

    kalibrasi_sukses = 1;

    printf("\r\n");
    printf("================================\r\n");
    printf("       HASIL KALIBRASI\r\n");
    printf("================================\r\n");

    printf("Gain   : %.6f\r\n", gain);
    printf("Offset : %.2f mm\r\n", offset);

    printf("\r\nPersamaan koreksi:\r\n");
    printf("Jarak = (Gain x Sensor) + Offset\r\n");

    printf("\r\nKalibrasi berhasil!\r\n");
}

/* PEMBACAAN SENSOR TERKALIBRASI */
void Baca_Sensor(void)
{
    uint16_t jarak;
    float hasil_koreksi;

    jarak = VL53L0X_ReadRangeSingleMM();

    if (VL53L0X_TimeoutOccurred() || jarak >= 8190)
    {
        printf("ERROR: Pembacaan sensor timeout!\r\n");
        return;
    }

    if (!kalibrasi_sukses)
    {
        printf("Sensor belum dikalibrasi!\r\n");
        return;
    }

    hasil_koreksi = (gain * (float)jarak) + offset;

    if (hasil_koreksi < 0)
        hasil_koreksi = 0;

    printf("Raw: %u mm | Koreksi: %.2f mm\r\n",
           jarak, hasil_koreksi);
}

/* PROGRAM UTAMA */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    printf("\r\n");
    printf("================================\r\n");
    printf("       VL53L0X STM32 BLUEPILL\r\n");
    printf("================================\r\n");

    VL53L0X_SetTimeout(500);

    if (!VL53L0X_Init(&hi2c1))
    {
        printf("Sensor VL53L0X gagal diinisialisasi!\r\n");

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    printf("Sensor berhasil diinisialisasi!\r\n");

    HAL_Delay(1000);

    /* MENJALANKAN KALIBRASI */
    if (Kalibrasi_Sensor())
    {
        Hitung_Koreksi();
    }
    else
    {
        printf("\r\nKalibrasi gagal!\r\n");
        printf("Periksa sensor dan posisi objek.\r\n");

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    printf("\r\nMulai pembacaan jarak...\r\n");

    /* LOOP PEMBACAAN */
    while (1)
    {
        Baca_Sensor();
        HAL_Delay(200);
    }
}

/* KONFIGURASI CLOCK STM32F103 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue =
        RCC_HSE_PREDIV_DIV1;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ERROR HANDLER */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
