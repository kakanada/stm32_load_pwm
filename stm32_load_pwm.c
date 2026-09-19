/**
 ******************************************************************************
 * @file    stm32_load_pwm.c
 * @brief   Реализация библиотеки управления ШИМ-нагрузкой (см. stm32_load_pwm.h).
 * @author  Mechanic
 * @date    19.09.2026
 * @version 0.4
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include "stm32_load_pwm.h"
#include <math.h>   /* fmodf/logf/powf - используются только в Init/Start
                       (не в Tick - см. комментарии ниже) */

/* ------------------------------------------------------------------------ */
/*  Таблицы формы циклов индикации                                          */
/* ------------------------------------------------------------------------ */
/*
 * Каждая таблица - LOAD_PWM_TABLE_SIZE точек визуальной яркости (0..255) на
 * ПОЛНЫЙ период цикла. Размер зависит от LOAD_PWM_TABLE_QUALITY (см. .h):
 * скомпилируется только ОДИН из трёх ниже перечисленных наборов (32/64/128
 * точек) - остальные два отсекаются препроцессором и не занимают Flash.
 * Значения посчитаны один раз офлайн (не в прошивке) и зашиты как константы.
 */

#if LOAD_PWM_TABLE_QUALITY == 0
/* ===== LOW: 32 точки на цикл ===== */
static const uint8_t LOAD_PWM_TableMeandr[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableBreath[LOAD_PWM_TABLE_SIZE] =
{
      0,  25,  50,  74,  98, 120, 142, 162, 180, 197, 212, 225,
    236, 244, 250, 254, 255, 254, 250, 244, 236, 225, 212, 197,
    180, 162, 142, 120,  98,  74,  50,  25
};

static const uint8_t LOAD_PWM_TablePulse[LOAD_PWM_TABLE_SIZE] =
{
      0,  50,  98, 142, 180, 212, 236, 250, 255, 250, 236, 212,
    180, 142,  98,  50,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableTrapezoid[LOAD_PWM_TABLE_SIZE] =
{
      0, 128, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 128,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableSawtooth[LOAD_PWM_TABLE_SIZE] =
{
      0,  17,  34,  51,  68,  85, 102, 119, 136, 153, 170, 187,
    204, 221, 238, 255,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableTriangle[LOAD_PWM_TABLE_SIZE] =
{
      0,  17,  34,  51,  68,  85, 102, 119, 136, 153, 170, 187,
    204, 221, 238, 255, 255, 238, 221, 204, 187, 170, 153, 136,
    119, 102,  85,  68,  51,  34,  17,   0
};

static const uint8_t LOAD_PWM_TableDoubleBlink[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,
    255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableStrobe[LOAD_PWM_TABLE_SIZE] =
{
    255,   0, 255,   0, 255,   0, 255,   0, 255,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

#elif LOAD_PWM_TABLE_QUALITY == 1
/* ===== MEDIUM: 64 точки на цикл ===== */
static const uint8_t LOAD_PWM_TableMeandr[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableBreath[LOAD_PWM_TABLE_SIZE] =
{
      0,  13,  25,  37,  50,  62,  74,  86,  98, 109, 120, 131,
    142, 152, 162, 171, 180, 189, 197, 205, 212, 219, 225, 231,
    236, 240, 244, 247, 250, 252, 254, 255, 255, 255, 254, 252,
    250, 247, 244, 240, 236, 231, 225, 219, 212, 205, 197, 189,
    180, 171, 162, 152, 142, 131, 120, 109,  98,  86,  74,  62,
     50,  37,  25,  13
};

static const uint8_t LOAD_PWM_TablePulse[LOAD_PWM_TABLE_SIZE] =
{
      0,  25,  50,  74,  98, 120, 142, 162, 180, 197, 212, 225,
    236, 244, 250, 254, 255, 254, 250, 244, 236, 225, 212, 197,
    180, 162, 142, 120,  98,  74,  50,  25,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableTrapezoid[LOAD_PWM_TABLE_SIZE] =
{
      0,  51, 102, 153, 204, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 204, 153, 102,  51,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableSawtooth[LOAD_PWM_TABLE_SIZE] =
{
      0,   8,  16,  25,  33,  41,  49,  58,  66,  74,  82,  90,
     99, 107, 115, 123, 132, 140, 148, 156, 165, 173, 181, 189,
    197, 206, 214, 222, 230, 239, 247, 255,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableTriangle[LOAD_PWM_TABLE_SIZE] =
{
      0,   8,  16,  25,  33,  41,  49,  58,  66,  74,  82,  90,
     99, 107, 115, 123, 132, 140, 148, 156, 165, 173, 181, 189,
    197, 206, 214, 222, 230, 239, 247, 255, 255, 247, 239, 230,
    222, 214, 206, 197, 189, 181, 173, 165, 156, 148, 140, 132,
    123, 115, 107,  99,  90,  82,  74,  66,  58,  49,  41,  33,
     25,  16,   8,   0
};

static const uint8_t LOAD_PWM_TableDoubleBlink[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableStrobe[LOAD_PWM_TABLE_SIZE] =
{
    255, 255,   0,   0, 255, 255,   0,   0, 255, 255,   0,   0,
    255, 255,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0
};

#else
/* ===== HIGH: 128 точек на цикл (умолчание, как в версии 0.1) ===== */
static const uint8_t LOAD_PWM_TableMeandr[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableBreath[LOAD_PWM_TABLE_SIZE] =
{
      0,   6,  13,  19,  25,  31,  37,  44,  50,  56,  62,  68,
     74,  80,  86,  92,  98, 103, 109, 115, 120, 126, 131, 136,
    142, 147, 152, 157, 162, 167, 171, 176, 180, 185, 189, 193,
    197, 201, 205, 208, 212, 215, 219, 222, 225, 228, 231, 233,
    236, 238, 240, 242, 244, 246, 247, 249, 250, 251, 252, 253,
    254, 254, 255, 255, 255, 255, 255, 254, 254, 253, 252, 251,
    250, 249, 247, 246, 244, 242, 240, 238, 236, 233, 231, 228,
    225, 222, 219, 215, 212, 208, 205, 201, 197, 193, 189, 185,
    180, 176, 171, 167, 162, 157, 152, 147, 142, 136, 131, 126,
    120, 115, 109, 103,  98,  92,  86,  80,  74,  68,  62,  56,
     50,  44,  37,  31,  25,  19,  13,   6
};

static const uint8_t LOAD_PWM_TablePulse[LOAD_PWM_TABLE_SIZE] =
{
      0,  13,  25,  37,  50,  62,  74,  86,  98, 109, 120, 131,
    142, 152, 162, 171, 180, 189, 197, 205, 212, 219, 225, 231,
    236, 240, 244, 247, 250, 252, 254, 255, 255, 255, 254, 252,
    250, 247, 244, 240, 236, 231, 225, 219, 212, 205, 197, 189,
    180, 171, 162, 152, 142, 131, 120, 109,  98,  86,  74,  62,
     50,  37,  25,  13,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableTrapezoid[LOAD_PWM_TABLE_SIZE] =
{
      0,  21,  42,  64,  85, 106, 128, 149, 170, 191, 212, 234,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 234, 212, 191, 170, 149, 128, 106,  85,
     64,  42,  21,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableSawtooth[LOAD_PWM_TABLE_SIZE] =
{
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  45,
     49,  53,  57,  61,  65,  69,  73,  77,  81,  85,  89,  93,
     97, 101, 105, 109, 113, 117, 121, 125, 130, 134, 138, 142,
    146, 150, 154, 158, 162, 166, 170, 174, 178, 182, 186, 190,
    194, 198, 202, 206, 210, 215, 219, 223, 227, 231, 235, 239,
    243, 247, 251, 255,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableTriangle[LOAD_PWM_TABLE_SIZE] =
{
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  45,
     49,  53,  57,  61,  65,  69,  73,  77,  81,  85,  89,  93,
     97, 101, 105, 109, 113, 117, 121, 125, 130, 134, 138, 142,
    146, 150, 154, 158, 162, 166, 170, 174, 178, 182, 186, 190,
    194, 198, 202, 206, 210, 215, 219, 223, 227, 231, 235, 239,
    243, 247, 251, 255, 255, 251, 247, 243, 239, 235, 231, 227,
    223, 219, 215, 210, 206, 202, 198, 194, 190, 186, 182, 178,
    174, 170, 166, 162, 158, 154, 150, 146, 142, 138, 134, 130,
    125, 121, 117, 113, 109, 105, 101,  97,  93,  89,  85,  81,
     77,  73,  69,  65,  61,  57,  53,  49,  45,  40,  36,  32,
     28,  24,  20,  16,  12,   8,   4,   0
};

static const uint8_t LOAD_PWM_TableDoubleBlink[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

static const uint8_t LOAD_PWM_TableStrobe[LOAD_PWM_TABLE_SIZE] =
{
    255, 255, 255, 255, 255,   0,   0,   0,   0,   0, 255, 255,
    255, 255, 255,   0,   0,   0,   0,   0, 255, 255, 255, 255,
    255,   0,   0,   0,   0,   0, 255, 255, 255, 255, 255,   0,
      0,   0,   0,   0, 255, 255, 255, 255, 255,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

#endif

/** Таблица указателей на таблицы форм, индексируется LOAD_PWM_Cycle_t. */
static const uint8_t * const LOAD_PWM_Tables[LOAD_PWM_CYCLE_COUNT] =
{
    LOAD_PWM_TableMeandr,
    LOAD_PWM_TableBreath,
    LOAD_PWM_TablePulse,
    LOAD_PWM_TableTrapezoid,
    LOAD_PWM_TableSawtooth,
    LOAD_PWM_TableTriangle,
    LOAD_PWM_TableDoubleBlink,
    LOAD_PWM_TableStrobe,
};

/* ------------------------------------------------------------------------ */
/*  Статические пулы (без malloc)                                           */
/* ------------------------------------------------------------------------ */

static LOAD_PWM_Handle_t s_pool[LOAD_PWM_MAX_LOADS];

/** Отдельный пул таблиц гамма-коррекции - ТОЛЬКО для нагрузок типа
 *  LOAD_PWM_TYPE_LED (см. описание в начале .h). Размер каждой записи
 *  зависит от LOAD_PWM_RAM_QUALITY. Линейные нагрузки этот пул не трогают
 *  вообще - их скважность считается по формуле без всякой таблицы.
 *
 *  Размер массива - LOAD_PWM_LED_POOL_SIZE, а не напрямую LOAD_PWM_MAX_LEDS:
 *  если проект целиком без единого LED (законный, поощряемый в README
 *  случай - LOAD_PWM_MAX_LEDS=0 для проекта только с LOAD_PWM_TYPE_LINEAR),
 *  массив нулевого размера - нарушение ISO C99 (только GNU-расширение) и
 *  ломает сборку под -Wpedantic. Минимум 1 "неиспользуемый" слот вместо
 *  этого - несколько лишних байт, зато честная чистая компиляция. Слот не
 *  читается в рантайме при LOAD_PWM_MAX_LEDS=0 - см. LOAD_PWM_Init(). */
#define LOAD_PWM_LED_POOL_SIZE ((LOAD_PWM_MAX_LEDS > 0U) ? LOAD_PWM_MAX_LEDS : 1U)
static LOAD_PWM_Duty_t s_led_lut_pool[LOAD_PWM_LED_POOL_SIZE][LOAD_PWM_GAMMA_LUT_POINTS];

/** Монотонный счётчик занятых слотов s_led_lut_pool - слоты не
 *  освобождаются индивидуально (см. комментарий в LOAD_PWM_Init про
 *  повторную регистрацию и смену типа нагрузки на лету). */
static uint8_t s_led_lut_used_count = 0U;

/** Глобальный "ночной" множитель, см. LOAD_PWM_SetGlobalBrightness. Хранится
 *  как целочисленная дробь Q16 (0..LOAD_PWM_GLOBAL_MULT_FULL = 0..100%), А
 *  НЕ float - иначе load_pwm_write_output() на каждый Tick() каждой активной
 *  нагрузки платил бы за программную эмуляцию float (__aeabi_fmul и т.п.),
 *  что на МК без аппаратного FPU (Cortex-M0/M0+, например STM32F030)
 *  ощутимо дороже целочисленного умножения+сдвига - причём ПОСТОЯННО, даже
 *  если этой функцией никто не пользуется. Применяется только в момент
 *  финальной записи в CCR - внутреннее логическое состояние каждой нагрузки
 *  его не учитывает. */
#define LOAD_PWM_GLOBAL_MULT_SHIFT 16U
#define LOAD_PWM_GLOBAL_MULT_FULL  (1UL << LOAD_PWM_GLOBAL_MULT_SHIFT)
static uint32_t s_global_multiplier_q16 = LOAD_PWM_GLOBAL_MULT_FULL;

/* ------------------------------------------------------------------------ */
/*  Внутренние вспомогательные функции                                      */
/* ------------------------------------------------------------------------ */

/**
 * @brief  Ограничивает процент в диапазон 0..100.
 * @note   NaN-безопасно: сравнение "percent > 0.0f" (а не "percent < 0.0f")
 *         специально выбрано первым - у NaN ЛЮБОЕ сравнение с обычным числом
 *         по IEEE 754 ложно, поэтому "percent < 0.0f" для NaN тоже false, и
 *         NaN прошёл бы через обе проверки НЕИЗМЕНЁННЫМ (нашли бы бы этот
 *         сценарий, только протестировав явно) - а дальше NaN неизбежно
 *         попал бы в приведение float/double -> беззнаковый целый тип, что
 *         является неопределённым поведением по C99. Форма "!(percent>0.0f)"
 *         для NaN истинна (т.к. "percent>0.0f" для NaN ложно) - NaN уходит в
 *         безопасный 0.0f, как и любой другой некорректный/отрицательный
 *         вход. Поведение для нормальных чисел и +-inf не меняется.
 */
static float load_pwm_clamp_percent(float percent)
{
    if (!(percent > 0.0f)) { return 0.0f; }
    if (percent > 100.0f)  { return 100.0f; }
    return percent;
}

/**
 * @brief  Переводит визуальную яркость (0..255) в значение ШИМ (тики CCR).
 *
 *         Для LOAD_PWM_TYPE_LINEAR (h->lut_index == LOAD_PWM_LUT_INDEX_NONE)
 *         - точная линейная формула, без всякой таблицы: LUT для линейной
 *         нагрузки просто не нужна (это не приближение, а математически
 *         точный результат).
 *
 *         Для LOAD_PWM_TYPE_LED - линейная интерполяция между двумя
 *         ближайшими узлами таблицы гамма-коррекции из ОТДЕЛЬНОГО пула
 *         s_led_lut_pool (см. LOAD_PWM_MAX_LEDS в .h) - не хранится в самом
 *         хэндле. Вызывается как из Tick() (продвижение цикла), так и вне
 *         её (SetBrightness) - в обоих случаях быстро (без деления, только
 *         сдвиг/маска/умножение).
 */
static LOAD_PWM_Duty_t load_pwm_visual_to_duty(const LOAD_PWM_Handle_t *h, uint16_t visual)
{
    if (visual >= 255U)
    {
        return h->pwm_max; /* точное верхнее значение, без интерполяционного зазора */
    }

    if (h->lut_index == LOAD_PWM_LUT_INDEX_NONE)
    {
        uint32_t range = (uint32_t)h->pwm_max - (uint32_t)h->pwm_min;
        return (LOAD_PWM_Duty_t)((uint32_t)h->pwm_min + ((range * (uint32_t)visual) / 255U));
    }

    const LOAD_PWM_Duty_t *lut = s_led_lut_pool[h->lut_index];
    uint32_t seg  = (uint32_t)visual >> LOAD_PWM_GAMMA_LUT_INDEX_SHIFT;
    uint32_t frac = (uint32_t)visual & LOAD_PWM_GAMMA_LUT_SEGMENTS_MASK;
    uint32_t d0 = lut[seg];
    uint32_t d1 = lut[seg + 1U];
    return (LOAD_PWM_Duty_t)(d0 + (((d1 - d0) * frac) >> LOAD_PWM_GAMMA_LUT_INDEX_SHIFT));
}

/**
 * @brief  Единая точка записи в CCR - применяет глобальный "ночной"
 *         множитель (LOAD_PWM_SetGlobalBrightness) к логическому (уже
 *         гамма-скорректированному) значению ПЕРЕД выводом в железо.
 *         Внутреннее состояние (background_pwm/current_pwm) всегда хранит
 *         логическое значение БЕЗ множителя.
 *
 *         Вызывается из Tick() на каждый активный тик каждой нагрузки -
 *         обычный случай (множитель == 100%, SetGlobalBrightness ни разу не
 *         вызывался или сброшен на 100%) оптимизирован до простого сравнения
 *         БЕЗ единого умножения - максимально быстро в том числе на МК без
 *         аппаратного умножителя/FPU. "Ночной режим" (множитель < 100%)
 *         платит за одно целочисленное 64-битное произведение вместо
 *         программной эмуляции float, которая потребовалась бы на МК без
 *         FPU (Cortex-M0/M0+).
 */
static void load_pwm_write_output(LOAD_PWM_Handle_t *h, LOAD_PWM_Duty_t logical_duty)
{
    uint32_t out;

    if (s_global_multiplier_q16 >= LOAD_PWM_GLOBAL_MULT_FULL)
    {
        out = (uint32_t)logical_duty;
    }
    else
    {
        /* 64-битное промежуточное произведение - безопасно от переполнения
         * для любого logical_duty (до UINT32_MAX при LOAD_PWM_RAM_QUALITY=
         * HIGH) и любого множителя (до LOAD_PWM_GLOBAL_MULT_FULL). */
        out = (uint32_t)(((uint64_t)logical_duty * s_global_multiplier_q16) >> LOAD_PWM_GLOBAL_MULT_SHIFT);
    }

    __HAL_TIM_SET_COMPARE(h->htim, h->channel, out);
}

/**
 * @brief  Переводит процент фоновой яркости (0..100, полная точность float)
 *         в тики ШИМ для LOAD_PWM_TYPE_LINEAR - НАПРЯМУЮ, без прохождения
 *         через 8-битное "визуальное" представление (0..255), которое в
 *         load_pwm_visual_to_duty существует только ради индексации гамма-LUT
 *         у LED. Если бы LINEAR-нагрузки использовали тот же 8-битный
 *         визуальный путь, что и LED (как было раньше) - LOAD_PWM_SetBrightness
 *         был бы искусственно ограничен 256 достижимыми уровнями скважности
 *         независимо от реального разрешения ARR/LOAD_PWM_RAM_QUALITY, что
 *         прямо противоречит заявленной "точной линейной формуле" для
 *         линейной нагрузки.
 */
static LOAD_PWM_Duty_t load_pwm_percent_to_duty_linear(const LOAD_PWM_Handle_t *h, float percent)
{
    /* double, а не float - у float не хватает точности мантиссы вплотную к
     * UINT32_MAX (актуально при LOAD_PWM_RAM_QUALITY=HIGH), из-за чего
     * duty_d при percent=100 мог бы округлиться ВВЕРХ за пределы диапазона,
     * представимого LOAD_PWM_Duty_t - приведение такого double->uint32_t
     * было бы неопределённым поведением по C99. Вызывается не из Tick() -
     * double не стоит производительности в горячем пути. */
    double range = (double)h->pwm_max - (double)h->pwm_min;
    double duty_d = (double)h->pwm_min + ((double)percent / 100.0) * range;
    return (LOAD_PWM_Duty_t)(duty_d + 0.5); /* округление, не усечение */
}

/** Ищет свободный слот в пуле либо уже зарегистрированный (htim, channel) -
 *  для идемпотентности повторного LOAD_PWM_Init(). NULL, если пул полон и
 *  совпадения не найдено. */
static LOAD_PWM_Handle_t *load_pwm_find_or_alloc_slot(TIM_HandleTypeDef *htim, uint32_t channel)
{
    uint32_t free_index = LOAD_PWM_MAX_LOADS;
    uint32_t has_free = 0U;

    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        if (s_pool[i].used != 0U)
        {
            if ((s_pool[i].htim == htim) && ((uint32_t)s_pool[i].channel == channel))
            {
                return &s_pool[i]; /* уже зарегистрирован - переиспользуем */
            }
        }
        else if (has_free == 0U)
        {
            free_index = i;
            has_free = 1U;
        }
    }

    if (has_free == 0U)
    {
        return NULL; /* пул исчерпан */
    }
    return &s_pool[free_index];
}

/**
 * @brief  Общая внутренняя реализация запуска цикла (используется и
 *         LOAD_PWM_Start, и LOAD_PWM_StartOnce).
 */
static HAL_StatusTypeDef load_pwm_start_internal(LOAD_PWM_Handle_t *h, LOAD_PWM_Cycle_t cycle,
                                                 float phase_shift, uint32_t period_ms,
                                                 LOAD_PWM_Mode_t mode, uint8_t allow_continue)
{
    if ((h == NULL) || (cycle >= LOAD_PWM_CYCLE_COUNT) || (period_ms == 0U))
    {
        if (h == NULL)
        {
            LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_NULL_HANDLE, 0, 0);
        }
        return HAL_ERROR;
    }

    if ((allow_continue != 0U) && (h->mode == (uint8_t)LOAD_PWM_MODE_REPEAT) && (h->cycle_id == (uint8_t)cycle))
    {
        return HAL_OK;
    }

    double ticks_per_period = ((double)period_ms * (double)h->tick_freq_hz_cached) / 1000.0;
    if (ticks_per_period < 1.0)
    {
        ticks_per_period = 1.0;
    }

    /* 2^32 / кол-во тиков за период, округляется до ближайшего (а не
     * усекается) - иначе накопленная за период потеря дробной части
     * систематически сдвигает момент завершения периода на 1 лишний тик. */
    double inc = 4294967296.0 / ticks_per_period;
    uint32_t phase_inc = (inc >= 4294967295.0) ? 0xFFFFFFFFU : (uint32_t)(inc + 0.5);
    if (phase_inc == 0U)
    {
        phase_inc = 1U; /* защита от зависания на нулевом приращении */
    }

    float shift = fmodf(phase_shift, 1.0f);
    /* NaN-безопасно: fmodf(NaN, ...) и fmodf(+-inf, ...) по IEEE 754 дают
     * NaN, который "shift < 0.0f" ниже пропустил бы НЕИЗМЕНЁННЫМ (сравнение
     * NaN с чем угодно ложно) - а дальше NaN неизбежно попал бы в
     * приведение float -> uint32_t (через double), что UB по C99. isnan()
     * ловит и NaN-от-входа, и NaN-от-fmodf(inf,...), заменяя на safe 0.0f. */
    if (isnan(shift))
    {
        shift = 0.0f;
    }
    else if (shift < 0.0f)
    {
        shift += 1.0f;
    }
    uint32_t phase_acc = (uint32_t)((double)shift * 4294967296.0);

    h->fade_active = 0U;
    h->cycle_id  = (uint8_t)cycle;
    h->phase_inc = phase_inc;
    h->phase_acc = phase_acc;
    h->mode      = (uint8_t)mode;

    LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_CYCLE_START, h->channel, (int32_t)cycle);

    return HAL_OK;
}

/* ------------------------------------------------------------------------ */
/*  Регистрация нагрузки                                                    */
/* ------------------------------------------------------------------------ */

LOAD_PWM_Handle_t *LOAD_PWM_Init(const LOAD_PWM_Config_t *config)
{
    if ((config == NULL) || (config->htim == NULL) || (config->tick_freq_hz == 0U))
    {
        LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_INIT_BAD_CONFIG, 0, 0);
        return NULL;
    }

    LOAD_PWM_Handle_t *h = load_pwm_find_or_alloc_slot(config->htim, config->channel);
    if (h == NULL)
    {
        LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_INIT_POOL_FULL, config->channel, LOAD_PWM_MAX_LOADS);
        return NULL; /* пул исчерпан */
    }

    /* Различаем "первая регистрация этого слота" от "повторный Init для
     * уже существующего физического (htim, channel)" - нужно, чтобы решить,
     * можно ли переиспользовать уже выделенный слот LUT (см. ниже). */
    uint8_t is_new_handle = (h->used == 0U);

    h->htim = config->htim;
    h->channel = (uint8_t)config->channel;
    h->tick_source_id = config->tick_source_id;
    h->tick_freq_hz_cached = config->tick_freq_hz;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(config->htim);
    float min_p = load_pwm_clamp_percent(config->brightness_min_percent);
    float max_p = load_pwm_clamp_percent(config->brightness_max_percent);
    if (max_p < min_p)
    {
        float tmp = max_p;
        max_p = min_p;
        min_p = tmp;
    }

    /* Тип скважности (LOAD_PWM_Duty_t) может быть уже uint8_t/uint16_t (см.
     * LOAD_PWM_RAM_QUALITY) - насыщаем расчётное значение сверху пределом
     * выбранного типа, если ARR канала больше, чем этот тип способен
     * выразить (сознательный компромисс ради экономии RAM). Сравнение с
     * пределом типа - в double, а не float: у float 24-битная мантисса не
     * может точно представить значения около UINT32_MAX (при
     * LOAD_PWM_RAM_QUALITY=HIGH), из-за чего сравнение "> duty_type_max"
     * могло не сработать вплотную к границе, и последующее приведение
     * float->uint32_t стало бы неопределённым поведением по C99. double
     * представляет весь диапазон uint32_t точно, стоит это только в Init(),
     * не в Tick(). */
    double pwm_min_d = ((double)min_p / 100.0) * (double)arr;
    double pwm_max_d = ((double)max_p / 100.0) * (double)arr;
    if (pwm_min_d > (double)LOAD_PWM_DUTY_MAX) { pwm_min_d = (double)LOAD_PWM_DUTY_MAX; }
    if (pwm_max_d > (double)LOAD_PWM_DUTY_MAX) { pwm_max_d = (double)LOAD_PWM_DUTY_MAX; }
    h->pwm_min = (LOAD_PWM_Duty_t)pwm_min_d;
    h->pwm_max = (LOAD_PWM_Duty_t)pwm_max_d;

    if (config->load_type == LOAD_PWM_TYPE_LED)
    {
        /* Новый слот LUT нужен, если хэндл только что создан, либо раньше
         * был LOAD_PWM_TYPE_LINEAR (у него не было LUT) - иначе спокойно
         * переиспользуем уже выделенный h->lut_index. */
        if (is_new_handle || (h->lut_index == LOAD_PWM_LUT_INDEX_NONE))
        {
#if LOAD_PWM_MAX_LEDS == 0U
            /* Проект без единого LED (LOAD_PWM_MAX_LEDS=0) - пул LUT
             * отсутствует по определению, LED-нагрузка зарегистрирована
             * быть не может. #if, а не рантайм-сравнение с константой 0 -
             * иначе "s_led_lut_used_count >= 0" (u8 >= 0) триггерит
             * -Wtype-limits как "сравнение всегда истинно". */
            LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_INIT_LED_POOL_FULL, config->channel, LOAD_PWM_MAX_LEDS);
            return NULL;
#else
            if (s_led_lut_used_count >= LOAD_PWM_MAX_LEDS)
            {
                LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_INIT_LED_POOL_FULL, config->channel, LOAD_PWM_MAX_LEDS);
                return NULL; /* пул LUT для LED исчерпан - см. LOAD_PWM_MAX_LEDS */
            }
#endif
            h->lut_index = s_led_lut_used_count;
            s_led_lut_used_count++;
        }

        /*
         * Гамма-коррекция: ищем степенную кривую u(x) = x^gamma, точно
         * проходящую через три опорные точки (0 -> pwm_min, 0.5 -> pwm_mid,
         * 1 -> pwm_max), из u(0.5) = 0.5^gamma = u_mid находим
         * gamma = log(u_mid) / log(0.5) - см. подробности в README.md.
         *
         * Упрощённая калибровка: если brightness_mid_percent < 0 (точка не
         * измерена) - берём типовое LOAD_PWM_DEFAULT_LED_GAMMA вместо
         * калибровки по факту.
         */
        float gamma;
        uint32_t pwm_min_u = (uint32_t)h->pwm_min;
        uint32_t pwm_max_u = (uint32_t)h->pwm_max;

        if (config->brightness_mid_percent < 0.0f)
        {
            gamma = LOAD_PWM_DEFAULT_LED_GAMMA;
        }
        else if (pwm_max_u > pwm_min_u)
        {
            float mid_p = load_pwm_clamp_percent(config->brightness_mid_percent);
            /* double по той же причине, что и pwm_min/pwm_max выше. */
            uint32_t pwm_mid_u = (uint32_t)(((double)mid_p / 100.0) * (double)arr);
            float u_mid = ((float)pwm_mid_u - (float)pwm_min_u) / ((float)pwm_max_u - (float)pwm_min_u);
            if (u_mid < 0.01f) { u_mid = 0.01f; }
            if (u_mid > 0.99f) { u_mid = 0.99f; }
            gamma = logf(u_mid) / logf(0.5f);
            if (gamma < 0.2f) { gamma = 0.2f; }
            if (gamma > 6.0f) { gamma = 6.0f; }
        }
        else
        {
            gamma = 1.0f;
        }

        /* Считается один раз здесь (не в Tick()!) и раскладывается в
         * таблицу отдельного пула - при вызовах Tick() остаётся только
         * быстрая линейная интерполяция, без вызова powf() каждый раз. */
        LOAD_PWM_Duty_t *lut = s_led_lut_pool[h->lut_index];
        for (uint32_t i = 0U; i < LOAD_PWM_GAMMA_LUT_POINTS; i++)
        {
            float frac = (float)i / (float)LOAD_PWM_GAMMA_LUT_SEGMENTS;
            float u = powf(frac, gamma);
            /* Насыщение в double по той же причине, что и pwm_min/pwm_max
             * выше - у float не хватает мантиссы вплотную к UINT32_MAX. */
            double duty_d = (double)pwm_min_u + (double)u * ((double)pwm_max_u - (double)pwm_min_u);
            if (duty_d > (double)LOAD_PWM_DUTY_MAX) { duty_d = (double)LOAD_PWM_DUTY_MAX; }
            if (duty_d < 0.0) { duty_d = 0.0; }
            lut[i] = (LOAD_PWM_Duty_t)(duty_d + 0.5);
        }
    }
    else
    {
        /* LOAD_PWM_TYPE_LINEAR - LUT не нужна вообще (см. load_pwm_visual_to_duty). */
        h->lut_index = LOAD_PWM_LUT_INDEX_NONE;
    }

    /* Запуск периферии ШИМ - идемпотентно безопасно вызывать даже если уже
     * запущена. Никакой "главный" таймер библиотека сама не трогает -
     * обеспечение периодичности вызова LOAD_PWM_Tick() полностью на
     * пользователе (см. stm32_load_pwm.h). Возврат проверяем ДО того, как
     * трогать рабочее (операционное) состояние хэндла - если это была
     * переинициализация УЖЕ активной нагрузки (is_new_handle == 0) и
     * HAL_TIM_PWM_Start вернул ошибку, нагрузка должна продолжить работать
     * ровно как работала (её текущий цикл/фон/фейд не должны молча
     * сброситься) - NULL означает "ничего не изменилось", а не "нагрузка
     * тихо остановлена". pwm_min/pwm_max/lut_index (калибровка) назад не
     * откатываются даже при ошибке - это не "работающее прямо сейчас"
     * состояние, а настройки для следующего успешного Init(). */
    HAL_StatusTypeDef pwm_start_status = HAL_TIM_PWM_Start(config->htim, config->channel);
    if (pwm_start_status != HAL_OK)
    {
        /* Для НОВОГО хэндла used ещё не выставлен (см. ниже) - слот и так
         * свободен, откатывать нечего. Слот LUT (если выделялся) не
         * освобождается индивидуально - см. комментарий у
         * s_led_lut_used_count. */
        LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_INIT_HAL_START_FAIL, config->channel, (int32_t)pwm_start_status);
        return NULL;
    }

    /* Цикл не запущен, фоновая яркость по умолчанию - 0%, фейда нет. Эта
     * "операционная" часть состояния намеренно выставляется ТОЛЬКО после
     * успешного старта PWM - см. комментарий выше. */
    h->cycle_id        = (uint8_t)LOAD_PWM_CYCLE_MEANDR;
    h->mode            = (uint8_t)LOAD_PWM_MODE_NONE;
    h->phase_acc       = 0U;
    h->phase_inc       = 0U;
    h->background_pwm  = h->pwm_min;
    h->current_pwm     = h->pwm_min;
    h->fade_active     = 0U;
    h->used            = 1U;

    load_pwm_write_output(h, h->background_pwm);

    LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_INIT_OK, config->channel, (int32_t)config->load_type);

    return h;
}

/* ------------------------------------------------------------------------ */
/*  Фоновая яркость                                                          */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef LOAD_PWM_SetBrightness(LOAD_PWM_Handle_t *h, float percent, uint32_t fade_ms)
{
    if (h == NULL)
    {
        LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_NULL_HANDLE, 0, 0);
        return HAL_ERROR;
    }

    float p = load_pwm_clamp_percent(percent);
    LOAD_PWM_Duty_t target_pwm;

    if (h->lut_index == LOAD_PWM_LUT_INDEX_NONE)
    {
        /* LOAD_PWM_TYPE_LINEAR - полное разрешение, см. комментарий у
         * load_pwm_percent_to_duty_linear. */
        target_pwm = load_pwm_percent_to_duty_linear(h, p);
    }
    else
    {
        /* LOAD_PWM_TYPE_LED - через 8-битное "визуальное" представление
         * (разрешение интерполяции гамма-LUT), округляем, а не усекаем. */
        uint16_t visual = (uint16_t)((p * 2.55f) + 0.5f); /* 0..100% -> 0..255 */
        if (visual > 255U)
        {
            visual = 255U;
        }
        target_pwm = load_pwm_visual_to_duty(h, visual);
    }

    h->background_pwm = target_pwm;

    if (h->mode != (uint8_t)LOAD_PWM_MODE_NONE)
    {
        /* Сейчас активен цикл - новое фоновое значение только запоминается
         * и будет применено МГНОВЕННО по остановке/завершению цикла. */
        h->fade_active = 0U;
        return HAL_OK;
    }

    if (fade_ms == 0U)
    {
        h->fade_active = 0U;
        h->current_pwm = target_pwm;
        load_pwm_write_output(h, target_pwm);
        return HAL_OK;
    }

    /* Плавный переход - ТОЛЬКО между фоновыми значениями, пока нагрузка
     * простаивает (реализация шага - в LOAD_PWM_Tick). Стартует от текущего
     * фактического значения (h->current_pwm), поэтому повторный вызов
     * посреди уже идущего фейда плавно переопределяет цель. */
    double ticks_d = ((double)fade_ms * (double)h->tick_freq_hz_cached) / 1000.0;
    /* Насыщение ДО приведения к uint32_t - без него очень длинный fade_ms в
     * сочетании с высокой tick_freq_hz мог бы дать ticks_d за пределами
     * диапазона uint32_t, а double->uint32_t с выходящим за диапазон
     * значением - неопределённое поведение по C99 (тот же класс проблемы,
     * что и с phase_inc в load_pwm_start_internal, там уже так насыщено). */
    if (ticks_d > 4294967295.0)
    {
        ticks_d = 4294967295.0;
    }
    uint32_t ticks = (uint32_t)(ticks_d + 0.5);
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    h->fade_step            = ((float)target_pwm - (float)h->current_pwm) / (float)ticks;
    h->fade_current_pwm     = (float)h->current_pwm;
    h->fade_ticks_remaining = ticks;
    h->fade_active          = 1U;

    return HAL_OK;
}

/* ------------------------------------------------------------------------ */
/*  Управление циклами индикации                                            */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef LOAD_PWM_Start(LOAD_PWM_Handle_t *h, LOAD_PWM_Cycle_t cycle,
                                  float phase_shift, uint32_t period_ms)
{
    return load_pwm_start_internal(h, cycle, phase_shift, period_ms, LOAD_PWM_MODE_REPEAT, 1U);
}

HAL_StatusTypeDef LOAD_PWM_StartOnce(LOAD_PWM_Handle_t *h, LOAD_PWM_Cycle_t cycle,
                                      float phase_shift, uint32_t period_ms)
{
    return load_pwm_start_internal(h, cycle, phase_shift, period_ms, LOAD_PWM_MODE_ONESHOT, 0U);
}

HAL_StatusTypeDef LOAD_PWM_Stop(LOAD_PWM_Handle_t *h)
{
    if (h == NULL)
    {
        LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_NULL_HANDLE, 0, 0);
        return HAL_ERROR;
    }

    h->mode        = (uint8_t)LOAD_PWM_MODE_NONE;
    h->fade_active = 0U;
    h->current_pwm = h->background_pwm;
    load_pwm_write_output(h, h->background_pwm);

    LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_STOP, h->channel, 0);

    return HAL_OK;
}

void LOAD_PWM_StopAll(void)
{
    uint32_t stopped_count = 0U;

    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        if (s_pool[i].used == 0U)
        {
            continue;
        }

        LOAD_PWM_Handle_t *h = &s_pool[i];
        h->mode        = (uint8_t)LOAD_PWM_MODE_NONE;
        h->fade_active = 0U;
        h->current_pwm = h->background_pwm;
        load_pwm_write_output(h, h->background_pwm);
        stopped_count++;
    }

    /* Один агрегированный лог на весь вызов, а не по одному на нагрузку -
     * StopAll() уже сам по себе редкий/аварийный вызов, но нагрузок может
     * быть LOAD_PWM_MAX_LOADS штук, лог по каждой был бы спамом. */
    LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_STOP_ALL, 0, (int32_t)stopped_count);
}

/* ------------------------------------------------------------------------ */
/*  Глобальный множитель яркости ("ночной режим")                           */
/* ------------------------------------------------------------------------ */

void LOAD_PWM_SetGlobalBrightness(float percent)
{
    float p = load_pwm_clamp_percent(percent);

    /* float/double здесь не проблема - функция вызывается редко (не из
     * Tick()), в отличие от load_pwm_write_output(). +0.5 - округление, не
     * усечение; при percent=100 даёт ровно LOAD_PWM_GLOBAL_MULT_FULL, что
     * включает быстрый путь без умножения в load_pwm_write_output(). */
    s_global_multiplier_q16 = (uint32_t)(((double)p / 100.0) * (double)LOAD_PWM_GLOBAL_MULT_FULL + 0.5);

    LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_GLOBAL_BRIGHTNESS, 0, (int32_t)(p + 0.5f));

    /* Активные циклы/фейды сами подхватят новый множитель на следующем
     * вызове LOAD_PWM_Tick() для них - а вот простаивающие (статичный фон)
     * нужно перерисовать сейчас же. */
    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        if (s_pool[i].used == 0U)
        {
            continue;
        }

        LOAD_PWM_Handle_t *h = &s_pool[i];
        if ((h->mode == (uint8_t)LOAD_PWM_MODE_NONE) && (h->fade_active == 0U))
        {
            load_pwm_write_output(h, h->current_pwm);
        }
    }
}

/* ------------------------------------------------------------------------ */
/*  Периодическое обновление                                                */
/* ------------------------------------------------------------------------ */

void LOAD_PWM_Tick(uint32_t tick_source_id)
{
    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        LOAD_PWM_Handle_t *h = &s_pool[i];

        if ((h->used == 0U) || (h->tick_source_id != tick_source_id))
        {
            continue;
        }

        if (h->mode != (uint8_t)LOAD_PWM_MODE_NONE)
        {
            /* Активен цикл индикации - продвигаем фазу. */
            uint32_t old_acc = h->phase_acc;
            uint32_t new_acc = old_acc + h->phase_inc;
            h->phase_acc = new_acc;

            if (new_acc < old_acc) /* переполнение (wrap) - период завершён */
            {
                if (h->mode == (uint8_t)LOAD_PWM_MODE_ONESHOT)
                {
                    h->mode = (uint8_t)LOAD_PWM_MODE_NONE;
                    h->current_pwm = h->background_pwm;
                    load_pwm_write_output(h, h->background_pwm);
                    /* Событие редкое (раз на завершившийся цикл, не на тик) -
                     * не спам, в отличие от лога на каждый LOAD_PWM_Tick(). */
                    LOAD_PWM_LOG(LOAD_PWM_LOG_CODE_CYCLE_DONE, h->channel, (int32_t)h->cycle_id);
                    continue; /* цикл завершён - яркость на этом такте больше не трогаем */
                }
                /* LOAD_PWM_MODE_REPEAT - остаток (new_acc) уже корректно
                 * "перенесён" на новый период, ничего специально сбрасывать
                 * не нужно. */
            }

            /* cycle_table намеренно НЕ кэшируется в хэндле - берём указатель
             * прямо из глобальной таблицы LOAD_PWM_Tables по cycle_id.
             * Стоимость идентична кэшированному указателю (одно обращение к
             * массиву что так, что так), но экономит 4 байта в каждом
             * хэндле - без потери скорости. */
            uint32_t idx = new_acc >> (32U - LOAD_PWM_TABLE_BITS);
            uint8_t visual = LOAD_PWM_Tables[h->cycle_id][idx];
            LOAD_PWM_Duty_t duty = load_pwm_visual_to_duty(h, (uint16_t)visual);

            h->current_pwm = duty;
            load_pwm_write_output(h, duty);
        }
        else if (h->fade_active != 0U)
        {
            /* Цикл не запущен, но идёт плавный переход фоновой яркости -
             * делаем один шаг фейда. */
            h->fade_ticks_remaining--;
            h->fade_current_pwm += h->fade_step;

            if (h->fade_ticks_remaining == 0U)
            {
                h->fade_active = 0U;
                h->current_pwm = h->background_pwm; /* точное целевое значение, без накопленной погрешности float */
            }
            else
            {
                /* Защита от накопленной погрешности float на длинных фейдах
                 * с обеих сторон диапазона - без неё fade_current_pwm
                 * теоретически может чуть выйти за [0, LOAD_PWM_DUTY_MAX]
                 * перед последним тиком, а приведение float-значения за
                 * пределами представимого диапазона беззнакового типа -
                 * неопределённое поведение по стандарту C.
                 *
                 * Верхняя граница сравнивается НЕ с (float)LOAD_PWM_DUTY_MAX
                 * напрямую - при LOAD_PWM_RAM_QUALITY=HIGH (uint32_t) само
                 * значение UINT32_MAX не представимо в float точно (не
                 * хватает мантиссы) и округлилось бы ВВЕРХ до 2^32, что
                 * сделало бы сравнение бесполезным. Сравниваем со следующей
                 * степенью двойки (LOAD_PWM_DUTY_MAX + 1) - она ВСЕГДА точно
                 * представима в float (256 / 65536 / 2^32) - и при
                 * попадании в "верхнюю" ветку берём точное целочисленное
                 * значение LOAD_PWM_DUTY_MAX напрямую, вообще не приводя
                 * float к целому типу в этом случае. */
                float clamped = h->fade_current_pwm;
                if (clamped < 0.0f)
                {
                    h->current_pwm = 0U;
                }
                else if (clamped >= (float)((double)LOAD_PWM_DUTY_MAX + 1.0))
                {
                    h->current_pwm = LOAD_PWM_DUTY_MAX;
                }
                else
                {
                    h->current_pwm = (LOAD_PWM_Duty_t)clamped;
                }
            }

            load_pwm_write_output(h, h->current_pwm);
        }
        else
        {
            /* Простаивает, фейда нет - трогать нечего, экономим время. */
        }
    }
}
