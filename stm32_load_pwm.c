/**
 ******************************************************************************
 * @file    stm32_load_pwm.c
 * @brief   Реализация библиотеки управления ШИМ-нагрузкой (см. stm32_load_pwm.h).
 * @author  Cload
 * @date    24.08.2026
 * @version 0.1
 *
 * @copyright Copyright (c) 2026 Cload.
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
 * Каждая таблица - LOAD_PWM_TABLE_SIZE (128) точек визуальной яркости
 * (0..255) на ПОЛНЫЙ период цикла. Значения посчитаны один раз офлайн (не в
 * прошивке) и зашиты как константы - в рантайме используется только
 * табличная выборка, без единой формулы (sin/pow и т.п.) в Tick().
 */

/** 1) Меандр: первая половина периода - 100%, вторая - 0%. */
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

/** 2) "Дыхание": плавное нарастание 0->100% и спад 100->0% на весь период. */
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

/** 3) Импульс с ожиданием: горб (~sin) в первой половине, пауза во второй. */
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

/** 4) Полутрапеция: 10% рост, 30% полка на 100%, 10% спад, 50% пауза. */
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

/** 5) Полупила: первая половина - линейный рост 0->100%, вторая - пауза. */
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

/** 6) Треугольник: непрерывный линейный рост 0->100% и сразу спад 100->0%
 *     без паузы (в отличие от полупилы). */
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

/** 7) Двойной проблеск ("маячок"): 12.5% периода включён на 100%, 25% -
 *     выключен, 25% - снова включён на 100%, 12.5% - выключен, оставшиеся
 *     25% - пауза. Резкие прямоугольные фронты (не плавный импульс). */
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

/** 8) Строб: пять коротких резких вспышек подряд (100%), затем долгая пауза. */
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
/*  Статический пул хэндлов (без malloc)                                    */
/* ------------------------------------------------------------------------ */

static LOAD_PWM_Handle_t s_pool[LOAD_PWM_MAX_LOADS];

/** Глобальный "ночной" множитель (0.0..1.0), см. LOAD_PWM_SetGlobalBrightness.
 *  Применяется только в момент финальной записи в CCR - внутреннее
 *  логическое состояние каждой нагрузки (background_pwm/current_pwm) его не
 *  учитывает, поэтому изменение множителя "на лету" не портит фейд/фазу. */
static float s_global_multiplier = 1.0f;

/* ------------------------------------------------------------------------ */
/*  Внутренние вспомогательные функции                                      */
/* ------------------------------------------------------------------------ */

/** Ограничивает процент в диапазон 0..100. */
static float LOAD_PWM_ClampPercent(float percent)
{
    if (percent < 0.0f)   { return 0.0f; }
    if (percent > 100.0f) { return 100.0f; }
    return percent;
}

/**
 * @brief  Переводит визуальную яркость (0..255) в значение ШИМ (тики CCR)
 *         через таблицу гамма-коррекции, линейно интерполируя между двумя
 *         ближайшими узловыми точками таблицы. Вызывается как из Tick()
 *         (продвижение цикла), так и вне её (SetBrightness) - быстрая (без
 *         деления - только сдвиг/маска/одно умножение). Для нагрузок с
 *         load_type == LOAD_PWM_TYPE_LINEAR таблица построена с gamma = 1,
 *         то есть фактически линейна - отдельной ветки кода не требуется.
 */
static uint32_t LOAD_PWM_VisualToDuty(const LOAD_PWM_Handle_t *h, uint16_t visual)
{
    if (visual >= 255U)
    {
        return h->pwm_max; /* точное верхнее значение, без интерполяционного зазора */
    }

    uint32_t seg  = (uint32_t)visual >> LOAD_PWM_GAMMA_LUT_INDEX_SHIFT;
    uint32_t frac = (uint32_t)visual & LOAD_PWM_GAMMA_LUT_SEGMENTS_MASK;

    uint32_t d0 = h->gamma_lut[seg];
    uint32_t d1 = h->gamma_lut[seg + 1U];

    return d0 + (((d1 - d0) * frac) >> LOAD_PWM_GAMMA_LUT_INDEX_SHIFT);
}

/**
 * @brief  Единая точка записи в CCR - применяет глобальный "ночной"
 *         множитель (LOAD_PWM_SetGlobalBrightness) к логическому (уже
 *         гамма-скорректированному) значению ПЕРЕД выводом в железо.
 *         Внутреннее состояние (background_pwm/current_pwm) всегда хранит
 *         логическое значение БЕЗ множителя - см. комментарий у
 *         s_global_multiplier.
 */
static void LOAD_PWM_WriteOutput(LOAD_PWM_Handle_t *h, uint32_t logical_duty)
{
    uint32_t out = (uint32_t)((float)logical_duty * s_global_multiplier);
    __HAL_TIM_SET_COMPARE(h->htim, h->channel, out);
}

/** Ищет свободный слот в пуле либо уже зарегистрированный (htim, channel) -
 *  для идемпотентности повторного LOAD_PWM_Init(). NULL, если пул полон и
 *  совпадения не найдено. */
static LOAD_PWM_Handle_t *LOAD_PWM_FindOrAllocSlot(TIM_HandleTypeDef *htim, uint32_t channel)
{
    uint32_t free_index = LOAD_PWM_MAX_LOADS;
    uint32_t has_free = 0U;

    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        if (s_pool[i].used != 0U)
        {
            if ((s_pool[i].htim == htim) && (s_pool[i].channel == channel))
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
static HAL_StatusTypeDef LOAD_PWM_StartInternal(LOAD_PWM_Handle_t *h, LOAD_PWM_Cycle_t cycle,
                                                 float phase_shift, uint32_t period_ms,
                                                 LOAD_PWM_Mode_t mode, uint8_t allow_continue)
{
    if ((h == NULL) || (cycle >= LOAD_PWM_CYCLE_COUNT) || (period_ms == 0U))
    {
        return HAL_ERROR;
    }

    /* Правило "не начинать заново, если тот же цикл уже зациклен" -
     * действует только для LOAD_PWM_Start (allow_continue == 1). */
    if ((allow_continue != 0U) && (h->mode == LOAD_PWM_MODE_REPEAT) && (h->cycle_id == cycle))
    {
        return HAL_OK;
    }

    /* Сколько вызовов LOAD_PWM_Tick() (для этой нагрузки) укладывается в
     * один период цикла, по заявленной tick_freq_hz. */
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
    if (shift < 0.0f)
    {
        shift += 1.0f;
    }
    uint32_t phase_acc = (uint32_t)((double)shift * 4294967296.0);

    /* Запуск/переключение цикла отменяет плавный фейд фоновой яркости, если
     * он был начат, пока нагрузка простаивала - переход цикл<->фон плавным
     * не делаем. */
    h->fade_active = 0U;

    /* Сначала готовим все "тихие" поля, и только в конце атомарно
     * переключаем mode - именно mode проверяется в LOAD_PWM_Tick(), поэтому
     * порядок записи важен. */
    h->cycle_table = LOAD_PWM_Tables[cycle];
    h->cycle_id    = cycle;
    h->phase_inc   = phase_inc;
    h->phase_acc   = phase_acc;
    h->mode        = mode;

    return HAL_OK;
}

/* ------------------------------------------------------------------------ */
/*  Регистрация нагрузки                                                    */
/* ------------------------------------------------------------------------ */

LOAD_PWM_Handle_t *LOAD_PWM_Init(const LOAD_PWM_Config_t *config)
{
    if ((config == NULL) || (config->htim == NULL) || (config->tick_freq_hz == 0U))
    {
        return NULL;
    }

    LOAD_PWM_Handle_t *h = LOAD_PWM_FindOrAllocSlot(config->htim, config->channel);
    if (h == NULL)
    {
        return NULL; /* пул исчерпан */
    }

    h->htim           = config->htim;
    h->channel        = config->channel;
    h->tick_source_id = config->tick_source_id;
    h->tick_freq_hz_cached = config->tick_freq_hz;

    /* Пересчёт границ диапазона из процентов в тики ШИМ по текущему ARR
     * канала. */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(config->htim);
    float min_p = LOAD_PWM_ClampPercent(config->brightness_min_percent);
    float max_p = LOAD_PWM_ClampPercent(config->brightness_max_percent);

    /* Защита от перепутанных местами min/max. */
    if (max_p < min_p)
    {
        float tmp = max_p;
        max_p = min_p;
        min_p = tmp;
    }

    h->pwm_min = (uint32_t)((min_p / 100.0f) * (float)arr);
    h->pwm_max = (uint32_t)((max_p / 100.0f) * (float)arr);

    float gamma = 1.0f;

    if (config->load_type == LOAD_PWM_TYPE_LINEAR)
    {
        /* Линейная нагрузка (лампа накаливания, кулер, DC-мотор,
         * электромагнит) - без гамма-коррекции, скважность ШИМ линейна
         * относительно заданного значения. brightness_mid_percent для
         * этого типа не используется. pwm_mid всё же считается для
         * справки/отладки (середина диапазона), в расчёт кривой не входит. */
        gamma = 1.0f;
        h->pwm_mid = (h->pwm_min + h->pwm_max) / 2U;
    }
    else /* LOAD_PWM_TYPE_LED */
    {
        if (config->brightness_mid_percent < 0.0f)
        {
            /*
             * Маркер "точка 50% не измерена" - упрощённая калибровка:
             * используем типовое значение gamma для светодиодов
             * (LOAD_PWM_DEFAULT_LED_GAMMA) вместо калибровки по факту. Это
             * НЕ дополнительная (четвёртая) точка калибровки, а наоборот -
             * упрощение до двух точек (min/max) для тех, кому не хочется
             * измерять/подбирать глазами среднюю точку, ценой чуть меньшей
             * точности кривой под конкретный экземпляр светодиода.
             */
            gamma = LOAD_PWM_DEFAULT_LED_GAMMA;
            h->pwm_mid = h->pwm_min
                + (uint32_t)(powf(0.5f, gamma) * (float)(h->pwm_max - h->pwm_min));
        }
        else
        {
            /*
             * Гамма-коррекция, калиброванная по измеренной точке: ищем
             * степенную кривую u(x) = x^gamma, ТОЧНО проходящую через все
             * три заданные пользователем опорные точки (0 -> pwm_min,
             * 0.5 -> pwm_mid, 1 -> pwm_max). Из u(0.5) = 0.5^gamma = u_mid
             * находим gamma = log(u_mid) / log(0.5).
             */
            float mid_p = LOAD_PWM_ClampPercent(config->brightness_mid_percent);
            h->pwm_mid = (uint32_t)((mid_p / 100.0f) * (float)arr);

            if (h->pwm_max > h->pwm_min)
            {
                float u_mid = ((float)h->pwm_mid - (float)h->pwm_min) / ((float)h->pwm_max - (float)h->pwm_min);
                if (u_mid < 0.01f) { u_mid = 0.01f; }
                if (u_mid > 0.99f) { u_mid = 0.99f; }
                gamma = logf(u_mid) / logf(0.5f);
                if (gamma < 0.2f) { gamma = 0.2f; }  /* защита от вырожденно резкой кривой */
                if (gamma > 6.0f) { gamma = 6.0f; }
            }
        }
    }

    /* Считается один раз здесь (не в Tick()!) и раскладывается в таблицу
     * (LOAD_PWM_GAMMA_LUT_POINTS точек) - при вызовах Tick() остаётся
     * только быстрая линейная интерполяция, без вызова powf() каждый раз. */
    for (uint32_t i = 0U; i < LOAD_PWM_GAMMA_LUT_POINTS; i++)
    {
        float frac = (float)i / (float)LOAD_PWM_GAMMA_LUT_SEGMENTS;
        float u = powf(frac, gamma);
        float duty_f = (float)h->pwm_min + u * ((float)h->pwm_max - (float)h->pwm_min);
        h->gamma_lut[i] = (uint32_t)(duty_f + 0.5f);
    }

    /* Цикл не запущен, фоновая яркость по умолчанию - 0%, фейда нет. */
    h->cycle_table     = NULL;
    h->cycle_id        = LOAD_PWM_CYCLE_MEANDR;
    h->mode            = LOAD_PWM_MODE_NONE;
    h->phase_acc       = 0U;
    h->phase_inc       = 0U;
    h->background_pwm  = h->pwm_min;
    h->current_pwm     = h->pwm_min;
    h->fade_active     = 0U;

    h->used = 1U;

    /* Запуск периферии ШИМ - идемпотентно безопасно вызывать даже если уже
     * запущена. Никакой "главный" таймер библиотека сама не трогает -
     * обеспечение периодичности вызова LOAD_PWM_Tick() полностью на
     * пользователе (см. stm32_load_pwm.h). */
    HAL_TIM_PWM_Start(config->htim, config->channel);
    LOAD_PWM_WriteOutput(h, h->background_pwm);

    return h;
}

/* ------------------------------------------------------------------------ */
/*  Фоновая яркость                                                          */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef LOAD_PWM_SetBrightness(LOAD_PWM_Handle_t *h, float percent, uint32_t fade_ms)
{
    if (h == NULL)
    {
        return HAL_ERROR;
    }

    float p = LOAD_PWM_ClampPercent(percent);
    uint16_t visual = (uint16_t)(p * 2.55f); /* 0..100% -> 0..255 */
    if (visual > 255U)
    {
        visual = 255U;
    }

    uint32_t target_pwm = LOAD_PWM_VisualToDuty(h, visual);
    h->background_pwm = target_pwm;

    if (h->mode != LOAD_PWM_MODE_NONE)
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
        LOAD_PWM_WriteOutput(h, target_pwm);
        return HAL_OK;
    }

    /* Плавный переход - ТОЛЬКО между фоновыми значениями, пока нагрузка
     * простаивает (реализация шага - в LOAD_PWM_Tick). Стартует от текущего
     * фактического значения (h->current_pwm), поэтому повторный вызов
     * посреди уже идущего фейда плавно переопределяет цель. */
    double ticks_d = ((double)fade_ms * (double)h->tick_freq_hz_cached) / 1000.0;
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
    return LOAD_PWM_StartInternal(h, cycle, phase_shift, period_ms, LOAD_PWM_MODE_REPEAT, 1U);
}

HAL_StatusTypeDef LOAD_PWM_StartOnce(LOAD_PWM_Handle_t *h, LOAD_PWM_Cycle_t cycle,
                                      float phase_shift, uint32_t period_ms)
{
    return LOAD_PWM_StartInternal(h, cycle, phase_shift, period_ms, LOAD_PWM_MODE_ONESHOT, 0U);
}

HAL_StatusTypeDef LOAD_PWM_Stop(LOAD_PWM_Handle_t *h)
{
    if (h == NULL)
    {
        return HAL_ERROR;
    }

    h->mode        = LOAD_PWM_MODE_NONE;
    h->fade_active = 0U;
    h->current_pwm = h->background_pwm;
    LOAD_PWM_WriteOutput(h, h->background_pwm);

    return HAL_OK;
}

void LOAD_PWM_StopAll(void)
{
    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        if (s_pool[i].used == 0U)
        {
            continue;
        }

        LOAD_PWM_Handle_t *h = &s_pool[i];
        h->mode        = LOAD_PWM_MODE_NONE;
        h->fade_active = 0U;
        h->current_pwm = h->background_pwm;
        LOAD_PWM_WriteOutput(h, h->background_pwm);
    }
}

/* ------------------------------------------------------------------------ */
/*  Глобальный множитель яркости ("ночной режим")                           */
/* ------------------------------------------------------------------------ */

void LOAD_PWM_SetGlobalBrightness(float percent)
{
    float p = LOAD_PWM_ClampPercent(percent);
    s_global_multiplier = p / 100.0f;

    /* Активные циклы/фейды сами подхватят новый множитель на следующем
     * вызове LOAD_PWM_Tick() для них - а вот простаивающие (статичный фон)
     * нужно перерисовать сейчас же, иначе "ночной режим" не применится, пока
     * не случится следующее событие для этой нагрузки. */
    for (uint32_t i = 0U; i < LOAD_PWM_MAX_LOADS; i++)
    {
        if (s_pool[i].used == 0U)
        {
            continue;
        }

        LOAD_PWM_Handle_t *h = &s_pool[i];
        if ((h->mode == LOAD_PWM_MODE_NONE) && (h->fade_active == 0U))
        {
            LOAD_PWM_WriteOutput(h, h->current_pwm);
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

        if (h->mode != LOAD_PWM_MODE_NONE)
        {
            /* Активен цикл индикации - продвигаем фазу. */
            uint32_t old_acc = h->phase_acc;
            uint32_t new_acc = old_acc + h->phase_inc;
            h->phase_acc = new_acc;

            if (new_acc < old_acc) /* переполнение (wrap) - период завершён */
            {
                if (h->mode == LOAD_PWM_MODE_ONESHOT)
                {
                    h->mode = LOAD_PWM_MODE_NONE;
                    h->current_pwm = h->background_pwm;
                    LOAD_PWM_WriteOutput(h, h->background_pwm);
                    continue; /* цикл завершён - яркость на этом такте больше не трогаем */
                }
                /* LOAD_PWM_MODE_REPEAT - остаток (new_acc) уже корректно
                 * "перенесён" на новый период, ничего специально сбрасывать
                 * не нужно. */
            }

            uint32_t idx = new_acc >> (32U - LOAD_PWM_TABLE_BITS);
            uint8_t visual = h->cycle_table[idx];
            uint32_t duty = LOAD_PWM_VisualToDuty(h, (uint16_t)visual);

            h->current_pwm = duty;
            LOAD_PWM_WriteOutput(h, duty);
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
                h->current_pwm = (uint32_t)h->fade_current_pwm;
            }

            LOAD_PWM_WriteOutput(h, h->current_pwm);
        }
        else
        {
            /* Простаивает, фейда нет - трогать нечего, экономим время. */
        }
    }
}
