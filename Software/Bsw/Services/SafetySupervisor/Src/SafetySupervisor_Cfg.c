#include "SafetySupervisor_Cfg.h"

#define SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM 4095U
#define SAFETY_SUPERVISOR_CFG_READY_MAXIMUM_AGE_MS 1UL
#define SAFETY_SUPERVISOR_CFG_MEASUREMENT_APPROVED FALSE
#define SAFETY_SUPERVISOR_CFG_VBUS_RAW_MINIMUM 0U
#define SAFETY_SUPERVISOR_CFG_VBUS_RAW_MAXIMUM \
    SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM
#define SAFETY_SUPERVISOR_CFG_TEMPERATURE_RAW_MINIMUM 0U
#define SAFETY_SUPERVISOR_CFG_TEMPERATURE_RAW_MAXIMUM \
    SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM

#ifndef SAFETY_SUPERVISOR_CFG_TEST_HOOK
#define SAFETY_SUPERVISOR_CFG_TEST_HOOK(Config) ((void)(Config))
#endif

STATIC const SafetySupervisor_ConfigType SafetySupervisor_Config =
{
    SAFETY_SUPERVISOR_CFG_MEASUREMENT_APPROVED,
    SAFETY_SUPERVISOR_CFG_VBUS_RAW_MINIMUM,
    SAFETY_SUPERVISOR_CFG_VBUS_RAW_MAXIMUM,
    SAFETY_SUPERVISOR_CFG_TEMPERATURE_RAW_MINIMUM,
    SAFETY_SUPERVISOR_CFG_TEMPERATURE_RAW_MAXIMUM,
    { 1UL, 1UL, 0 },
    { 1UL, 1UL, 0 },
    SAFETY_SUPERVISOR_CFG_READY_MAXIMUM_AGE_MS,
    {
        SC1_MEASUREMENT_SCHEMA_VERSION,
        SC1_MEASUREMENT_BOARD_PROFILE_CODE,
        { 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL }
    }
};

_Static_assert(SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM == 0x0FFFU,
               "SC-1 safety configuration uses the 12-bit ADC envelope");
_Static_assert(SAFETY_SUPERVISOR_CFG_READY_MAXIMUM_AGE_MS == 1U,
               "SC-1 READY snapshot age must remain one millisecond");
_Static_assert(SAFETY_SUPERVISOR_CFG_MEASUREMENT_APPROVED == FALSE,
               "SC-1A measurement approval must remain false");
_Static_assert(SAFETY_SUPERVISOR_CFG_VBUS_RAW_MINIMUM <=
               SAFETY_SUPERVISOR_CFG_VBUS_RAW_MAXIMUM,
               "SC-1 VBUS raw bounds are impossible");
_Static_assert(SAFETY_SUPERVISOR_CFG_TEMPERATURE_RAW_MINIMUM <=
               SAFETY_SUPERVISOR_CFG_TEMPERATURE_RAW_MAXIMUM,
               "SC-1 temperature raw bounds are impossible");

STATIC boolean SafetySupervisor_CfgDigestIsZero(
    const Sc1_MeasurementIdentityType *identity)
{
    uint32 index;

    for (index = 0U;
         index < SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT;
         index++)
    {
        if (identity->evidenceSha256[index] != 0U)
        {
            return FALSE;
        }
    }
    return TRUE;
}

STATIC boolean SafetySupervisor_CfgIsValid(
    const SafetySupervisor_ConfigType *config)
{
    boolean digestIsZero;

    if (((config->measurementApproved != FALSE) &&
         (config->measurementApproved != TRUE)) ||
        (config->vbusRawMinimum > config->vbusRawMaximum) ||
        (config->vbusRawMaximum > SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM) ||
        (config->temperatureRawMinimum > config->temperatureRawMaximum) ||
        (config->temperatureRawMaximum >
         SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM) ||
        (config->vbusMillivolts.denominator == 0U) ||
        (config->temperatureCentiDegrees.denominator == 0U) ||
        (config->readySnapshotMaximumAgeMs !=
         SAFETY_SUPERVISOR_CFG_READY_MAXIMUM_AGE_MS) ||
        (config->identity.schemaVersion != SC1_MEASUREMENT_SCHEMA_VERSION) ||
        (config->identity.boardProfileCode !=
         SC1_MEASUREMENT_BOARD_PROFILE_CODE))
    {
        return FALSE;
    }

    digestIsZero = SafetySupervisor_CfgDigestIsZero(&config->identity);
    if (((config->measurementApproved == TRUE) &&
         ((digestIsZero == TRUE) ||
          (config->vbusRawMinimum == 0U) ||
          (config->vbusRawMaximum == SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM) ||
          (config->temperatureRawMinimum == 0U) ||
          (config->temperatureRawMaximum ==
           SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM))) ||
        ((config->measurementApproved == FALSE) &&
         ((digestIsZero != TRUE) ||
          (config->vbusRawMinimum != 0U) ||
          (config->vbusRawMaximum != SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM) ||
          (config->temperatureRawMinimum != 0U) ||
          (config->temperatureRawMaximum !=
           SAFETY_SUPERVISOR_CFG_ADC_MAXIMUM))))
    {
        return FALSE;
    }
    return TRUE;
}

Std_ReturnType SafetySupervisor_CfgGet(SafetySupervisor_ConfigType *Config)
{
    if (Config == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *Config = SafetySupervisor_Config;
    SAFETY_SUPERVISOR_CFG_TEST_HOOK(Config);
    if (SafetySupervisor_CfgIsValid(Config) != TRUE)
    {
        *Config = (SafetySupervisor_ConfigType){ 0 };
        return E_NOT_OK;
    }
    return E_OK;
}
