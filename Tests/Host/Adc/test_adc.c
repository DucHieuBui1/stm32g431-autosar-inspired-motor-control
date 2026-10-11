#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "stm32g431xx.h"

ADC_TypeDef AdcHost_Adc1;
ADC_TypeDef AdcHost_Adc2;
ADC_Common_TypeDef AdcHost_Adc12Common;
GPIO_TypeDef AdcHost_GpioA;
GPIO_TypeDef AdcHost_GpioC;
RCC_TypeDef AdcHost_Rcc;
uint32_t SystemCoreClock;

#include "../../../Software/Bsw/Mcal/Adc/Src/Adc.c"
#include "../../../Software/Bsw/Mcal/Mcu/Src/Mcu_Adc12.c"

#define TEST_NEVER_OBSERVATION UINT_MAX

static uint32_t Test_CalibrationClearObservation;
static uint32_t Test_ReadySetObservation;
static uint32_t Test_CorruptAdcEvent;
static uint32_t Test_McuCorruptStage;
static uint32_t Test_McuCorruptAction;
static uint32_t Test_StatusClearCount;
static uint32_t Test_InjectedReadCount[4];

#define TEST_CHECK(condition)                                             \
    do                                                                    \
    {                                                                     \
        if (!(condition))                                                 \
        {                                                                 \
            (void)printf("FAIL line %d: %s\n", __LINE__, #condition);     \
            return 1;                                                     \
        }                                                                 \
    } while (0)

void AdcHost_EventHook(uint32_t event)
{
    if (event != Test_CorruptAdcEvent)
    {
        return;
    }

    if (event == ADC_TEST_EVENT_GPIO_CONFIGURED)
    {
        GPIOA->MODER &= ~ADC_GPIO_PA0_MODE_MASK;
    }
    else if (event == ADC_TEST_EVENT_COMMON_CONFIGURED)
    {
        ADC12_COMMON->CCR |= ADC_CCR_DUAL;
    }
    else if (event == ADC_TEST_EVENT_FINAL_READBACK)
    {
        ADC2->CFGR |= ADC_CFGR_JQM;
    }
    else
    {
        /* No action for an unknown test event. */
    }
}

void AdcHost_WaitHook(uint32_t stage, uint32_t observation)
{
    if ((stage == ADC_TEST_WAIT_CALIBRATION) &&
        (observation == Test_CalibrationClearObservation))
    {
        ADC2->CR &= ~ADC_CR_ADCAL;
    }

    if ((stage == ADC_TEST_WAIT_READY) &&
        (observation == Test_ReadySetObservation))
    {
        ADC2->ISR |= ADC_ISR_ADRDY;
    }
}

void AdcHost_StatusClearHook(uint32_t mask)
{
    Test_StatusClearCount++;
    ADC2->ISR &= ~mask;
}

uint32_t AdcHost_ReadInjectedData(volatile uint32_t *registerAddress,
                                  uint32_t rank)
{
    if (rank < 4U)
    {
        Test_InjectedReadCount[rank]++;
    }
    return *registerAddress;
}

void McuAdc12Host_Hook(uint32_t stage)
{
    if (stage != Test_McuCorruptStage)
    {
        return;
    }

    if (Test_McuCorruptAction == 1U)
    {
        RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
    }
    else if (Test_McuCorruptAction == 2U)
    {
        RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADC12RST;
    }
    else if (Test_McuCorruptAction == 3U)
    {
        RCC->AHB2RSTR |= RCC_AHB2RSTR_ADC12RST;
    }
    else if (Test_McuCorruptAction == 4U)
    {
        ADC2->CFGR = 0U;
    }
    else if (Test_McuCorruptAction == 5U)
    {
        ADC1->CFGR = ADC_CFGR_JQDIS | 1U;
    }
    else
    {
        /* No action for an unknown scripted corruption. */
    }
}

static void Test_ResetFixture(void)
{
    (void)memset(&AdcHost_Adc1, 0, sizeof(AdcHost_Adc1));
    (void)memset(&AdcHost_Adc2, 0, sizeof(AdcHost_Adc2));
    (void)memset(&AdcHost_Adc12Common, 0, sizeof(AdcHost_Adc12Common));
    (void)memset(&AdcHost_GpioA, 0, sizeof(AdcHost_GpioA));
    (void)memset(&AdcHost_GpioC, 0, sizeof(AdcHost_GpioC));
    (void)memset(&AdcHost_Rcc, 0, sizeof(AdcHost_Rcc));

    SystemCoreClock = 170000000U;
    ADC1->CR = ADC_CR_DEEPPWD;
    ADC2->CR = ADC_CR_DEEPPWD;
    ADC1->CFGR = ADC_CFGR_JQDIS;
    ADC2->CFGR = ADC_CFGR_JQDIS;
    RCC->CCIPR = RCC_CCIPR_ADC12SEL_0;

    Adc_State = ADC_INTERNAL_UNINITIALIZED;
    Adc_CommonFrozen = FALSE;
    (void)memset(&Adc_CachedResult, 0, sizeof(Adc_CachedResult));
    Adc_AcquisitionSequence = 0U;
    Mcu_Adc12ClockInitAttempted = FALSE;

    Test_CalibrationClearObservation = 0U;
    Test_ReadySetObservation = 0U;
    Test_CorruptAdcEvent = 0U;
    Test_McuCorruptStage = 0U;
    Test_McuCorruptAction = 0U;
    Test_StatusClearCount = 0U;
    (void)memset(Test_InjectedReadCount, 0,
                 sizeof(Test_InjectedReadCount));
}

static void Test_EnableAdcClocks(void)
{
    RCC->AHB2ENR = (RCC_AHB2ENR_GPIOAEN |
                    RCC_AHB2ENR_GPIOCEN |
                    RCC_AHB2ENR_ADC12EN);
}

static int Test_InitSuccessfully(void)
{
    Test_EnableAdcClocks();
    TEST_CHECK(Adc_Init() == E_OK);
    TEST_CHECK(Adc_CheckCommonConfiguration() == E_OK);
    return 0;
}

static int Test_McuClockResetSuccessAndOneShot(void)
{
    RCC_TypeDef rccSnapshot;
    ADC_TypeDef adc1Snapshot;
    ADC_TypeDef adc2Snapshot;

    Test_ResetFixture();
    RCC->AHB2ENR = (1UL << 20U);
    RCC->AHB2RSTR = (1UL << 20U);

    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_OK);
    TEST_CHECK((RCC->AHB2ENR & ADC_REQUIRED_RCC_GATES) ==
               ADC_REQUIRED_RCC_GATES);
    TEST_CHECK((RCC->AHB2ENR & (1UL << 20U)) != 0U);
    TEST_CHECK(RCC->AHB2RSTR == (1UL << 20U));
    TEST_CHECK(ADC1->CFGR == ADC_CFGR_JQDIS);
    TEST_CHECK(ADC2->CFGR == ADC_CFGR_JQDIS);

    rccSnapshot = *RCC;
    adc1Snapshot = *ADC1;
    adc2Snapshot = *ADC2;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);
    TEST_CHECK(memcmp(&rccSnapshot, RCC, sizeof(rccSnapshot)) == 0);
    TEST_CHECK(memcmp(&adc1Snapshot, ADC1, sizeof(adc1Snapshot)) == 0);
    TEST_CHECK(memcmp(&adc2Snapshot, ADC2, sizeof(adc2Snapshot)) == 0);
    TEST_CHECK(Adc_Init() == E_OK);
    TEST_CHECK(Adc_CheckCommonConfiguration() == E_OK);
    return 0;
}

static int Test_McuClockResetFailures(void)
{
    uint32_t stage;

    for (stage = 1U; stage <= 3U; stage++)
    {
        Test_ResetFixture();
        Test_McuCorruptStage = stage;
        Test_McuCorruptAction = stage;
        TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);
    }

    Test_ResetFixture();
    Test_McuCorruptStage = 3U;
    Test_McuCorruptAction = 4U;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);

    Test_ResetFixture();
    Test_McuCorruptStage = 3U;
    Test_McuCorruptAction = 5U;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);

    Test_ResetFixture();
    SystemCoreClock = 169999999U;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);

    Test_ResetFixture();
    RCC->CCIPR = 0U;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);

    Test_ResetFixture();
    RCC->AHB2RSTR = RCC_AHB2RSTR_GPIOARST;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);

    Test_ResetFixture();
    ADC1->CR |= ADC_CR_ADEN;
    TEST_CHECK(Mcu_Adc12PeripheralClockInit() == E_NOT_OK);
    return 0;
}

static int Test_AdcInitConfigurationAndOneShot(void)
{
    ADC_TypeDef adcSnapshot;
    ADC_Common_TypeDef commonSnapshot;
    GPIO_TypeDef gpioASnapshot;
    GPIO_TypeDef gpioCSnapshot;

    Test_ResetFixture();
    Test_EnableAdcClocks();
    GPIOA->MODER = 0xAAAAAAAAU;
    GPIOA->PUPDR = 0xFFFFFFFFU;
    GPIOC->MODER = 0x55555555U;
    GPIOC->PUPDR = 0xFFFFFFFFU;
    ADC12_COMMON->CCR = 0x80000000U;
    ADC2->SMPR1 = 0x40000000U;
    ADC2->DIFSEL = 0x00000122U;

    TEST_CHECK(Adc_Init() == E_OK);
    TEST_CHECK((GPIOA->MODER & ADC_GPIO_PA0_MODE_MASK) ==
               ADC_GPIO_ANALOG_MODE_PA0);
    TEST_CHECK((GPIOA->PUPDR & ADC_GPIO_PA0_MODE_MASK) == 0U);
    TEST_CHECK((GPIOC->MODER & ADC_GPIO_PC4_MODE_MASK) ==
               ADC_GPIO_ANALOG_MODE_PC4);
    TEST_CHECK((GPIOC->PUPDR & ADC_GPIO_PC4_MODE_MASK) == 0U);
    TEST_CHECK((ADC12_COMMON->CCR & ADC_COMMON_OWNED_MASK) == 0U);
    TEST_CHECK((ADC12_COMMON->CCR & 0x80000000U) != 0U);
    TEST_CHECK(ADC2->IER == 0U);
    TEST_CHECK(ADC2->CFGR == ADC_CFGR_JQDIS);
    TEST_CHECK((ADC2->CFGR & ADC_CFGR_JQM) == 0U);
    TEST_CHECK(ADC2->CFGR2 == 0U);
    TEST_CHECK((ADC2->SMPR1 & ADC_SMPR1_SAFETY_MASK) ==
               ADC_SMPR1_SAFETY_VALUE);
    TEST_CHECK((ADC2->SMPR1 & 0x40000000U) != 0U);
    TEST_CHECK((ADC2->JSQR & ADC_JSQR_SAFETY_MASK) ==
               ADC_JSQR_SAFETY_VALUE);
    TEST_CHECK((ADC2->DIFSEL & ADC_SAFETY_CHANNEL_DIFSEL_MASK) == 0U);
    TEST_CHECK((ADC2->DIFSEL & 0x00000100U) != 0U);
    TEST_CHECK((ADC2->CR & ADC_CR_ADEN) != 0U);
    TEST_CHECK((ADC2->CR & ADC_CR_DEEPPWD) == 0U);
    TEST_CHECK((ADC2->CR & ADC_CR_ADVREGEN) != 0U);
    TEST_CHECK((ADC2->ISR & ADC_ISR_ADRDY) == 0U);
    TEST_CHECK((ADC2->CR & ADC_CR_JADSTART) == 0U);

    adcSnapshot = *ADC2;
    commonSnapshot = *ADC12_COMMON;
    gpioASnapshot = *GPIOA;
    gpioCSnapshot = *GPIOC;
    TEST_CHECK(Adc_Init() == E_NOT_OK);
    TEST_CHECK(memcmp(&adcSnapshot, ADC2, sizeof(adcSnapshot)) == 0);
    TEST_CHECK(memcmp(&commonSnapshot, ADC12_COMMON,
                      sizeof(commonSnapshot)) == 0);
    TEST_CHECK(memcmp(&gpioASnapshot, GPIOA, sizeof(gpioASnapshot)) == 0);
    TEST_CHECK(memcmp(&gpioCSnapshot, GPIOC, sizeof(gpioCSnapshot)) == 0);
    return 0;
}

static int Test_AdcInitReadbackFailures(void)
{
    uint32_t event;

    Test_ResetFixture();
    TEST_CHECK(Adc_Init() == E_NOT_OK);
    TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);

    for (event = ADC_TEST_EVENT_GPIO_CONFIGURED;
         event <= ADC_TEST_EVENT_FINAL_READBACK;
         event++)
    {
        Test_ResetFixture();
        Test_EnableAdcClocks();
        Test_CorruptAdcEvent = event;
        TEST_CHECK(Adc_Init() == E_NOT_OK);
        TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);
        TEST_CHECK(Adc_Init() == E_NOT_OK);
    }
    return 0;
}

static int Test_BoundedWaits(void)
{
    Test_ResetFixture();
    Test_EnableAdcClocks();
    Test_CalibrationClearObservation =
        ADC_CALIBRATION_TIMEOUT_OBSERVATIONS - 1U;
    Test_ReadySetObservation = ADC_READY_TIMEOUT_OBSERVATIONS - 1U;
    TEST_CHECK(Adc_Init() == E_OK);

    Test_ResetFixture();
    Test_EnableAdcClocks();
    Test_CalibrationClearObservation = ADC_CALIBRATION_TIMEOUT_OBSERVATIONS;
    TEST_CHECK(Adc_Init() == E_NOT_OK);
    TEST_CHECK((ADC2->CR & ADC_CR_ADEN) == 0U);

    Test_ResetFixture();
    Test_EnableAdcClocks();
    Test_ReadySetObservation = ADC_READY_TIMEOUT_OBSERVATIONS;
    TEST_CHECK(Adc_Init() == E_NOT_OK);
    TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);
    return 0;
}

static int Test_RuntimeHappyPathAndBoundaries(void)
{
    Adc_SafetyGroupResultType result;
    Adc_SafetyGroupResultType repeated;
    Adc_SafetyGroupResultType sentinel;
    uint32_t frozenCommon;
    uint32_t frozenJsqr;

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    frozenCommon = ADC12_COMMON->CCR;
    frozenJsqr = ADC2->JSQR;

    result.vbusRaw = 123U;
    result.temperatureRaw = 456U;
    result.acquisitionSequence = 789U;
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_IDLE);
    TEST_CHECK(result.vbusRaw == 123U);
    TEST_CHECK(result.temperatureRaw == 456U);
    TEST_CHECK(result.acquisitionSequence == 789U);

    TEST_CHECK(Adc_StartSafetyGroup() == E_OK);
    TEST_CHECK((ADC2->CR & ADC_CR_JADSTART) != 0U);
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);
    TEST_CHECK(ADC2->JSQR == frozenJsqr);

    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_PENDING);
    sentinel = result;
    TEST_CHECK(memcmp(&result, &sentinel, sizeof(result)) == 0);

    ADC2->JDR1 = 0x0AAAU;
    ADC2->JDR2 = 0U;
    ADC2->JDR3 = 4095U;
    ADC2->CR &= ~ADC_CR_JADSTART;
    ADC2->ISR |= ADC_ISR_JEOS;
    TEST_CHECK(Adc_PollSafetyGroup(NULL_PTR) ==
               ADC_SAFETY_GROUP_INVALID_ARGUMENT);
    TEST_CHECK(memcmp(&result, &sentinel, sizeof(result)) == 0);
    TEST_CHECK(Test_InjectedReadCount[1] == 0U);
    TEST_CHECK(Test_InjectedReadCount[2] == 0U);
    TEST_CHECK(Test_InjectedReadCount[3] == 0U);
    TEST_CHECK((ADC2->ISR & ADC_ISR_JEOS) != 0U);
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_COMPLETED);
    TEST_CHECK(result.vbusRaw == 0U);
    TEST_CHECK(result.temperatureRaw == 4095U);
    TEST_CHECK(result.acquisitionSequence == 1U);
    TEST_CHECK(Test_InjectedReadCount[1] == 1U);
    TEST_CHECK(Test_InjectedReadCount[2] == 1U);
    TEST_CHECK(Test_InjectedReadCount[3] == 1U);
    TEST_CHECK((ADC2->ISR & ADC_ISR_JEOS) == 0U);

    repeated.vbusRaw = 1U;
    repeated.temperatureRaw = 2U;
    repeated.acquisitionSequence = 3U;
    TEST_CHECK(Adc_PollSafetyGroup(&repeated) ==
               ADC_SAFETY_GROUP_COMPLETED);
    TEST_CHECK(memcmp(&result, &repeated, sizeof(result)) == 0);
    TEST_CHECK(Test_InjectedReadCount[1] == 1U);

    TEST_CHECK(Adc_StartSafetyGroup() == E_OK);
    ADC2->JDR1 = 4095U;
    ADC2->JDR2 = 4095U;
    ADC2->JDR3 = 0U;
    ADC2->CR &= ~ADC_CR_JADSTART;
    ADC2->ISR |= ADC_ISR_JEOS;
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_COMPLETED);
    TEST_CHECK(result.vbusRaw == 4095U);
    TEST_CHECK(result.temperatureRaw == 0U);
    TEST_CHECK(result.acquisitionSequence == 2U);
    TEST_CHECK(ADC12_COMMON->CCR == frozenCommon);
    TEST_CHECK(ADC2->JSQR == frozenJsqr);
    return 0;
}

static int Test_RuntimeFaultsPreserveCallerData(void)
{
    Adc_SafetyGroupResultType result;
    Adc_SafetyGroupResultType sentinel;

    Test_ResetFixture();
    result.vbusRaw = 11U;
    result.temperatureRaw = 22U;
    result.acquisitionSequence = 33U;
    sentinel = result;
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_FAILED);
    TEST_CHECK(memcmp(&result, &sentinel, sizeof(result)) == 0);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    ADC2->ISR |= ADC_ISR_JEOS;
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);
    TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);
    TEST_CHECK((ADC2->ISR & ADC_ISR_JEOS) != 0U);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    TEST_CHECK(Adc_StartSafetyGroup() == E_OK);
    ADC2->JDR1 = 1U;
    ADC2->JDR2 = 2U;
    ADC2->JDR3 = 3U;
    ADC2->ISR |= (ADC_ISR_JQOVF | ADC_ISR_JEOS);
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_FAILED);
    TEST_CHECK(memcmp(&result, &sentinel, sizeof(result)) == 0);
    TEST_CHECK(Test_InjectedReadCount[1] == 0U);
    TEST_CHECK((ADC2->ISR & ADC_ISR_JQOVF) != 0U);
    TEST_CHECK((ADC2->ISR & ADC_ISR_JEOS) != 0U);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    ADC2->CR &= ~ADC_CR_ADEN;
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);
    TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    ADC12_COMMON->CCR |= ADC_CCR_DUAL;
    TEST_CHECK(Adc_CheckCommonConfiguration() == E_NOT_OK);
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);
    TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    GPIOC->MODER &= ~ADC_GPIO_PC4_MODE_MASK;
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    RCC->AHB2ENR &= ~RCC_AHB2ENR_ADC12EN;
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    ADC2->CFGR &= ~ADC_CFGR_JQDIS;
    TEST_CHECK(Adc_StartSafetyGroup() == E_NOT_OK);
    return 0;
}

static int Test_SequenceWrapAndCorruptState(void)
{
    Adc_SafetyGroupResultType result;
    Adc_SafetyGroupResultType sentinel;

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    Adc_AcquisitionSequence = UINT_MAX;
    TEST_CHECK(Adc_StartSafetyGroup() == E_OK);
    ADC2->CR &= ~ADC_CR_JADSTART;
    ADC2->ISR |= ADC_ISR_JEOS;
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_COMPLETED);
    TEST_CHECK(result.acquisitionSequence == 0U);

    Test_ResetFixture();
    TEST_CHECK(Test_InitSuccessfully() == 0);
    sentinel.vbusRaw = 101U;
    sentinel.temperatureRaw = 202U;
    sentinel.acquisitionSequence = 303U;
    result = sentinel;
    Adc_State = (Adc_InternalStateType)99;
    TEST_CHECK(Adc_PollSafetyGroup(&result) == ADC_SAFETY_GROUP_FAILED);
    TEST_CHECK(Adc_State == ADC_INTERNAL_FAILED);
    TEST_CHECK(memcmp(&result, &sentinel, sizeof(result)) == 0);
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_McuClockResetSuccessAndOneShot();
    failures += Test_McuClockResetFailures();
    failures += Test_AdcInitConfigurationAndOneShot();
    failures += Test_AdcInitReadbackFailures();
    failures += Test_BoundedWaits();
    failures += Test_RuntimeHappyPathAndBoundaries();
    failures += Test_RuntimeFaultsPreserveCallerData();
    failures += Test_SequenceWrapAndCorruptState();

    if (failures == 0)
    {
        (void)printf("PASS: ADC/Mcu host matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d ADC/Mcu host test group(s)\n", failures);
    return 1;
}
