# CMSIS component provenance

This project vendors the minimum GCC/Cortex-M4 CMSIS subset required by the
STM32G431RB production image. The vendored upstream files are unmodified. They come from the STM32CubeG4 V1.6.3 firmware package.

Pinned component versions:

- STM32CubeG4 firmware package: `V1.6.3`;
- CMSIS Core(M): `5.3.0`;
- STM32G4 CMSIS Device: `1.2.6`;
- target: `STM32G431RBTx`, Cortex-M4;
- compiler family: GNU Arm Embedded GCC.

Included Core headers are `core_cm4.h`, `cmsis_compiler.h`, `cmsis_gcc.h`,
`cmsis_version.h`, and `mpu_armv7.h`. Included Device headers are
`stm32g4xx.h`, `stm32g431xx.h`, and `system_stm32g4xx.h`.
`system_stm32g4xx.c` is copied verbatim to
`Device/ST/STM32G4xx/Source/`; the top-level `Drivers/` production source root
includes it.

The STM32Cube HAL and LL components are not vendored. The optional HAL include
inside the standard `stm32g4xx.h` wrapper remains inactive because
`USE_HAL_DRIVER` is prohibited.

Both Debug and Release configurations contain these include paths:

```text
../Drivers/CMSIS/Include
../Drivers/CMSIS/Device/ST/STM32G4xx/Include
```

Both configurations define the exact device symbol `STM32G431xx` and use the
software floating-point ABI. `USE_HAL_DRIVER` and `USE_FULL_LL_DRIVER` are not
defined and shall remain prohibited.

The Apache-2.0 license files from the Core and STM32G4 Device components are
stored beside their respective sources.
