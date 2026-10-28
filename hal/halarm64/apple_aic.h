/*
 * PROJECT:     LiberNT HAL — ARM64 Apple AIC platform support
 * LICENSE:     GPL-3.0-or-later (https://spdx.org/licenses/GPL-3.0-or-later)
 * PURPOSE:     Header for the Apple Interrupt Controller backend
 * COPYRIGHT:   Copyright 2026 Ahmed ARIF <arif.ing@outlook.com>
 */

#pragma once

#define APPLE_AIC_MIDR_IMPLEMENTER  0x61

extern BOOLEAN AppleAicDetected;

BOOLEAN
AppleAicInterruptProbe(
    _In_opt_ PLOADER_PARAMETER_BLOCK LoaderBlock);
