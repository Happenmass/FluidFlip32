#pragma once

// Minimal Unity config for native host test builds. Unity's `unity_internals.h`
// includes this when `UNITY_INCLUDE_CONFIG_H` is defined (PlatformIO does so).
#define UNITY_SUPPORT_64
#define UNITY_FLOAT_DETAILED_OUTPUT
