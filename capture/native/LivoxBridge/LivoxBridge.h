#pragma once
#include <stdint.h>

#ifdef LIVOX_BRIDGE_EXPORTS
#define LIVOX_API __declspec(dllexport)
#else
#define LIVOX_API __declspec(dllimport)
#endif

extern "C" {

struct LivoxBridgePoint {
  float x;
  float y;
  float z;
  uint8_t reflectivity;
};

LIVOX_API void* __cdecl livox_create(const char* broadcast_code);
LIVOX_API int __cdecl livox_start(void* context);
LIVOX_API int __cdecl livox_poll(void* context, LivoxBridgePoint* points, int capacity,
                                 int64_t* device_timestamp_raw, int timeout_ms);
LIVOX_API void __cdecl livox_stop(void* context);
LIVOX_API void __cdecl livox_destroy(void* context);

}
