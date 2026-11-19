#pragma once
#include <stdint.h>

#ifdef LIVOX_BRIDGE_EXPORTS
#define LIVOX_API __declspec(dllexport)
#else
#define LIVOX_API __declspec(dllimport)
#endif

extern "C" {

struct LivoxBridgePoint {
  int32_t x_mm;
  int32_t y_mm;
  int32_t z_mm;
  uint8_t reflectivity;
};

LIVOX_API void* __cdecl livox_create(const char* broadcast_code);
LIVOX_API int __cdecl livox_start(void* context);
LIVOX_API int __cdecl livox_poll(void* context, LivoxBridgePoint* points, int capacity,
                                 int64_t* device_timestamp_raw, uint8_t* timestamp_type,
                                 int64_t* host_qpc, int64_t* dropped_batches, int timeout_ms);
LIVOX_API int __cdecl livox_get_device_info(void* context, char* broadcast_code, int code_capacity,
                                            uint8_t* firmware_version, int firmware_capacity);
LIVOX_API int __cdecl livox_get_diagnostics(void* context, int* device_added, int* connected,
                                            int* lidar_state, int* configuring, int* sampling,
                                            int* coordinate_status, int* sampling_status,
                                            int* data_type, int64_t* callback_count);
LIVOX_API void __cdecl livox_stop(void* context);
LIVOX_API void __cdecl livox_destroy(void* context);

}
