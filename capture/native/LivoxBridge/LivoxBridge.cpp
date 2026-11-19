#define NOMINMAX
#include "LivoxBridge.h"
#include "livox_sdk.h"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <Windows.h>

namespace {

struct Batch {
  int64_t timestamp_raw = 0;
  uint8_t timestamp_type = 0;
  int64_t host_qpc = 0;
  std::vector<LivoxBridgePoint> points;
};

struct Context {
  std::string broadcast_code;
  uint8_t handle = 0;
  bool device_added = false;
  bool connected = false;
  bool configuring = false;
  bool sampling = false;
  bool running = false;
  int lidar_state = -1;
  int coordinate_status = -1;
  int sampling_status = -1;
  int data_type = -1;
  int64_t callback_count = 0;
  uint8_t firmware_version[4]{};
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<Batch> batches;
  int64_t dropped_batches = 0;
};

std::mutex global_mutex;
Context* active_context = nullptr;

int64_t timestamp_to_raw(const uint8_t* bytes) {
  uint64_t value = 0;
  for (int index = 0; index < 8; ++index) value |= static_cast<uint64_t>(bytes[index]) << (index * 8);
  return static_cast<int64_t>(value);
}

void on_data(uint8_t handle, LivoxEthPacket* packet, uint32_t data_num, void* client_data) {
  auto* context = static_cast<Context*>(client_data);
  if (!context || !packet) return;

  {
    std::lock_guard<std::mutex> lock(context->mutex);
    context->data_type = packet->data_type;
    ++context->callback_count;
  }
  Batch batch;
  batch.timestamp_raw = timestamp_to_raw(packet->timestamp);
  batch.timestamp_type = packet->timestamp_type;
  LARGE_INTEGER qpc{};
  QueryPerformanceCounter(&qpc);
  batch.host_qpc = qpc.QuadPart;
  batch.points.reserve(data_num);
  if (packet->data_type == kCartesian) {
    auto* input = reinterpret_cast<LivoxRawPoint*>(packet->data);
    for (uint32_t index = 0; index < data_num; ++index) {
      LivoxBridgePoint point{};
      point.x_mm = input[index].x;
      point.y_mm = input[index].y;
      point.z_mm = input[index].z;
      point.reflectivity = input[index].reflectivity;
      batch.points.push_back(point);
    }
  } else if (packet->data_type == kExtendCartesian) {
    auto* input = reinterpret_cast<LivoxExtendRawPoint*>(packet->data);
    for (uint32_t index = 0; index < data_num; ++index) {
      LivoxBridgePoint point{};
      point.x_mm = input[index].x;
      point.y_mm = input[index].y;
      point.z_mm = input[index].z;
      point.reflectivity = input[index].reflectivity;
      batch.points.push_back(point);
    }
  } else {
    return;
  }

  std::lock_guard<std::mutex> lock(context->mutex);
  if (!context->running) return;
  if (context->batches.size() >= 8) {
    context->batches.pop_front();
    ++context->dropped_batches;
  }
  context->batches.push_back(std::move(batch));
  context->ready.notify_one();
}

void on_start_sampling(livox_status status, uint8_t handle, uint8_t response, void* client_data) {
  auto* context = static_cast<Context*>(client_data);
  if (!context) return;
  std::lock_guard<std::mutex> lock(context->mutex);
  context->sampling = status == kStatusSuccess && response == 0;
  context->sampling_status = static_cast<int>(status) * 256 + response;
  context->configuring = false;
}

void on_set_coordinate(livox_status status, uint8_t handle, uint8_t response, void* client_data) {
  auto* context = static_cast<Context*>(client_data);
  if (!context) return;
  {
    std::lock_guard<std::mutex> lock(context->mutex);
    context->coordinate_status = static_cast<int>(status) * 256 + response;
  }
  if (status == kStatusSuccess && response == 0) {
    if (LidarStartSampling(handle, on_start_sampling, context) == kStatusSuccess) return;
  }
  std::lock_guard<std::mutex> lock(context->mutex);
  context->configuring = false;
}

void on_broadcast(const BroadcastDeviceInfo* info) {
  Context* context = nullptr;
  {
    std::lock_guard<std::mutex> global_lock(global_mutex);
    context = active_context;
  }
  if (!context || !info || info->dev_type != kDeviceTypeLidarMid70) return;
  {
    std::lock_guard<std::mutex> lock(context->mutex);
    if (!context->broadcast_code.empty() && context->broadcast_code != info->broadcast_code) return;
    if (context->device_added) return;
    context->device_added = true;
  }
  uint8_t handle = 0;
  bool added = AddLidarToConnect(info->broadcast_code, &handle) == kStatusSuccess;
  {
    std::lock_guard<std::mutex> lock(context->mutex);
    if (!added) {
      context->device_added = false;
      return;
    }
    context->handle = handle;
    context->broadcast_code = info->broadcast_code;
    SetDataCallback(handle, on_data, context);
  }
}

void on_device_state(const DeviceInfo* info, DeviceEvent event) {
  Context* context = nullptr;
  {
    std::lock_guard<std::mutex> global_lock(global_mutex);
    context = active_context;
  }
  if (!context || !info) return;
  bool configure = false;
  {
    std::lock_guard<std::mutex> lock(context->mutex);
    if (!context->broadcast_code.empty() && context->broadcast_code != info->broadcast_code) return;
    if ((event == kEventConnect || event == kEventStateChange) && info->handle == context->handle) {
      context->connected = true;
      context->lidar_state = static_cast<int>(info->state);
      std::memcpy(context->firmware_version, info->firmware_version, sizeof(context->firmware_version));
      if (info->state == kLidarStateNormal && !context->configuring && !context->sampling) {
        context->configuring = true;
        configure = true;
      }
    } else if (event == kEventDisconnect && info->handle == context->handle) {
      context->connected = false;
      context->configuring = false;
      context->sampling = false;
      context->lidar_state = -1;
    }
  }
  if (configure && SetCartesianCoordinate(info->handle, on_set_coordinate, context) != kStatusSuccess) {
    std::lock_guard<std::mutex> lock(context->mutex);
    context->configuring = false;
  }
}

}  // namespace

extern "C" {

void* __cdecl livox_create(const char* broadcast_code) {
  auto* context = new Context();
  context->broadcast_code = broadcast_code ? broadcast_code : "";
  return context;
}

int __cdecl livox_start(void* opaque) {
  auto* context = static_cast<Context*>(opaque);
  if (!context) return -1;
  std::lock_guard<std::mutex> global_lock(global_mutex);
  if (active_context) return -2;
  if (!Init()) return -3;
  SetBroadcastCallback(on_broadcast);
  SetDeviceStateUpdateCallback(on_device_state);
  active_context = context;
  context->running = true;
  if (!Start()) {
    context->running = false;
    active_context = nullptr;
    Uninit();
    return -4;
  }
  return 0;
}

int __cdecl livox_poll(void* opaque, LivoxBridgePoint* points, int capacity,
                       int64_t* device_timestamp_raw, uint8_t* timestamp_type,
                       int64_t* host_qpc, int64_t* dropped_batches, int timeout_ms) {
  auto* context = static_cast<Context*>(opaque);
  if (!context || !points || capacity <= 0 || !device_timestamp_raw || !timestamp_type ||
      !host_qpc || !dropped_batches) return -1;
  std::unique_lock<std::mutex> lock(context->mutex);
  if (context->batches.empty()) {
    context->ready.wait_for(lock, std::chrono::milliseconds(std::max(0, timeout_ms)),
                            [context] { return !context->batches.empty() || !context->running; });
  }
  if (context->batches.empty()) return 0;
  Batch batch = std::move(context->batches.front());
  context->batches.pop_front();
  int count = std::min(capacity, static_cast<int>(batch.points.size()));
  std::memcpy(points, batch.points.data(), count * sizeof(LivoxBridgePoint));
  *device_timestamp_raw = batch.timestamp_raw;
  *timestamp_type = batch.timestamp_type;
  *host_qpc = batch.host_qpc;
  *dropped_batches = context->dropped_batches;
  return count;
}

int __cdecl livox_get_device_info(void* opaque, char* broadcast_code, int code_capacity,
                                  uint8_t* firmware_version, int firmware_capacity) {
  auto* context = static_cast<Context*>(opaque);
  if (!context || !broadcast_code || code_capacity <= 0 ||
      !firmware_version || firmware_capacity < 4) return -1;
  std::lock_guard<std::mutex> lock(context->mutex);
  if (!context->connected) return -1;
  std::strncpy(broadcast_code, context->broadcast_code.c_str(), static_cast<size_t>(code_capacity - 1));
  broadcast_code[code_capacity - 1] = '\0';
  std::memcpy(firmware_version, context->firmware_version, 4);
  return 0;
}

int __cdecl livox_get_diagnostics(void* opaque, int* device_added, int* connected,
                                  int* lidar_state, int* configuring, int* sampling,
                                  int* coordinate_status, int* sampling_status,
                                  int* data_type, int64_t* callback_count) {
  auto* context = static_cast<Context*>(opaque);
  if (!context || !device_added || !connected || !lidar_state || !configuring || !sampling ||
      !coordinate_status || !sampling_status || !data_type || !callback_count) return -1;
  std::lock_guard<std::mutex> lock(context->mutex);
  *device_added = context->device_added ? 1 : 0;
  *connected = context->connected ? 1 : 0;
  *lidar_state = context->lidar_state;
  *configuring = context->configuring ? 1 : 0;
  *sampling = context->sampling ? 1 : 0;
  *coordinate_status = context->coordinate_status;
  *sampling_status = context->sampling_status;
  *data_type = context->data_type;
  *callback_count = context->callback_count;
  return 0;
}

void __cdecl livox_stop(void* opaque) {
  auto* context = static_cast<Context*>(opaque);
  if (!context) return;
  {
    std::lock_guard<std::mutex> lock(context->mutex);
    context->running = false;
    context->ready.notify_all();
  }
  bool owns_sdk = false;
  {
    std::lock_guard<std::mutex> global_lock(global_mutex);
    if (active_context == context) {
      active_context = nullptr;
      owns_sdk = true;
    }
  }
  uint8_t handle = 0;
  bool was_connected = false;
  {
    std::lock_guard<std::mutex> lock(context->mutex);
    handle = context->handle;
    was_connected = context->connected;
    context->connected = false;
    context->device_added = false;
    context->configuring = false;
    context->sampling = false;
  }
  if (was_connected) LidarStopSampling(handle, nullptr, nullptr);
  if (owns_sdk) {
    Uninit();
  }
}

void __cdecl livox_destroy(void* opaque) {
  auto* context = static_cast<Context*>(opaque);
  if (!context) return;
  livox_stop(context);
  delete context;
}

}
