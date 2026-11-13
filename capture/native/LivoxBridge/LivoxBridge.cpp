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

namespace {

struct Batch {
  int64_t timestamp_raw = 0;
  std::vector<LivoxBridgePoint> points;
};

struct Context {
  std::string broadcast_code;
  uint8_t handle = 0;
  bool connected = false;
  bool running = false;
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<Batch> batches;
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
  if (!context || !packet || packet->data_type != kCartesian) return;

  auto* input = reinterpret_cast<LivoxRawPoint*>(packet->data);
  Batch batch;
  batch.timestamp_raw = timestamp_to_raw(packet->timestamp);
  batch.points.reserve(data_num);
  for (uint32_t index = 0; index < data_num; ++index) {
    LivoxBridgePoint point{};
    point.x = input[index].x / 1000.0f;
    point.y = input[index].y / 1000.0f;
    point.z = input[index].z / 1000.0f;
    point.reflectivity = input[index].reflectivity;
    batch.points.push_back(point);
  }

  std::lock_guard<std::mutex> lock(context->mutex);
  if (!context->running) return;
  if (context->batches.size() >= 8) context->batches.pop_front();
  context->batches.push_back(std::move(batch));
  context->ready.notify_one();
}

void on_broadcast(const BroadcastDeviceInfo* info) {
  Context* context = nullptr;
  {
    std::lock_guard<std::mutex> global_lock(global_mutex);
    context = active_context;
  }
  if (!context || !info || info->dev_type != kDeviceTypeLidarMid70) return;
  if (!context->broadcast_code.empty() && context->broadcast_code != info->broadcast_code) return;
  uint8_t handle = 0;
  if (AddLidarToConnect(info->broadcast_code, &handle) == kStatusSuccess)
    context->handle = handle;
}

void on_device_state(const DeviceInfo* info, DeviceEvent event) {
  Context* context = nullptr;
  {
    std::lock_guard<std::mutex> global_lock(global_mutex);
    context = active_context;
  }
  if (!context || !info) return;
  if (!context->broadcast_code.empty() && context->broadcast_code != info->broadcast_code) return;

  if (event == kEventConnect) {
    if (info->handle == context->handle) {
      context->connected = true;
      SetDataCallback(context->handle, on_data, context);
      SetCartesianCoordinate(context->handle, nullptr, nullptr);
      LidarStartSampling(context->handle, nullptr, nullptr);
    }
  } else if (event == kEventDisconnect && info->handle == context->handle) {
    context->connected = false;
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
    Stop();
    context->running = false;
    active_context = nullptr;
    Uninit();
    return -4;
  }
  return 0;
}

int __cdecl livox_poll(void* opaque, LivoxBridgePoint* points, int capacity,
                       int64_t* device_timestamp_raw, int timeout_ms) {
  auto* context = static_cast<Context*>(opaque);
  if (!context || !points || capacity <= 0 || !device_timestamp_raw) return -1;
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
  return count;
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
  if (context->connected) LidarStopSampling(context->handle, nullptr, nullptr);
  if (owns_sdk) {
    Stop();
    Uninit();
  }
  context->connected = false;
}

void __cdecl livox_destroy(void* opaque) {
  auto* context = static_cast<Context*>(opaque);
  if (!context) return;
  livox_stop(context);
  delete context;
}

}
