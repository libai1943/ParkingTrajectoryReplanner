// Public C ABI adapter. Protected LIOM/search implementation is in the DLL.
// Copyright 2022-2026 Bai Li. PolyForm Noncommercial 1.0.0.
#pragma once
#define NOMINMAX
#include <windows.h>
#include <array>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

using State = std::array<double, 7>;
using Sample = std::array<double, 8>; // t, x, y, theta, v, a, phi, omega
using Trajectory = std::vector<Sample>;
namespace fs = std::filesystem;

class ParkingBackend {
 public:
  ParkingBackend(const fs::path& root, const fs::path& runtime, int case_id) {
    SetDllDirectoryW(runtime.c_str());
    wchar_t previous[32768];
    GetEnvironmentVariableW(L"PATH", previous, 32768);
    SetEnvironmentVariableW(L"PATH", (runtime.wstring() + L";" + previous).c_str());
    SetEnvironmentVariableW(L"OPENBLAS_NUM_THREADS", L"1");
    library_ = LoadLibraryW((root / "native/win64/parking_backend.dll").c_str());
    if (!library_) throw std::runtime_error("Cannot load protected backend. Check --runtime and CasADi 3.7.2.");
    error_ = symbol<const char*(*)()>("ps_error");
    free_ = symbol<void(*)(void*)>("ps_free");
    original_ = symbol<int(*)(void*,double*,int)>("ps_original");
    evasive_ = symbol<int(*)(void*,double*,int)>("ps_evasive");
    connect_ = symbol<int(*)(const double*,const double*,double*,int)>("ps_connect");
    collision_ = symbol<int(*)(void*,const double*,int)>("ps_collision_free");
    auto load = symbol<void*(*)(const char*,int)>("ps_load");
    scene_ = load((root / "data/cases_data.mat").u8string().c_str(), case_id);
    if (!scene_) throw std::runtime_error(error_());
    symbol<int(*)(void*,double*)>("ps_metadata")(scene_, metadata.data());
  }
  ~ParkingBackend() { if (scene_) free_(scene_); if (library_) FreeLibrary(library_); }
  ParkingBackend(const ParkingBackend&) = delete;
  ParkingBackend& operator=(const ParkingBackend&) = delete;
  Trajectory original() const {
    int count = original_(scene_, nullptr, 0);
    Trajectory output(count); original_(scene_, output[0].data(), count); return output;
  }
  Trajectory evasive() const {
    Trajectory output(100); check(evasive_(scene_, output[0].data(), 100)); return output;
  }
  Trajectory connect(const State& start, const State& finish) const {
    Trajectory output(100); check(connect_(start.data(), finish.data(), output[0].data(), 100)); return output;
  }
  bool collision_free(const Trajectory& trajectory) const {
    return collision_(scene_, trajectory.front().data(), static_cast<int>(trajectory.size())) == 1;
  }
  std::array<double, 4> metadata{}; // t0, t1, tbrake, original terminal time

 private:
  template<class Function> Function symbol(const char* name) {
    auto result = GetProcAddress(library_, name);
    if (!result) throw std::runtime_error(std::string("Missing backend entry: ") + name);
    return reinterpret_cast<Function>(result);
  }
  void check(int code) const { if (code < 0) throw std::runtime_error(error_()); }
  HMODULE library_ = nullptr;
  void* scene_ = nullptr;
  const char* (*error_)() = nullptr;
  void (*free_)(void*) = nullptr;
  int (*original_)(void*,double*,int) = nullptr;
  int (*evasive_)(void*,double*,int) = nullptr;
  int (*connect_)(const double*,const double*,double*,int) = nullptr;
  int (*collision_)(void*,const double*,int) = nullptr;
};
