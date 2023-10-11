#pragma once
#include "fixquat.hpp"
#include "quant.hpp"

#include <cstring>

namespace writer {
size_t write_header(uint8_t* out, size_t n_out);

size_t write_gyro_setup(uint16_t samples_per_block, uint8_t* out, size_t n_out);

size_t write_time_block(uint32_t time_elapsed_us, uint8_t* out, size_t n_out);

size_t write_gyro_data(quant::State& state, quat::quat const* quats, size_t n_quats, uint8_t* data,
                       size_t n_data, int8_t* scratch, size_t n_scratch);

size_t write_accel_setup(uint8_t block_size, uint8_t accel_range, uint8_t* out, size_t n_out);

size_t write_accel_data(int16_t const* acc_data, size_t n_acc_data, uint8_t* out, size_t n_out);

size_t write_global_time(int32_t ofs_us, uint8_t* out, size_t n_out);

size_t write_imu_orient(char const* orient, uint8_t* out, size_t n_out);
}  // namespace writer