#include "writer.hpp"
#include "compress.hpp"

namespace writer {
size_t write_header(uint8_t* out, size_t n_out) {
    if (n_out < 7) {
        return 0;
    }
    memcpy(out, "EspLog0", 7);
    return 7;
}

size_t write_gyro_setup(uint16_t samples_per_block, uint8_t* out, size_t n_out) {
    if (n_out < 4) {
        return 0;
    }
    out[0] = 1;  // block id
    out[1] = 1;  // revision
    out[2] = (samples_per_block >> 0) & 0xff;
    out[3] = (samples_per_block >> 8) & 0xff;
    return 4;
}

size_t write_time_block(uint32_t time_elapsed_us, uint8_t* out, size_t n_out) {
    if (n_out < 5) {
        return 0;
    }
    out[0] = 2;  // block id
    out[1] = (time_elapsed_us >> 0) & 0xff;
    out[2] = (time_elapsed_us >> 8) & 0xff;
    out[3] = (time_elapsed_us >> 16) & 0xff;
    out[4] = (time_elapsed_us >> 24) & 0xff;
    return 5;
}

size_t write_gyro_data(quant::State& state, quat::quat const* quats, size_t n_quats, uint8_t* data,
                       size_t n_data, int8_t* scratch, size_t n_scratch) {
    if (n_data < 3) {
        return 0;
    }
    data[0] = 3;  // block id
    compress::CompressResult res = compress::compress_block(state, quats, n_quats, 14, data + 1,
                                                            n_data - 1, scratch, n_scratch);
    if (!res.success) {
        res = compress::compress_block(state, quats, n_quats, 20, data + 1, n_data - 1, scratch,
                                       n_scratch);
    }
    if (res.success) {
        state = res.new_state;
    }

    return res.bytes_put + 1;
}

size_t write_accel_setup(uint8_t block_size, uint8_t accel_range, uint8_t* out, size_t n_out) {
    if (n_out < 3) {
        return 0;
    }
    out[0] = 4;  // block id
    out[1] = block_size;
    out[2] = accel_range;
    return 3;
}

size_t write_accel_data(int16_t const* acc_data, size_t n_acc_data, uint8_t* out, size_t n_out) {
    if (n_out < n_acc_data * 6 + 2 || n_acc_data > 255) {
        return 0;
    }
    out[0] = 5;  // block id
    for (size_t i = 0; i < 3 * n_acc_data; ++i) {
        out[1 + 2 * i] = (acc_data[i] >> 0) & 0xff;
        out[1 + 2 * i + 1] = (acc_data[i] >> 8) & 0xff;
    }
    return n_acc_data * 6 + 1;
}

size_t write_global_time(int32_t ofs_us, uint8_t* out, size_t n_out) {
    if (n_out < 5) {
        return 0;
    }
    out[0] = 0x06;  // block id
    out[1] = (((uint32_t)ofs_us) >> 0) & 0xff;
    out[2] = (((uint32_t)ofs_us) >> 8) & 0xff;
    out[3] = (((uint32_t)ofs_us) >> 16) & 0xff;
    out[4] = (((uint32_t)ofs_us) >> 24) & 0xff;
    return 5;
}

size_t write_imu_orient(char const* orient, uint8_t* out, size_t n_out) {
    if (n_out < 4) {
        return 0;
    }
    out[0] = 0x07;  // block id
    out[1] = orient[0];
    out[2] = orient[1];
    out[3] = orient[2];
    return 4;
}
}  // namespace writer