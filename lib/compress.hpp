#pragma once
#include "quant.hpp"

namespace compress {
struct CompressResult {
    bool success{};
    quant::State new_state{};
    size_t bytes_put{};
    size_t dbg_qbytes{};
};

struct DecompressResult {
    bool success;
    quant::State new_state;
    size_t bytes_eaten;
    size_t quats_put;
};

CompressResult compress_block(quant::State state, quat::quat const* quats, size_t n_quats,
                              uint8_t qp, uint8_t* data, size_t n_data, int8_t* scratch,
                              size_t n_scratch);

DecompressResult decompress_block(quant::State state, uint8_t const* data, size_t n_data,
                                  quat::quat* quats, size_t n_quats);
}  // namespace compress