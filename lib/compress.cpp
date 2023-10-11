#include "compress.hpp"
#include "fixquat.hpp"
#include "quant.hpp"
#include "laplace_model.hpp"

#include <algorithm>

namespace compress {
static constexpr uint32_t RANS_BYTE_L = 1U << 23;

static inline size_t rans_encode(int8_t* data, size_t n_data, uint8_t* out, size_t n_out,
                                 uint8_t i_var) {
    uint32_t state = RANS_BYTE_L;
    size_t bytes_put = 0;
    for (size_t i = n_data; i--;) {
        int8_t sym = data[i];
        int start = model::cdf(sym, i_var);
        int freq = model::cdf(sym + 1, i_var) - start;
        uint32_t x_max = ((RANS_BYTE_L >> model::scale()) << 8) * freq;
        while (state >= x_max) {
            if (bytes_put >= n_out) {
                return 0;
            }
            out[bytes_put] = state & 0xff;
            bytes_put += 1;
            state >>= 8;
        }
        state = ((state / freq) << model::scale()) + (state % freq) + start;
    }
    out[bytes_put + 0] = (state >> 24);
    out[bytes_put + 1] = (state >> 16);
    out[bytes_put + 2] = (state >> 8);
    out[bytes_put + 3] = (state >> 0);
    bytes_put += 4;
    std::reverse(out, out + bytes_put);
    return bytes_put;
}

CompressResult compress_block(quant::State state, quat::quat const* quats, size_t n_quats,
                              uint8_t qp, uint8_t* data, size_t n_data, int8_t* scratch,
                              size_t n_scratch) {
    auto quant_result = quant::quant_block(state, quats, n_quats, qp, scratch, n_scratch);
    if (!quant_result.success) {
        return CompressResult{.success = false};
    }

    int i_var{};
    uint8_t cksum{};
    {
        int64_t sum = 0;
        for (size_t i = 0; i < quant_result.bytes_put; ++i) {
            sum += (int64_t)scratch[i] * (int64_t)scratch[i];
            cksum += (uint8_t)scratch[i];
        }
        i_var = model::var_to_ivar(double(sum) / quant_result.bytes_put);
    }

    size_t rans_result = rans_encode(scratch, quant_result.bytes_put, data + 2, n_data - 2, i_var);
    if (rans_result == 0) {
        return CompressResult{.success = false};
    }

    data[0] = qp;
    data[1] = (i_var) | (cksum << 5);

    return CompressResult{.success = true,
                          .new_state = quant_result.new_state,
                          .bytes_put = rans_result + 2,
                          .dbg_qbytes = quant_result.bytes_put};
}

DecompressResult decompress_block(quant::State state, uint8_t const* data, size_t n_data,
                                  quat::quat* quats, size_t n_quats) {
    uint8_t qp = data[0];
    uint8_t i_var = data[1] & 0x1f;
    uint8_t cksum = data[1] >> 5;

    uint32_t rstate = (((uint32_t)data[2]) << 0) | (((uint32_t)data[3]) << 8) |
                      (((uint32_t)data[4]) << 16) | (((uint32_t)data[5]) << 24);
    size_t bytes_eaten = 6;

    size_t quats_put{};
    uint8_t own_cksum{};
    uint32_t mask = (1U << model::scale()) - 1;
    while (quats_put < n_quats) {
        int8_t s[3];
        for (size_t i = 0; i < 3; ++i) {
            int cum = rstate & mask;
            int sym = model::icdf(cum, i_var);
            s[i] = sym;
            own_cksum += (uint8_t)s[i];

            int start = model::cdf(sym, i_var);
            int freq = model::cdf(sym + 1, i_var) - start;

            rstate = freq * (rstate >> model::scale()) + (rstate & mask) - start;

            while (rstate < RANS_BYTE_L) {
                if (bytes_eaten >= n_data) {
                    return DecompressResult{.success = false};
                }
                rstate = (rstate << 8) | data[bytes_eaten];
                bytes_eaten += 1;
            }
        }

        if (quant::dequant_one(state, s, qp)) {
            if (quats_put >= n_quats) {
                return DecompressResult{.success = false};
            }
            quats[quats_put] = state.q;
            quats_put += 1;
        }
    }
    if (own_cksum != cksum) {
        return DecompressResult{.success = false};
    }
    return DecompressResult{
        .success = true, .new_state = state, .bytes_eaten = bytes_eaten, .quats_put = quats_put};
}
}  // namespace compress