#pragma once
#include <cstdint>
#include "fixquat.hpp"

namespace quant {
struct single_update {
    int8_t x, y, z;
    bool is_saturated() const { return abs(x) == 127 || abs(y) == 127 || abs(z) == 127; };
};

struct State {
    quat::quat q;
    quat::vec v;
};

struct QuantResult {
    bool success{};
    State new_state{};
    size_t bytes_put{};
    quat::base_type max_ang_err{};
};

QuantResult quant_block(State state, quat::quat const* quats, size_t n_quats, uint8_t qp,
                        int8_t* out, size_t n_out);

bool dequant_one(State& state, int8_t* data, uint8_t qp);

}  // namespace quant