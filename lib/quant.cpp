#include "quant.hpp"
#include "fixquat.hpp"

#include <algorithm>

namespace quant {
static inline single_update quant_update(quat::vec update, int8_t scale) {
    int8_t ux = update.x.raw_value() >> scale;
    int8_t uy = update.y.raw_value() >> scale;
    int8_t uz = update.z.raw_value() >> scale;

    using T = decltype(update.x);
    if (ux != (update.x.raw_value() >> scale) || ux == -128) ux = update.x < T{0} ? -127 : 127;
    if (uy != (update.y.raw_value() >> scale) || uy == -128) uy = update.y < T{0} ? -127 : 127;
    if (uz != (update.z.raw_value() >> scale) || uz == -128) uz = update.z < T{0} ? -127 : 127;
    return single_update{ux, uy, uz};
}

static inline quat::vec dequant_update(single_update update, int8_t scale) {
    using R = quat::base_type;
    return {R::from_raw_value(((int)update.x) << scale),
            R::from_raw_value(((int)update.y) << scale),
            R::from_raw_value(((int)update.z) << scale)};
}

QuantResult quant_block(State state, quat::quat const* quats, size_t n_quats, uint8_t qp,
                               int8_t* out, size_t n_out) {
    size_t bytes_put = 0;
    quat::base_type max_ang_err = {};

    for (size_t i = 0; i < n_quats; ++i) {
        // compute angular acceleration update
        quat::quat q_update = state.q.conj() * quats[i];
        quat::vec v_update = q_update.axis_angle() - state.v;

        // quantize update
        quat::vec sum{};
        bool correction_needed{true};
        while (correction_needed) {
            auto update_quanted = quant_update(v_update, qp);
            auto update_dequanted = dequant_update(update_quanted, qp);

            sum = sum + update_dequanted;
            v_update = v_update - update_dequanted;

            correction_needed = update_quanted.is_saturated();

            if (bytes_put > n_out) {
                return QuantResult{.success = false};
            }
            out[bytes_put + 0] = update_quanted.x;
            out[bytes_put + 1] = update_quanted.y;
            out[bytes_put + 2] = update_quanted.z;
            bytes_put += 3;
        }

        // update state
        state.v = state.v + sum;
        state.q = (state.q * quat::quat(state.v)).normalized();

        // update max quantization error
        max_ang_err = std::max((state.q.conj() * quats[i]).axis_angle().norm(), max_ang_err);
    }

    return QuantResult{
        .success = true, .new_state = state, .bytes_put = bytes_put, .max_ang_err = max_ang_err};
}

bool dequant_one(State& state, int8_t* data, uint8_t qp) {
    single_update upd{.x = data[0], .y = data[1], .z = data[2]};
    state.v = state.v + dequant_update(upd, qp);

    if (!upd.is_saturated()) {
        state.q = (state.q * quat::quat(state.v)).normalized();
        return true;
    }
    return false;
}

}  // namespace quant
