#include <cstddef>
#include <iostream>
#include <vector>

#include "lib/laplace_model.hpp"
#include "lib/fixquat.hpp"
#include "lib/quant.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cstring>

std::vector<quat::quat> load_raw_q(char const* path) {
    std::vector<quat::quat> quats;
    int fd = open(path, O_RDONLY);
    char buf[sizeof(quat::quat) * 1024];
    int nread{};
    while ((nread = read(fd, buf, sizeof(buf))) > 0) {
        for (int i = 0; i < nread / sizeof(quat::quat); ++i) {
            quats.push_back(((quat::quat*)buf)[i]);
        }
    }
    // std::cout << quats.size() << std::endl;
    return quats;
}

int main() {
    // for (int i = -128; i < 129; ++i) {
    //     std::cout << model::cdf(i, 2) << std::endl;
    // }

    auto quats = load_raw_q("test.rawquat");

    // quat::quat pq{};
    // for (auto q : quats) {
    //     auto dq = pq.conj() * q;
    //     std::cout << dq.w.raw_value() << " " << dq.x.raw_value() << " " << dq.y.raw_value() << "
    //     " << dq.z.raw_value()
    //               << std::endl;
    //     pq = q;
    // }

    quant::State state{};
    int8_t out[1024 * 1024];
    quant::quant_block(state, quats.data(), 8192, 14, out, sizeof(out));

    return 0;
}