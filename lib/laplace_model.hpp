#pragma once
#include <cstdint>

namespace model {
int cdf(int x, int i_var);
int icdf(int y, int i_var);
int freq(int x, int i_var);

uint8_t var_to_ivar(double var);
double ivar_to_var(int v);
int scale();
}  // namespace model