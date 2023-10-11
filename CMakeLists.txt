project(ebin-encoder)

add_executable(main main.cpp lib/laplace_model.cpp lib/quant.cpp lib/compress.cpp)
add_executable(test_distr test_distr.cpp lib/laplace_model.cpp lib/quant.cpp lib/compress.cpp)