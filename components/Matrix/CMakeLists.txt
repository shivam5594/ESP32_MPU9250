set(matrix_srcs "src/Matrix.cpp")

idf_component_register(SRCS "${matrix_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES MATLABCoderInit SVD)
