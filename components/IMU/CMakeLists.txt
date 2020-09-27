set(imu_srcs "src/IMU.cpp")

idf_component_register(SRCS "${imu_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos Matrix MathLib BasicMath MATLABCoderInit)
