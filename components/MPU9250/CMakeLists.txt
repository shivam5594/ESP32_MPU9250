set(MPU9250_srcs "src/MPU9250.cpp"
				 "src/MPU9250_Bus.cpp")

idf_component_register(SRCS "${MPU9250_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES IMU)
