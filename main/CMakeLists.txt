set(srcs "ESP32_MPU9250.cpp")

# Only ESP32 has enough memory to do jpeg decoding
if(IDF_TARGET STREQUAL "esp32")
    
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
                    REQUIRES IMU MPU9250)

