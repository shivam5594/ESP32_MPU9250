set(math_srcs "src/MathLib.cpp")

idf_component_register(SRCS "${math_srcs}"
                       INCLUDE_DIRS "include")
