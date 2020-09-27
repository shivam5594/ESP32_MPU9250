set(matlab_srcs "src/MATLABCoderInit.cpp"
				"src/rt_nonfinite.cpp"
				"src/rtGetInf.cpp"
				"src/rtGetNaN.cpp")

idf_component_register(SRCS "${matlab_srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES BasicMath Matrix)
