px4_add_library(HydrofoilAllocationModel
	HydrofoilAllocationModel.cpp
	HydrofoilAllocationModel.hpp
)

target_include_directories(HydrofoilAllocationModel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(HydrofoilAllocationModel PUBLIC mathlib)

px4_add_unit_gtest(SRC HydrofoilAllocationModelTest.cpp LINKLIBS HydrofoilAllocationModel)
px4_add_unit_gtest(
	SRC ../../modules/hydro_rate_control/SafeRollControllerTest.cpp
	EXTRA_SRCS
		../../modules/hydro_rate_control/SafeRollController.cpp
		../../modules/hydro_rate_control/CADRC.cpp
	INCLUDES ../../modules/hydro_rate_control
	COMPILE_FLAGS -DSAFE_ROLL_DISABLE_PERF
	LINKLIBS mathlib HydrofoilAllocationModel
)
