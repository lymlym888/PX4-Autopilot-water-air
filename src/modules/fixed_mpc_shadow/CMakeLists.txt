px4_add_module(
	MODULE modules__fixed_mpc_shadow
	MAIN fixed_mpc_shadow
	SRCS
		fixed_mpc_shadow_main.cpp
		FixedLinearMPC.cpp
		FixedLinearMPC.hpp
		SafetyGate.cpp
		SafetyGate.hpp
		FixedMPCDiagnostics.hpp
		FixedMPCParams.hpp
	DEPENDS
		px4_work_queue
	)
