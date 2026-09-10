#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
build_dir="${TMPDIR:-/tmp}/px4_stage5a_host_tests"
mkdir -p "$build_dir"

"${CXX:-c++}" -std=c++14 -Wall -Wextra -Werror \
	-I"$repo_root/src/modules/control_allocator" \
	-I"$repo_root/src/modules/fixed_mpc_shadow" \
	"$repo_root/src/modules/control_allocator/RollFinOutputSelectorTest.cpp" \
	-o "$build_dir/roll_fin_output_selector_test"

"$build_dir/roll_fin_output_selector_test"

# The ownership selector must remain upstream of the standard output safety layer.
grep -q 'output_limit_calc' "$repo_root/src/lib/mixer_module/mixer_module.cpp"
grep -q '_armed.lockdown' "$repo_root/src/lib/mixer_module/mixer_module.cpp"
grep -q '_armed.force_failsafe' "$repo_root/src/lib/mixer_module/mixer_module.cpp"
printf 'hardware_safety_source_check,PASS,Standard disarm lockdown and force-failsafe output layer remains present\n'
