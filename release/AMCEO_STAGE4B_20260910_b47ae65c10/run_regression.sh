#!/usr/bin/env bash
set -euo pipefail
cd /home/linyi666/PX4-Autopilot-water-air
test_output=$(mktemp -d /tmp/amceo-regression-20260910.XXXXXX)
controller_dir=src/modules/hydro_rate_control
g++ -std=c++14 -Wall -Wextra -Werror \
  -Ibuild/matek_h743_dummy -Isrc -Isrc/lib -Isrc/lib/matrix \
  -Iplatforms/common -Iplatforms/common/include \
  "$controller_dir/test/CadrcGoldenTest.cpp" "$controller_dir/CADRC.cpp" \
  -o "$test_output/CadrcGoldenTest"
"$test_output/CadrcGoldenTest"
for test_name in HydroRollInfrastructureTest LadrcGoldenTest MceoGoldenTest AmceoGoldenTest AmceoMatlabConsistencyTest NadrcMathTest NadrcGoldenTest; do
  g++ -std=c++14 -Wall -Wextra -Werror \
    "$controller_dir/test/$test_name.cpp" \
    "$controller_dir/HydroRollController.cpp" "$controller_dir/RollReferenceGenerator.cpp" \
    "$controller_dir/LADRC.cpp" "$controller_dir/MCEO_ADRC.cpp" \
    "$controller_dir/AMCEO_ADRC.cpp" "$controller_dir/ClassicalNADRC.cpp" \
    -o "$test_output/$test_name"
  "$test_output/$test_name"
done
for contract in Stage2BSourceContractTest Stage3SourceContractTest Stage4SourceContractTest Stage4BNadrcSourceContractTest; do
  python3 "$controller_dir/test/$contract.py"
done
git diff --check
printf 'Test executables: %s\n' "$test_output"
