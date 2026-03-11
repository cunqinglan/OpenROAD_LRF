#!/bin/bash

OPENROAD=/home/jzj/physyn/OpenROAD/build/bin/openroad
SCRIPT=/home/jzj/physyn/OpenROAD/testremap/testscript_input_fp.tcl
LOG_DIR=/home/jzj/physyn/OpenROAD/testremap/logs_detailed
mkdir -p "$LOG_DIR"
#designs=("ac97_top" "aes_cipher_top" "fpu" "NV_NVDLA_partition_m" "NV_NVDLA_partition_p" "NV_NVDLA_partition_p_mod")
designs=("ac97_top" "aes_cipher_top" "fpu")
for design in "${designs[@]}"; do
    echo "Starting: $design"
    DESIGN_NAME="$design" "$OPENROAD" "$SCRIPT" \
        > "$LOG_DIR/${design}.log" 2>&1 &
done
wait
echo "All done."