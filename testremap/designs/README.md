# Benchmarks

This directory contains contest benchmarks. Each benchmark folder has the following folders:
- EDA_files: This folder has all the provided EDA files of respective design. 
  - <design_name.v>: This is the synthesized netlist generated using ASAP7 library.
  - <design_name_fp.def>: This is the given floorplan for the design. 
  - <design_name.sdc>: This is the given sdc constraint file for the design.
- IR_tables_fp: This folder has the IR tables in csv format for the floorplan stage. Please do `gunzip *` to extract the files in this directory.
- IR_tables_gr: This folder has the IR tables in csv format for the post-global route stage. Please do `gunzip *` to extract the files in this directory.
---

## Note
<i>The provided floorplan for every design cannot be altered. You will be evaluated using the same floorplan as provided.</i>

