from openroad import Tech, Design, Timing
import pdn, odb, utl
import openroad as ord
from pathlib import Path
import os, argparse, sys, time, re

# ── Benchmark paths ──────────────────────────────────────────────
BENCHMARK_BASES = {
    "mlcad": Path("/home/culan/Desktop/workspace/MLCAD25-Contest-Scripts-Benchmarks"),
    "iccad": Path("/home/culan/Desktop/workspace/2024_ICCAD_Contest_Gate_Sizing_Benchmark"),
}

# ── Design loading ───────────────────────────────────────────────
def load_design(design_name, design_base, benchmark="mlcad"):
    tech = Tech()
    libDir = design_base / Path("platform/ASAP7/lib/")
    lefDir = design_base / Path("platform/ASAP7/lef/")
    rcFile = design_base / Path("platform/ASAP7/setRC.tcl")

    if benchmark == "iccad":
        designDir = design_base / Path("design/%s" % design_name)
        defFile = "%s/%s.def" % (designDir.as_posix(), design_name)
    else:
        designDir = design_base / Path("designs/%s/EDA_files/" % design_name)
        defFile = "%s/%s_fp.def.gz" % (designDir.as_posix(), design_name)

    for libFile in libDir.glob('*.lib'):
        tech.readLiberty(libFile.as_posix())
    for techLefFile in lefDir.glob("*tech*.lef"):
        tech.readLef(techLefFile.as_posix())
    for lefFile in lefDir.glob('*.lef'):
        tech.readLef(lefFile.as_posix())
    design = Design(tech)

    if benchmark == "iccad":
        design.readDef(defFile)
    else:
        verilogFile = "%s/%s.v" % (designDir.as_posix(), design_name)
        design.readVerilog(verilogFile)
        design.link(design_name)
        design.evalTclString("read_def -floorplan_initialize " + defFile)

    sdcFile = "%s/%s.sdc" % (designDir.as_posix(), design_name)
    design.evalTclString("read_sdc %s" % sdcFile)
    design.evalTclString("source %s" % rcFile.as_posix())

    # Global connect VDD/VSS
    block = design.getBlock()
    for net_name, sig_type in [("VDD", "POWER"), ("VSS", "GROUND")]:
        net = block.findNet(net_name)
        if net is None:
            net = odb.dbNet_create(block, net_name)
        net.setSpecial()
        net.setSigType(sig_type)
    block.addGlobalConnect(None, ".*", "VDD", block.findNet("VDD"), True)
    block.addGlobalConnect(None, ".*", "VSS", block.findNet("VSS"), True)
    block.globalConnect(False, True)

    design.evalTclString("set_thread_count 10")
    return tech, design


def Initialize(tech, design):
    timing = Timing(design)
    db = ord.get_db()
    chip = db.getChip()
    block = ord.get_db_block()
    nets = block.getNets()
    return timing, db, chip, block, nets


def global_placement(design, timing_driven=False):
    td_flag = "-timing_driven" if timing_driven else ""
    design.evalTclString(
        "global_placement -density 0.7 -pad_left 1 -pad_right 1 %s" % td_flag
    )


# ── Score evaluation (extracted from MLCAD_infrastructure) ───────
def get_score(design, design_name, work_dir="/tmp"):
    timing = Timing(design)
    corner = timing.getCorners()[0]

    tmp_file = os.path.join(work_dir, "%s_eval_tmp.txt" % design_name)

    design.evalTclString("report_tns > %s" % tmp_file)
    with open(tmp_file) as f:
        for line in f:
            tns = float(line.split()[-1]) / 1000
    os.remove(tmp_file)

    design.evalTclString("report_wns > %s" % tmp_file)
    with open(tmp_file) as f:
        for line in f:
            wns = float(line.split()[-1]) / 1000
    os.remove(tmp_file)

    slew_total = 0.0
    cap_total = 0.0
    leakage = 0.0

    for pin_ in design.getBlock().getITerms():
        net = pin_.getNet()
        if net is None:
            continue
        sig = net.getSigType()
        if sig in ("POWER", "GROUND", "CLOCK"):
            continue
        name = pin_.getName()
        if re.match(r".*/SETN$", name) or re.match(r".*/RESETN$", name):
            continue
        mterms = pin_.getInst().getMaster().getMTerms()
        library_pin = None
        for mt in mterms:
            if (pin_.getInst().getName() + "/" + mt.getName()) == name:
                library_pin = mt
                break
        if library_pin is None:
            continue
        if timing.getMaxSlewLimit(library_pin) < timing.getPinSlew(pin_):
            slew_total += abs(timing.getMaxSlewLimit(library_pin) - timing.getPinSlew(pin_)) * 1e9
        if pin_.isOutputSignal():
            if timing.getMaxCapLimit(library_pin) < timing.getNetCap(net, corner, timing.Max):
                cap_total += abs(timing.getMaxCapLimit(library_pin) - timing.getNetCap(net, corner, timing.Max)) * 1e15

    for inst in design.getBlock().getInsts():
        leakage += timing.staticPower(inst, corner)
    leakage *= 1e6

    overflow = 0
    score = leakage
    if tns < 0:
        score += abs(tns)
    if slew_total > 0:
        score += slew_total
    if cap_total > 0:
        score += cap_total

    print("===================================================")
    print("WNS: %f ps" % (wns * 1000))
    print("TNS: %f ns" % tns)
    if slew_total > 0:
        print("Total slew violation difference: %f ns" % slew_total)
    else:
        print("No slew violation")
    if cap_total > 0:
        print("Total load capacitance violation difference: %f fF" % cap_total)
    else:
        print("No load capacitance violation")
    print("Overflow is zero")
    print("Leakage power difference: %f uW" % leakage)
    print("Score: %f" % score)
    print("Require runtime in official score calculation")
    print("===================================================")
    sys.stdout.flush()
    return score


# ── LR command definitions ───────────────────────────────────────
LR_COMMANDS = {
    "resize":    "timing.testParallelResizeByArray(20000000, 12, 6, True, 10, lr_mode)",
    "buffering": "timing.testParallelResizeByArrayWithBuffering(20000000, 12, 3, True, 10, lr_mode)",
    "precheck":  "timing.testParallelResizeByArrayWithPrecheck(20000000, 30, 3, True, 10, lr_mode, 0.3)",
    "precheckbuf": "timing.testParallelResizeByArrayWithPrecheckBuffering(20000000, 12, 3, True, 10, lr_mode, 0.3)",
    "lr":        "timing.testParallelLrResizing(20000000, 20, 6, True, 10, lr_mode)",
    "lrbuf":     "timing.testParallelResizingBuffering(20000000, 12, 6, True, 10, lr_mode)",
}
DEFAULT_LR = "precheck"


# ── Main ─────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="LRF test runner")
    parser.add_argument("--design_name", type=str, default="ac97_top")
    parser.add_argument("--benchmark", type=str, default="mlcad",
                        choices=["mlcad", "iccad"])
    parser.add_argument("--lr_mode", type=str, default="RAPIDLRHELPER")
    parser.add_argument("--lr_call", type=str, default="")
    parser.add_argument("--thread_count", type=int, default=10)
    args = parser.parse_args()

    design_base = BENCHMARK_BASES[args.benchmark]
    tech, design = load_design(args.design_name, design_base, args.benchmark)
    (timing, db, chip, block, nets) = Initialize(tech, design)
    timing.makeEquivCells()
    lr_mode = args.lr_mode

    # MLCAD: remove_buffers + global placement + repair_design
    if args.benchmark == "mlcad":
        design.evalTclString("remove_buffers")
        global_placement(design, timing_driven=True)
        design.evalTclString("report_wns")
        design.evalTclString("report_tns")
        design.evalTclString("report_checks -path_delay max -digit 3")
        design.evalTclString("repair_design -verbose")

    design.evalTclString("estimate_parasitics -placement")
    design.evalTclString("report_wns")
    design.evalTclString("report_tns")
    design.evalTclString("report_checks -path_delay max -digit 3")

    # Set thread count
    design.evalTclString("set_thread_count %d" % args.thread_count)
    print("Current thread count: %s" % design.evalTclString("thread_count"))
    sys.stdout.flush()

    # Run LR command
    start_time = time.time()
    design.evalTclString("estimate_parasitics -placement")

    if args.lr_call:
        lr_call = LR_COMMANDS.get(args.lr_call)
        if lr_call is None:
            print("ERROR: Unknown --lr_call '%s'. Available: %s"
                  % (args.lr_call, ", ".join(LR_COMMANDS.keys())))
            sys.exit(1)
    else:
        lr_call = LR_COMMANDS[DEFAULT_LR]

    print("LR_COMMAND: %s" % lr_call)
    sys.stdout.flush()
    eval(lr_call)

    design.evalTclString("estimate_parasitics -placement")
    print("\n[After LR optimization]")
    get_score(design, args.design_name)

    design.evalTclString(
        'repair_timing -setup -sequence "size" -skip_last_gasp -skip_vt_swap -verbose'
    )
    print("\n[After repair_timing]")
    get_score(design, args.design_name)
    design.evalTclString("report_timing_histogram -setup")

    if args.benchmark == "mlcad":
        global_placement(design, timing_driven=True)

    print("\n[After global_placement]")
    get_score(design, args.design_name)
    design.evalTclString("report_timing_histogram -setup")

    end_time = time.time()
    print("Total time: %.1f seconds" % (end_time - start_time))


if __name__ == "__main__":
    main()
