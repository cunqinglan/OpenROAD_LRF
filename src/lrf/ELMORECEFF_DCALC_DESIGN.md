# ElmoreCeff Delay Calculator for the `lrf` Local STA — Design & Refactor Plan

**Status:** Phase A skeleton landed; Phase A wiring + Phase B pending.
**Companion document:** *"ElmoreCeff: A GPU‑Friendly Elmore‑Like Delay Calculator with a Closed‑Form Effective Capacitance Model"*, Liu/Guo/Wang/Lin, ISEDA 2026 (PDF shipped alongside this file). Equation/Algorithm numbers below refer to that paper.
**Scope:** the custom local STA engine in `openroad/src/lrf/` (not the OpenSTA submodule).
**Build/test note:** all `file:line` references are current as of 2026‑05 on branch `develop_FF_resize`. The dev machine that produced this doc is macOS and cannot build OpenROAD; build + integration tests run on the Linux box (`/home/phy/PhyLS/openroad`, see repo `CLAUDE.md`).

---

## 0. TL;DR

We are replacing the delay model the `lrf` local STA uses for interconnect from the **iterative Dartu/Menezes/Pileggi (DMP) effective‑capacitance** calculator (OpenSTA `dmp_ceff_elmore`) with the paper's **closed‑form ElmoreCeff** model.

- **Why:** the local STA is the hot inner‑loop evaluator for the Lagrangian‑relaxation optimization (rebuffering / sizing / cloning what‑ifs). DMP does nested Newton iteration + per‑load root‑finding per net — expensive and data‑dependent. ElmoreCeff is non‑iterative (≈ Elmore speed) with accuracy between Elmore and MOR, and the paper shows it yields **better final QoR** than a plain‑Elmore‑guided flow.
- **What changes:** three coupled pieces — (1) parasitic reduction, (2) the driver‑gate Ceff solve, (3) the load wire‑delay/slew model.
- **How:** mirror OpenSTA's pluggable `ArcDelayCalc` pattern. **Phase A** packages today's DMP behavior into a swappable module (`LocalDmpDelayCalc`, behavior‑preserving). **Phase B** adds `LocalElmoreCeffDelayCalc` on the same interface.

---

## 1. Motivation — why upgrade

### 1.1 Where the local STA sits
`lrf` is a fast incremental STA used inside the LR optimization loop. Every candidate move (buffer insert/remove, resize, clone, pin swap) is evaluated by re‑timing locally. The delay calculator is therefore on the critical path of the whole optimizer: its speed bounds how many what‑ifs we can afford, and its accuracy directly determines move quality.

### 1.2 Two delay paths today
- **Real loads** (load pin exists): routed through OpenSTA's `dmp_ceff_elmore` via `arc_delay_calc->gateDelay(...)`.
- **Virtual loads** (pin‑less, e.g. virtual buffers during rebuffering): handled by an inline **plain‑Elmore** wire‑delay/slew formula in `LocalSta::annotateLoadDelays`.

So today we pay for DMP where we have real pins, and fall back to a cruder Elmore where we don't.

### 1.3 What DMP actually costs (read from `openroad/src/sta/dcalc/DmpCeff.cc`)
Per driver net:
1. **A 3×3 Newton solve** for `(t0, dt, Ceff)` — `newtonRaphson(max_iter=100, …)` at `DmpCeff.cc:344`. Each Newton step evaluates 3 residuals `[y20, y50, ipi]` + a 3×3 Jacobian full of `exp()` (`DmpCeff.cc:988‑1017`), **with an NLDM table lookup inside the loop**, then an LU decompose/solve.
2. **3 root‑finds** on the driver waveform `Vo(t)` for the vth/vl/vh crossings (`DmpCeff.cc:484‑486`, each `findRoot` up to 20 iters).
3. **3 root‑finds per fanout load** on the load waveform `Vl(t)` (`DmpCeff.cc:578‑580`, loop at `DmpCeff.cc:1519`).

Net result per net: nested iteration + hundreds of `exp()` + several table lookups + LU, with **data‑dependent convergence** (bad for SIMT/GPU, and variable CPU cost).

### 1.4 What ElmoreCeff gives (paper)
- **Speed:** runs at essentially Elmore speed (paper Table III: 0.95× the Elmore runtime; ~2.62× faster than Arnoldi MOR; up to 40.95× vs CPU MOR). It is *non‑iterative* — no Newton, no root‑finding, no `exp()` in the Ceff pass.
- **Accuracy:** "Moderate" — between plain Elmore and MOR (paper Table IV: R²≈0.965 vs Elmore 0.948, OpenSTA/Arnoldi 0.976/0.989).
- **QoR:** paper Table V — an ElmoreCeff‑guided `repair_timing` flow beats a plain‑Elmore‑guided one on signoff WNS/TNS (up to 8.4% WNS / 5.4% TNS).
- **GPU‑friendly:** the Ceff pass is structurally isomorphic to the Elmore capacitance pass (a fixed post‑order traversal), so it parallelizes like Elmore.

**Sweet spot for us:** much faster than DMP, much more accurate than the plain‑Elmore virtual‑load path, and it unifies both paths into one model.

---

## 2. Theory — the ElmoreCeff model (原理)

### 2.1 The need for effective capacitance
NLDM cell tables are characterized with a **pure capacitive load**: `delay/slew = LUT(in_slew, Cload)`. A real RC load is not a lumped cap — interconnect resistance **shields** far capacitance, so the driver effectively sees less than the total cap `Ctot`. Using `Ctot` is pessimistic. Effective capacitance `Ceff` is the single cap that draws the **same charge** from the driver over the transition as the full RC network; feed `Ceff` (not `Ctot`) into the NLDM lookup.

### 2.2 Closed‑form Ceff for a 1R1C (paper Eq.4→9)
Charge conservation over the input ramp `Ts` (Eq.4): the charge the driver delivers must match between the real RC load and its `Ceff` equivalent. Solving the ramp‑driven RC gives a charge function (Eq.5)

```
h(C, R) = C · [ 1 − (R·C/Ts)·(1 − e^(−Ts/(R·C))) ]
```

and charge balance becomes (Eq.6): `h(C, Rd+R) = h(Ceff, Rd)`.

**Key subtlety:** in this *exact* (transcendental) form, the driver resistance `Rd` is buried in the exponentials and does **not** cancel — that is precisely why DMP must solve it numerically. The paper applies the **[1,1] Padé approximant** `e^(−z) ≈ (1−z/2)/(1+z/2)` (Eq.7), turning `h` into a rational function (Eq.8):

```
h(C, R) ≈ C·Ts / (Ts + 2·R·C)
```

Now `Rd` appears only linearly and cancels on cross‑multiplication, leaving the **closed‑form** (Eq.9):

```
Ceff = C·Ts / (Ts + 2·R·C)
```

This `Rd`‑independence is what licenses applying the formula **per branch** regardless of what drives the branch upstream — the basis for the tree generalization.

### 2.3 Applying it to the Pi model (what reduction gives us today)
`LocalReduceParasitic.cc` reduces the full RC net to a **Pi model** (O'Brien–Savarino moment matching): `c2` (near cap, on the driver node), `rpi` (shield R), `c1` (far cap). In code (`LocalReduceParasitic.cc:92‑94`): `c1 = y2²/y3`, `c2 = y1 − c1`, `rpi = −y3²/y2³`; `capacitance() = c1 + c2 = Ctot`.

Eq.9's minimal circuit has only a far cap, so you must **not** push `Ctot` through Eq.9 (that would wrongly shield the near cap). Use charge **superposition** (Eq.10): the near cap is local to the driver node (unshielded, contributes fully); only the far cap is shielded through `rpi`:

```
Ceff = c2 + c1·Ts / (Ts + 2·rpi·c1)
```

Limits check: `rpi→0 ⇒ Ceff→c1+c2` (no shielding = total cap); `rpi→∞ ⇒ Ceff→c2` (near cap only). These match DMP's bounds `Ceff ∈ [c2, c1+c2]`.

### 2.4 Full‑tree generalization — Algorithm 2 (the chosen fidelity)
We are doing the **full‑tree** version (not just Pi‑level). The driver‑node Ceff is the superposition of its local cap plus each child branch reduced via Eq.9, recursively (Eq.11):

```
Ceff(n) = C_n + Σ_{m ∈ children(n)}  Ceff(m)·Tn / (Tn + 2·R_m·Ceff(m))
```

Implemented as a **post‑order traversal** (Algorithm 2), isomorphic to the Elmore capacitance pass:

```
ramp_factor = 0.8
Ceff[*] = 0
for each node n in post-order (children before parent):
    Ceff[n] += C_n
    m  = father(n)
    Rn = resistance of branch (m, n)
    Tn = slew[m] / ramp_factor          # ramp duration from the PARENT node's slew
    Ceff[m] += Ceff[n] · Tn / (Tn + 2·Rn·Ceff[n])
return Ceff      # Ceff[root] is what feeds the NLDM gate lookup
```

**Isomorphism with Elmore:** the only change vs the Elmore cap pass `load[m] += load[n]` is the **shielding factor** `Tn/(Tn + 2·Rn·Ceff[n]) ∈ (0,1]`. `Rn→0` ⇒ factor→1 ⇒ reduces to plain Elmore. Same traversal, same data structures.

### 2.5 Refined slew (paper §III‑C, Eq.12→15) — supplies `Tn`
Probabilistic interpretation: the step response is a CDF, its derivative a PDF. **Delay = mean** (= Elmore delay); **slew ∝ standard deviation** of the impulse response. The variance is obtained for free from Elmore moments (Algorithm 1):

```
delay[n]  = Σ R·C_down                 # 1st moment (Elmore delay)
ldelay[n] = Σ C·delay
beta[n]   = Σ R·ldelay                 # 2nd moment m2 = 2·beta[n]
variance  = m2 − m1² = 2·beta[n] − delay[n]²
```

Slew propagation through an LTI stage adds variances (convolution) → **sum of squares** (Eq.13/14), giving the refined per‑node slew (Eq.15):

```
slew[n] = sqrt( slew_i² + slew_factor² · (2·beta[n] − delay[n]²) )
```

`slew_i` = driver output slew from the NLDM table. `slew_factor = √constant`, chosen **per corner** (heuristic): `√(2π)≈2.51` for slow corners, `1.0` for fast corners. The ramp duration for the Ceff pass is `Tn = slew[node]/ramp_factor` (ramp_factor=0.8 converts a 10‑90% slew to a 0‑100% ramp).

### 2.6 Coupling / single refinement pass
`slew_i` needs the driver table lookup, which needs `Ceff` — a chicken‑and‑egg. The paper does a **single refinement** (no iteration):

```
(1) initial driver slew via total-cap lookup
(2) Elmore pass → delay[n], beta[n], refined slew[n]   (Algorithm 1 + Eq.15)
(3) Ceff pass with Tn = slew[parent]/ramp_factor       (Algorithm 2)
(4) re-lookup driver gate delay/slew with Ceff[root]
    wire delay = Elmore delay;  load slew = Eq.15
```

At most one Ceff↔driver‑slew iteration; everything is closed form.

### 2.7 Why ElmoreCeff is faster than DMP (summary)

| | DMP (`dmp_ceff_elmore`, today) | ElmoreCeff (target) |
|---|---|---|
| Driver Ceff | 3×3 Newton, table lookup + LU inside loop | 1 post‑order pass, pure arithmetic |
| Iteration | Newton (outer) ⊃ root‑find (inner), nested | none (deterministic traversal) |
| Transcendentals | many `exp()` per Newton/root step | none in Ceff pass (one `sqrt` in slew) |
| NLDM lookups | inside Newton loop + driver | 1 per net (driver) |
| Per fanout load | 3 iterative root‑finds | O(1) closed form |
| GPU | data‑dependent → divergent | isomorphic to Elmore → friendly |

---

## 3. New dcalc design

### 3.1 Mirror OpenSTA's pluggable `ArcDelayCalc`
OpenSTA hierarchy (read for reference):

```
ArcDelayCalc (abstract: gateDelay / inputPortDelay / loadDelaySlew / findParasitic / reduceParasitic / finishDrvrPin / …)
   ↑
LumpedCapDelayCalc (concrete; implements all pure virtuals; wire delays = 0)
   ↑
DmpCeffDelayCalc (template method: piModel → setCeffAlgorithm → gateDelaySlew → loop loadDelaySlew)   [DmpCeff.cc:1491]
   ↑
DmpCeffElmoreDelayCalc / DmpCeffTwoPoleDelayCalc (override loadDelaySlew / inputPortDelay / name / copy)
```

Our calcs live **in `lrf/`** (we don't touch the OpenSTA submodule) and slot into the same interface, so `LocalSta` keeps calling `arc_delay_calc->gateDelay(...)` polymorphically.

```
sta::LumpedCapDelayCalc
   ↑
lrf::LocalDmpDelayCalc        (Phase A: behavior-preserving wrapper of dmp_ceff_elmore)
   ↑  (Phase B sibling, not subclass)
lrf::LocalElmoreCeffDelayCalc (Phase B: closed-form Ceff + refined slew)
```

### 3.2 The three parts that change (Phase B)
1. **Parasitic reduce** — `LocalReduceParasitic.cc`. Keep `reduceToPi` and the existing post‑order DFS skeleton; **extend the full‑tree DFS** (`reduceElmoreDfs`/`reduceElmoreDfsToPt`) to also accumulate, per node: `ldelay`, `beta` (2nd moment) and `Ceff` (Algorithm 2). Note: per‑load refined slew (Eq.15) **requires** the full tree — the Pi model is collapsed and cannot provide per‑load `beta`.
2. **Driver gate Ceff** — bypass DMP's Newton. Use `Ceff[root]` from the tree pass (or, as a fallback, the Pi‑level closed form `c2 + c1·Ts/(Ts+2·rpi·c1)`) and do a **single** NLDM table lookup.
3. **Load wire‑delay/slew** — unify the real‑load (DMP root‑find) and virtual‑load (plain Elmore) paths onto **Elmore wire delay + Eq.15 refined slew**. This is exactly where `LocalDmpDelayCalc::elmoreWireDelaySlew` lives today; Phase B replaces its slew term and makes it parasitic‑dependent (needs `beta`).

### 3.3 Data structures
- `PtPiElmore` (`PtPiElmore.hh`) currently stores the Pi `(c2,rpi,c1)` + per‑load Elmore. **Extend** (don't rewrite): add driver `Ceff` and per‑load refined `slew` (and `beta` if needed at lookup time).
- The full‑tree Ceff/beta accumulation uses transient per‑node arrays during the DFS (the DFS already computes `downstreamCap`; bolt the new accumulators onto the same traversal).

### 3.4 Pipeline ordering
Implement §2.6 explicitly: total‑cap seed → Elmore+slew pass → Ceff pass → driver re‑lookup → annotate. The ordering spans reduce + gate‑delay + load‑slew, so the calc module owns the orchestration (rather than scattering it across `LocalSta`).

---

## 4. Modification plan

### Phase A — modularize today's DMP (behavior‑preserving)
**Goal:** one swappable module; identical numbers; sets up Phase B. Base class: `sta::LumpedCapDelayCalc` (per decision).

**A1 — skeleton (DONE):**
- `lrf/src/LocalDmpDelayCalc.hh` / `.cc` — wraps a `makeDmpCeffElmoreDelayCalc` instance; forwards `gateDelay` / `inputPortDelay` / `finishDrvrPin` (bit‑identical); hosts static `elmoreWireDelaySlew` (relocated from the inline virtual‑load formula). Added to `lrf/src/CMakeLists.txt`. Currently dead code (compiles, no behavior change until wired).

**A2 — wire it in:**
- Add a factory `makeLocalDelayCalc(StaState*) → new lrf::LocalDmpDelayCalc(sta)`.
- Redirect the ~13 creation sites from `sta->arcDelayCalc()->copy()` / `= sta->arcDelayCalc()` to the factory. Runtime‑critical ones: `LocalSta.cc:2606` (LRSInstanceVisitor ctor), `NetlistTransformation.cc:1604`, `IncreSta.cc:1243`; the calc is then threaded via `EvalContext.arc_delay_calc` (`NetlistTransformation.hh:46`) to `LrRebuffer`. The ~12 `TestLrf.cc` sites are mostly standalone tests (lower priority).
- Replace the inline Elmore formula at `LocalSta.cc:1337‑1346` with a call to `LocalDmpDelayCalc::elmoreWireDelaySlew(...)`.

**A3 — validate (ac97_top diff):**
```bash
cd /home/phy/PhyLS/openroad/build && make -j$(nproc)
/home/phy/test/run ac97_top 2>&1 | tee /tmp/ac97_before.log   # BEFORE A2
# … apply A2 …
/home/phy/test/run ac97_top 2>&1 | tee /tmp/ac97_after.log
diff <(grep -E 'WNS|TNS|Iter' /tmp/ac97_before.log) \
     <(grep -E 'WNS|TNS|Iter' /tmp/ac97_after.log)            # expect empty
```

### Phase B — implement ElmoreCeff (new `LocalElmoreCeffDelayCalc`)
Map to the three parts; do it stage‑by‑stage with a build + ac97_top sanity check between stages (per `CLAUDE.md` TDD). Track WNS/TNS as a *quality* check (numbers should move, ideally improve; not a bit‑diff like Phase A).

- **B1 — full‑tree Ceff pass.** Extend the reduce DFS to accumulate `Ceff[node]` via Algorithm 2; expose `Ceff[root]`. Override `gateDelay` in `LocalElmoreCeffDelayCalc` to use it + a single NLDM lookup (no Newton). Keep load slew as‑is initially.
- **B2 — refined slew.** Extend the DFS to accumulate `ldelay`/`beta`; compute per‑node slew via Eq.15 with corner‑dependent `slew_factor`. Use it for `Tn` in B1 and for load slews.
- **B3 — unify load paths.** Real + virtual loads both use Elmore wire delay + Eq.15 slew through the calc's `loadDelaySlew` (the virtual path stops being special‑cased).
- **B4 — wire & validate.** Switch the factory to `LocalElmoreCeffDelayCalc`; run ac97_top and the broader design set; compare WNS/TNS/leakage/slew‑viol against the DMP baseline and (if available) against signoff.

**Tunables to expose:** `ramp_factor` (0.8), per‑corner `slew_factor` (√(2π) / 1.0), and a switch to fall back to `LocalDmpDelayCalc` (for A/B comparison and safety).

---

## 5. Code reference map (current, as of 2026‑05)

| Concern | Location |
|---|---|
| Pi reduction (O'Brien–Savarino) | `LocalReduceParasitic.cc:58` `reduceToPi`; coeffs at `:92‑94` |
| Per‑load Elmore DFS (full tree) | `LocalReduceParasitic.cc:307` `reduceElmoreDfs`, `:358` `reduceElmoreDfsToPt` |
| Parasitic build orchestration | `LocalParasitics.cc:61` `recomputePtParasitics`, `:101` `recomputeSinglePtParasitic` |
| Pi/Elmore container | `PtPiElmore.hh` / `.cc` (extends sta `ConcretePi`+`ConcreteParasitic`) |
| Driver load‑cap + parasitic select | `LocalSta.cc:1966` `localParasiticLoad` (returns `c1+c2`) |
| Driver gate delay call | `LocalSta.cc:1233` `findDriverArcDelays` → `:1263` `gateDelay(...)` |
| Load annotate (real + virtual) | `LocalSta.cc:1301` `annotateLoadDelays`; virtual Elmore formula `:1337‑1346` |
| Delay‑calc creation sites | `LocalSta.cc:2606`, `NetlistTransformation.cc:1604`, `IncreSta.cc:1243`, `TestLrf.cc` (×~12) |
| EvalContext (threads the calc) | `NetlistTransformation.hh:46` |
| OpenSTA DMP (reference) | `sta/dcalc/DmpCeff.cc` (Newton `:344/:1294`, loads `:1519`), `DmpDelayCalc.cc` (wrapper pattern) |
| New module (Phase A) | `lrf/src/LocalDmpDelayCalc.hh` / `.cc`; `lrf/src/CMakeLists.txt` |

---

## 6. Risks & open questions
- **`reduceToPi` vs full tree.** We keep the Pi reduction for the driver‑facing model, but Algorithm 2 + Eq.15 run on the *full* parasitic tree (the DFS already walks it). Confirm the DFS visits every RC node with correct parent/branch‑R for the post‑order accumulation, and that loop resistors are handled as in `reducePiDfs`.
- **Multi‑driver nets.** `findDriverArcDelays` currently bails on `multi_drvr_net` (`LocalSta.cc:1273`). ElmoreCeff doesn't change that; keep the existing behavior.
- **Corner/rise‑fall.** Everything is per `(rf, corner)`; the existing loops already cover this — inherit them.
- **Padé validity.** Eq.9 is most accurate for small/moderate `ζ = R·C/Ts`; very large `ζ` increases error. Worth a guard / comparison vs DMP on stress nets.
- **`slew_factor` heuristic.** Corner‑dependent values are heuristic in the paper; expose as tunables and calibrate on our PDK (ASAP7).
- **Default calc assumption.** `LocalDmpDelayCalc` wraps `dmp_ceff_elmore` because the `lrf` flow never calls `set_delay_calculator`. If that changes, the wrapper must wrap the configured calc instead.

---

## 7. What is done vs pending
- **Done:** model fully worked out (Eq.9 derivation incl. why `Rd` cancels only post‑Padé; Pi superposition with unshielded near cap; Algorithm 2; Eq.15 slew; DMP‑vs‑ElmoreCeff speed analysis from source). Phase A **skeleton** (`LocalDmpDelayCalc`) created and added to the build (dead code, no behavior change).
- **Pending:** Phase A wiring (A2) + ac97_top diff (A3); Phase B (B1–B4).