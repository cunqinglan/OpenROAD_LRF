# PLAN: ML-based GPL → GR Parasitic / Timing Prediction for LR

**Status**: proposed, not started
**Owner**: Cunqing
**Last updated**: 2026-04-24

Reference paper: Chhabria et al., *From Global Route to Detailed Route: ML for
Fast and Accurate Wire Parasitics and Timing Prediction*, MLCAD '22.
Local copy: `/home/culan/Desktop/workspace/report/Chhabria_MLCAD2022_GR_to_DR_Parasitics.pdf`

## 1. Motivation

LR currently runs after GPL / CTS / legalize but **before** GRT. Parasitics
come from FLUTE Steiner trees + layer-averaged per-unit R/C, which is
optimistic/pessimistic in unpredictable ways. To survive the
`placement-RC → GR-RC` transition, `LrConfig` carries two heuristic margins:

- `slew_margin = 0.10` (10% slew headroom, [LrConfig.hh:110](include/lrf/LrConfig.hh#L110))
- `timing_margin = 0.01` (absolute slack headroom, [LrConfig.hh:119](include/lrf/LrConfig.hh#L119))

Both are blunt, global knobs. The paper shows that fixing the analogous GR→DR
discrepancy nets 0.1–0.2 ns WNS and large TNS improvements. Doing the
equivalent GPL→GR correction should let us **shrink or remove these margins
without regressing post-GR timing**, which is the real ROI.

## 2. What differs from the paper

| | Paper (MLCAD '22) | This plan |
|---|---|---|
| Input stage | post-GR (route guides available) | post-GPL (placement only) |
| Predict target | post-DR R/C1/C2, sink delay, sink slew | **post-GR** R/C1/C2, sink delay, sink slew |
| Strongest features | source-sink length, per-segment layer R/C (from guides) | **unavailable** — must use FLUTE topology + layer-average R/C as proxy |
| Injection point | OpenROAD post-GR optimizer via OpenSTA Tcl API | `LocalParasitics` / `LocalSta` inside LR |

Paper §5 explicitly notes post-GR prediction beats placement-based prediction
because guides contribute source-sink RC. We are giving that up, so expect a
**lower accuracy ceiling** (rough target: net MAPE < 15%; paper achieved ~6%
GR→DR). A feasibility check (§6) gates full investment.

## 3. Scope

**In scope**
- Three XGBoost models: `π(R, C1, C2)` per net, `wire_delay` per sink,
  `wire_slew` per sink.
- Data pipeline on MLCAD ASAP7 designs.
- Optional integration into LR behind a feature flag.
- Evaluation on MLCAD benchmarks with `run_test.sh -c buffering`.

**Out of scope**
- Replacing GRT itself.
- Predicting post-DR (could be a follow-up once GPL→GR works).
- Non-XGBoost architectures (GNN, transformer) — revisit only if XGBoost
  plateaus.

## 4. Phases

### Phase 1 — Data collection (1–2 weeks)
New script: `test_lrf/collect_gpl2gr_data.py`.
For each MLCAD design × {3 utilizations}:

1. Run flow to post-GPL+CTS+legalize. Dump `pre_lr.odb`.
2. On that ODB, extract **GPL features** (no GRT):
   - Per net: HPWL, #sinks, driver slew (from current placement-STA), RUDY /
     density mean & std over net bbox, FLUTE Steiner tree length, rise/fall
     flag.
   - Per sink: source→sink FLUTE path length, source→sink R/C using layer
     averages (proxy for the paper's per-segment R/C).
3. On the same ODB run GRT + `estimate_parasitics -global_routing`, dump
   SPEF + STA. Extract **labels**:
   - Per net: reduced π model (R, C1, C2) from SPEF.
   - Per sink: post-GR wire delay, post-GR wire slew.
4. Write parquet/csv to `test_lrf/ml_datasets/gpl2gr/<design>_<util>.parquet`.

Target volume: 8 MLCAD designs × 3 utilizations ≈ O(10⁵) sink samples.

### Phase 2 — Offline training (1 week)
- Location: `test_lrf/ml_models/gpl2gr/` (NOT in OpenROAD tree).
- XGBoost hyperparameters from the paper: `lr=0.01, max_depth=4,
  n_estimators=900, subsample=0.8, loss=RMSE`. Don't tune until baseline lands.
- Leave-one-design-out CV. Metrics:
  - Net/sink level MAPE (paper Table 2 equivalent)
  - **Path-level slack error** (paper Table 3 equivalent) — the LR-relevant metric
- Export model as XGBoost JSON (`model.ubj`) for C++ loading.

### Phase 3 — LR integration (2–3 weeks)
New class: `MlParasiticPredictor` (`include/lrf/MlParasiticPredictor.hh` +
`src/MlParasiticPredictor.cc`).
- Loads XGBoost model via xgboost C API (gate with `-DUSE_ML_PARASITICS=ON`
  optional dep; do not force on all builds).
- API:
  ```cpp
  PiModel predictPi(const Net*);          // (R, C1, C2)
  float   predictSinkDelay(Pin* drv, Pin* sink);
  float   predictSinkSlew (Pin* drv, Pin* sink);
  ```
- Feature extraction helper reuses FLUTE + `PlacementDensityMap`
  ([src/PlacementDensityMap.cc](src/PlacementDensityMap.cc)) for RUDY/density.

Injection points:
- [src/LocalParasitics.cc](src/LocalParasitics.cc): when constructing the π
  model, replace (or weighted-blend) FLUTE-derived R/C1/C2 with predictions.
- [src/LocalSta.cc](src/LocalSta.cc) (or [src/PtPiElmore.cc](src/PtPiElmore.cc)):
  annotate wire delay / slew with predictions during incremental update.
- LrConfig knobs:
  ```cpp
  bool use_ml_parasitics = false;
  std::string ml_model_dir;
  float ml_blend_alpha = 1.0f;   // 1.0 = pure ML, 0.0 = pure FLUTE
  ```

### Phase 4 — Validation (1 week)
1. **Accuracy plots**: scatter (current FLUTE est, ML pred, GR ground truth)
   for one held-out design — paper Fig 8 / Fig 9 equivalents.
2. **MLCAD end-to-end**: `./run_test.sh -c buffering -v ml_v1 -d <all>`.
   Compare WNS / TNS / slew violations / cap violations against current best.
   Do **not** rank by MLCAD score (leakage-dominated — see
   `feedback_no_score_leakage.md`).
3. **Margin ablation** (the ROI argument):
   - baseline: `slew_margin=0.10, timing_margin=0.01`, no ML
   - `slew_margin=0.05, timing_margin=0`, ML on
   - `slew_margin=0.00, timing_margin=0`, ML on
   Success = ML runs can drop margins without post-GR regressions.
4. **Runtime**: ML inference must stay well under current LR visit cost
   (writeBack is already 74–88% of a visit per prior profiling — ML must be
   µs/net or lower).

## 5. Key risks

1. **Accuracy ceiling from missing route guides.** No source-sink RC from
   real layer assignment. Mitigation: gate on §6 feasibility; if MAPE > 25%,
   revisit features (GCell-level RUDY, density gradient, per-layer FLUTE
   assignment heuristic, etc.) before training full pipeline.
2. **Chicken-and-egg on driver slew.** `driver_slew` feature comes from the
   current (placement-RC) STA, so predictions correct what fed them.
   Mitigation: single-shot first; if residual is systematic, iterate
   `predict → re-STA → predict` 1–2 times to fixed point.
3. **Build complexity.** xgboost C++ dep in OpenROAD build. Mitigation:
   optional CMake flag; ship CPU-only; no GPU dep.
4. **Domain shift MLCAD → ICCAD.** Train on MLCAD ASAP7, but we also use
   ICCAD benchmarks. Plan: collect ICCAD data too in Phase 1 and train
   jointly, or train per-technology.

## 6. Feasibility gate (do this FIRST)

Before committing to Phase 1 full pipeline, on `ac97_top` only:
- Run through to post-GPL+CTS+legalize.
- Snapshot FLUTE-based per-net wire delay estimate.
- Run GRT + STA, snapshot post-GR per-net wire delay.
- Scatter-plot `(FLUTE_delay, GR_delay)` — analog of paper Fig 2 but
  placement→GR.

**Go criterion**: best-fit slope `k` in `y = kx` deviates from 1.0 by > 0.3,
or per-net MAPE > 20%. If the gap is smaller, heuristic margins are already
close to optimal and this project isn't worth the complexity.

## 7. File manifest (when implemented)

```
src/lrf/
  include/lrf/
    MlParasiticPredictor.hh           # new
    LrConfig.hh                       # +3 knobs
  src/
    MlParasiticPredictor.cc           # new
    LocalParasitics.cc                # integration hook
    LocalSta.cc  or  PtPiElmore.cc    # integration hook

test_lrf/
  collect_gpl2gr_data.py              # new, Phase 1
  ml_models/gpl2gr/
    train.py                          # new, Phase 2
    model.ubj                         # artifact, Phase 2
  ml_datasets/gpl2gr/*.parquet        # artifact, Phase 1
  feasibility_gate.py                 # new, §6
```

## 8. Open questions

- Use placement-stage STA's driver slew, or re-derive with a simple Elmore
  pass over the FLUTE tree? (affects Phase 1 feature extraction)
- Per-technology model or single joint (ASAP7 / NanGate45 / SkyWater130)
  model? Paper trained per-tech — start there.
- Blend vs. replace? `ml_blend_alpha=1.0` (pure ML) is simplest; blending may
  help near the decision boundary. Default to 1.0, expose the knob.
