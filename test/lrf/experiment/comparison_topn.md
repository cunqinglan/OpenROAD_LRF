# Buffering top_n Comparison: 10000 vs 100

**Design:** ac97_top (MLCAD, ASAP7)
**Date:** 2026-03-18
**Branch:** develop_tunbuffer
**Threads:** 10
**Buffering iterations per pin:** 3 (2 coarse + 1 precise)

## Configuration

| Parameter | v3 | v4 |
|-----------|----|----|
| top_n | 10000 (effective: 3460) | 100 |
| Log | v3/ac97_top.log | v4/ac97_top.log |

Note: ac97_top only has ~3460 negative-slack gates, so top_n=10000 selects all of them.

## Results

| Metric | top_n=10000 | top_n=100 | Delta |
|--------|-------------|-----------|-------|
| Total time | 447s | 414s | -7% |
| Buffering time/round | 7-8s | 1.7-2.4s | **~3x faster** |
| Total buffers inserted | 384 | 40 | **~10x fewer** |
| Final WNS | -43.7ps | -39.7ps | +4.0ps (better) |
| Final TNS | -16933ps | -19133ps | -2200ps (13% worse) |
| Initial WNS | -154.8ps | -154.8ps | same |
| Initial TNS | -44435ps | -44435ps | same |

## Per-round Buffering Summary

### top_n=10000 (v3)
| Round | TNS (ps) | WNS (ps) | Buffering time (s) |
|-------|----------|----------|---------------------|
| 1 | -16182 | -51.0 | 6.7 |
| 2 | -16432 | -43.4 | 6.7 |
| 3 | -16226 | -51.1 | 7.6 |
| 4 | -17261 | -42.3 | 7.8 |
| 5 | -19162 | -47.9 | 8.6 |
| 6 | -17335 | -42.3 | 7.5 |
| 7 | -16933 | -43.7 | 8.1 |

### top_n=100 (v4)
| Round | TNS (ps) | WNS (ps) | Buffering time (s) |
|-------|----------|----------|---------------------|
| 1 | -15635 | -41.1 | 2.1 |
| 2 | -15831 | -40.2 | 2.2 |
| 3 | -16452 | -40.0 | 2.4 |
| 4 | -17628 | -41.3 | 2.4 |
| 5 | -18913 | -39.9 | 2.4 |
| 6 | -19086 | -39.7 | 1.7 |
| 7 | -19427 | -43.0 | 1.7 |
| 8 | -19133 | -39.7 | 1.7 |

## Profiling Breakdown (per-thread average, top_n=10000)

| Phase | Time (s) | % of total |
|-------|----------|------------|
| rebuffer_total | ~5.0 | 100% |
| rebuffer_precise (iter 2) | ~4.3 | **86%** |
| rebuffer_coarse (iter 0+1) | ~0.23 | 5% |
| rebuffer_setup | ~0.30 | 6% |

## Conclusions

1. **Precise evaluation dominates runtime** — 86% of per-pin rebuffer time is spent on the last iteration's precise evaluation (virtual buffer + synthetic parasitics + local STA).

2. **top_n=100 is 3x faster per buffering round** but total speedup is only 7% because resize dominates overall runtime.

3. **WNS is slightly better with top_n=100** (-39.7 vs -43.7ps), likely because fewer buffer insertions means less interference between decisions.

4. **TNS is 13% worse with top_n=100** (-19133 vs -16933ps) — 3300+ gates with negative slack miss their buffering opportunity, only 40 buffers inserted vs 384.

5. **Buffer insertion efficiency**: top_n=10000 inserts ~55 buffers/round from 3460 candidates (1.6% hit rate); top_n=100 inserts ~5 buffers/round from 100 candidates (5% hit rate). The top candidates by Cout/Cin have higher buffer acceptance rates.
