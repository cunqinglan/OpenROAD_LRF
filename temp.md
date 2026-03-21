Runtime improvement:
Our target runtime was resizing a million gates in 2 hours with 4
threads, knowing the speed should scale well with more threads
to reduce runtime below an hour per million gates. As the LR sizer
neared production quality, the runtime was too slow. So, four
developer months of work went into runtime reduction.
Major contributions to the LR sizer runtime are: 75% for sizing
iterations; 11% preparing data in Nitro and sending it via GRPC to
LR sizer; 4% sizer initialization; and 2% building the timing graph
and calibrating vs. Nitro’s reference timing and RC values. Some
factors affecting the LR sizer runtime are: the number of fixed cells
such as macros, timing unconstrained, or dont_modify cells that
are not resized; the number of alternate libcells considered pcell; the number of timing arcs and nodes in the timing graph,
which is significantly worse in post-CTS and with more timing
corners enabled. The libcell resize evaluation is more expensive if
there are more tags per pin as the local timing graph is bigger.
We achieved an average 1.8 hour runtime to resize a million
gates with 4 threads on the pre-CTS test suite, including some
runtime outliers. Decent multi-threaded scaling was achieved per
the average speedups on about 100 designs in the pre-CTS test
suite shown in Table 1. There’s about a 10% single-threaded
runtime overhead limiting parallelization speedup by Amdahl’s
law. With 16 threads, the LR sizer averages 0.8 hours per million
gates. However, runtime averages 5 hours per million gates with
4 threads on the post-CTS test suite. Post-CTS usage was not our
main focus. These runtimes are on a variety of the computing
grid’s servers, for example a 16-core 2.6 GHz Xeon E5-2667.
Table 2 summarizes techniques that gave a 3x speedup, and
reducing memory helped as it was excessive. E.g., not passing
names of pins, cells and nets reduced memory usage by 3% and
gave 2% speedup. Memory usage per million cells averaged 18GB
in pre-CTS and 40GB in post-cts for the LR sizer including data
preparation in Nitro. Some of the speedups are detailed below.
1. History-based adaptive libcell pruning: As LR iterations
advance, the difference in optimal cell choice decreases, so we can
check just a small range around the current size. For every cell, its
alternate libcells are evaluated for LRS cost and change in local
slack, then put in ascending cost order. The order is first computed
after iteration K, the first iteration where the percent of cells that
change sizes falls below 10%. Pruning is first enabled at iteration
K + 1, reducing libcell choices to the first P = 20% of the libcells,
minimum of 2. Pruning is enabled for the next M − 1 iterations.
On the K + M iteration, the order is recomputed, and so forth.
M is initialized to 3, and then adapted on a per-cell basis during
every reordering iteration. The value of M depends on the amount
of jump in the optimal libcell with respect to the new ordering.
For example, consider a cell during a reordering iteration K.
Suppose the new order of libcells has been computed. In the new
ordering, the optimal libcell is at index 0. Suppose the previous
optimal libcell (from iteration K − 1) appears at index 5 in the new
order. That means the optimal libcell has jumped 5 libcells. If the
jump is not more than P for a cell, it is assumed that convergence
holds for that cell and the set of libcells keep shrinking. Therefore,
reordering can be less frequent and M is set to M + 1. If the jump
has increased, M is set to M − 1 to reorder more frequently.
2. Lagrange-multiplier-based sibling-arc skipping: Delay
is computed for each local arc to calculate a libcell’s LRS cost. The
timing impact is first-order for fanin-arcs, arcs of the resized cell,
and fanout-arcs. The impact is second-order for sibling-arcs. Arcs
with smaller Lagrange multipliers add little LRS cost and are the
least timing critical, so they also contribute little to the local slack
change. To reduce the libcell evaluation runtime, such sibling-arcs
can be ignored, without affecting the choice of the optimal libcell.
Before a cell is sized, we sum Lagrange multipliers for every
local-arc. For the sibling-arcs which contribute less than 1% to the
multiplier sum, they are skipped from the set of local-arcs to save
runtime. This 1% threshold was chosen experimentally as
providing the best runtime savings without degrading results.