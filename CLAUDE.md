# Packing — AI agent context

Project-specific guidance for AI agent sessions working on this repository.

## What this is

A small, self-contained Python library for **boundary-aware rectangle (macro)
packing** and **packable-bounding-box (shape-curve) generation**. It was factored out
of an OpenROAD-style macro clustering / floorplanning flow and is the heterogeneous
analogue of OpenROAD's `calculateMacroTilings` — it certifies footprint feasibility
with a real packer instead of a closed-form factorization.

There is a sibling project (not in this repo) that does the **clustering**; this repo
is intentionally standalone so the packer can be reused on its own.

## Layout

```
packing.py            # the entire library (single module) + a __main__ demo
examples/             # runnable usage examples
README.md             # Python API documentation (user-facing)
CLAUDE.md             # this file
LICENSE               # BSD 3-Clause
requirements.txt      # matplotlib (optional, plotting only)
```

## Public API (keep stable)

- `Macro(name, w, h)`, `Placement`, `PackResult`, `BBoxOption` — dataclasses.
- `pack_macros(macros, box, aligned_sides, sort_key, util_warn, logger)` — pack into a
  fixed box hugging given edges.
- `find_packable_bboxes(macros, min_util, ar_min, ar_max, n_ar, ...)` — the (w,h)
  shape-curve set at ≥ min_util utilization across an aspect-ratio sweep.
- `pack_in_best_box(...)` — find tightest box then pack into it; reports edge fill.
- `edge_fill(...)`, `plot_packing(...)` — diagnostics / visualization.

If you change a signature, update README.md in the same commit.

## Invariants — do NOT change without explicit request

These reflect deliberate design decisions:

1. **Fixed orientation** — macros are never rotated. Along-edge vs inward dimension is
   chosen by an axis transform (`_frame` / `_to_real`), not by rotating the macro.
2. **Zero spacing** — tight abutment, no halo/channel.
3. **Skyline-Bottom-Left, single greedy pass**, macros sorted largest-area-first by
   default. `_skyline_place` / `_skyline_add` implement the contour update.
4. **Coordinate convention** — placement `(x, y)` is lower-left; box is
   `(llx, lly, urx, ury)`; `+x` right, `+y` up.
5. **Feasibility before packing** — dimensional check (hard) + utilization > 0.95
   warning ("unlikely to pack"). The 0.95 threshold is `UTIL_WARN_THRESHOLD`.
6. **`aligned_sides`** accepts 1 or 2 *adjacent* sides; two adjacent sides anchor at
   their shared corner. Opposite-side pairs are unsupported by design.

## Common tasks

- **Run the demo / smoke test:** `python packing.py` (writes `out_*.png`).
- **Validate a change:** packing must (a) never place overlapping macros, (b) keep all
  placements inside the box, (c) report every non-placed macro in `unplaced`. A good
  check is to assert no pairwise overlap and `0 <= x, x+w <= box_w` for every placement.
- **Monotonicity:** `find_packable_bboxes` relies on packing feasibility being monotone
  in box size (bigger box ⇒ easier). Keep that property if you alter the packer.

## Conventions

- Pure standard library for the core; `matplotlib` only inside plotting functions
  (import locally so the library works without it).
- Keep it a single module unless there's a strong reason to split.
- Lengths are unit-agnostic (microns in the originating flow).
- Git: commit with `-s` (DCO sign-off); author is the repo owner. Branch is `main`.

## Possible future extensions (only if asked)

- Optional 90° rotation.
- Min spacing / halo between macros and to the box edge.
- Multi-pass / best-of-N sort orders for higher utilization.
- Wiring into the clustering flow so each HardMacro cluster gets a shape curve.
