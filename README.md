# Packing

Boundary-aware macro (rectangle) packing utilities.

Given a set of macros and a bounding box, place the macros so they pack tightly
toward chosen boundary edges; or, given just a set of macros, generate the set of
bounding-box shapes `(width, height)` into which they pack at a target utilization.

This is a lightweight, dependency-light building block originally factored out of an
OpenROAD-style macro-clustering / floorplanning flow. It is the *heterogeneous*
analogue of OpenROAD's `calculateMacroTilings`: instead of a closed-form `cols × rows`
factorization (which only works for identical macros), it uses a real packer to
**certify** feasibility, so it handles mixed-size macro groups.

## Design choices

The packer is intentionally simple and deterministic:

| Property | Behavior |
|----------|----------|
| Orientation | **Fixed** — macros are never rotated; their `(w, h)` are used as-is |
| Spacing | **Zero** — tight abutment, no channels/halo |
| Algorithm | **Edge-anchored Skyline-Bottom-Left**, growing inward from the aligned corner |
| Effort | **Single greedy pass**, macros sorted largest-area-first (configurable) |
| Coordinates | placement `(x, y)` is the macro's **lower-left** corner; box is `(llx, lly, urx, ury)`; `+x` right, `+y` up |

Feasibility is checked **before** packing:
1. **Dimensional** — every macro must fit the box (`w ≤ box_w` and `h ≤ box_h`); a
   violation is hard-infeasible.
2. **Utilization** — `total_macro_area / box_area`. If `> 0.95` a warning
   *"unlikely to pack"* is emitted (packing is still attempted).

## Install

Pure Python; the core has **no third-party dependencies**. `matplotlib` is needed
only for the optional plotting helpers.

```bash
pip install matplotlib          # optional, for plot_packing()
```

Python ≥ 3.7 (uses `dataclasses`).

## Data model

```python
Macro(name: str, w: float, h: float)                 # .area property
Placement(name, x, y, w, h)                           # x,y = lower-left
PackResult(placements, unplaced, utilization,         # see below
           feasible, warnings)                         # .num_placed, .summary()
BBoxOption(aspect, w, h, util, packs)                 # one shape-curve point
```

`PackResult`:
- `placements: List[Placement]` — every macro that was placed
- `unplaced: List[(name, reason)]` — macros that could not be placed
- `utilization: float` — `total_macro_area / box_area`
- `feasible: bool` — dimensional feasibility of **all** macros
- `warnings: List[str]`
- `.num_placed`, `.summary()`

## API

### 1. `pack_macros(macros, box, aligned_sides=("left","top"), sort_key="area", util_warn=0.95, logger=print) -> PackResult`

Pack `macros` into a fixed `box`, hugging `aligned_sides`.

- `box` — `(llx, lly, urx, ury)`
- `aligned_sides` — 1 or 2 **adjacent** of `{"left","right","top","bottom"}`. Two
  adjacent sides anchor the pack at the corner where they meet; macros grow inward.
- `sort_key` — `"area"` (default), `"height"`, `"width"`, or `"maxdim"`.

```python
from packing import Macro, pack_macros

macros = [Macro(f"m{i}", 60.61, 169.4) for i in range(16)]
res = pack_macros(macros, box=(0, 0, 250, 780), aligned_sides=("left", "top"))
print(res.summary())
for p in res.placements:
    print(p.name, p.x, p.y)
```

### 2. `find_packable_bboxes(macros, min_util=0.80, ar_min=0.25, ar_max=4.0, n_ar=13, sort_key="area", aligned_sides=("left","bottom"), logger=print) -> List[BBoxOption]`

Given a macro group, return a **set of `(width, height)` boxes** — one per aspect
ratio sampled geometrically over `[ar_min, ar_max]` (aspect = `w/h`). For each ratio
it binary-searches the **tightest** box (maximum utilization ≥ `min_util`) that still
packs **all** macros. Ratios that cannot reach `min_util` are returned with
`packs=False`, so you can read off the feasible aspect-ratio window.

This is the "shape curve" / tiling-set generator.

```python
from packing import Macro, find_packable_bboxes

macros = [Macro(f"bank{i}", 60.61, 169.4) for i in range(16)] + \
         [Macro(f"ic{i}", 56.05, 102.2) for i in range(4)]
opts = find_packable_bboxes(macros, min_util=0.80, ar_min=0.25, ar_max=4.0)
for o in opts:
    if o.packs:
        print(f"AR={o.aspect:.2f}  {o.w:.1f} x {o.h:.1f}  util={o.util*100:.1f}%")
```

### 3. `pack_in_best_box(macros, aligned_sides=("left","top"), min_util=0.80, ar_min=0.25, ar_max=4.0, n_ar=13, sort_key="area", logger=print) -> (PackResult, box, BBoxOption, edge_fill)`

Convenience workflow: pick the **tightest packable box** (max utilization over the
aspect sweep), then pack into it hugging `aligned_sides`, and report per-edge fill.
Use this when you want the box sized to the macros (which both maximizes utilization
*and* fills the aligned boundaries).

```python
from packing import Macro, pack_in_best_box

res, box, best, ef = pack_in_best_box(macros, aligned_sides=("left", "top"))
# ef -> {"left": 1.0, "top": 0.99}
```

### 4. `edge_fill(res, box, aligned_sides) -> dict`

Fraction of each aligned edge's length covered by macros flush with it. Useful to
verify that the chosen boundaries are actually filled.

### 5. `plot_packing(res, box, aligned_sides, path, title=None)`

Render a packing result (aligned edges highlighted) to an image. Requires matplotlib.

## Quick demo

```bash
python packing.py                 # runs the built-in demo, writes out_*.png
python examples/tile_demo.py      # tile macros -> shape curve + tightest-box pack
```

## Notes & limitations

- The packer is a **single-pass heuristic** (Skyline-Bottom-Left). It will not always
  achieve the theoretical optimum; near very high utilization (square boxes especially)
  it may leave a few macros unplaced. Prefer `find_packable_bboxes` / `pack_in_best_box`
  to discover a box shape the macros actually fit.
- Fixed orientation and zero spacing are deliberate. If you need rotation or routing
  channels, that is a future extension (see `CLAUDE.md`).
- All lengths are in whatever unit you pass in (microns in the originating flow).

## License

BSD 3-Clause. See `LICENSE`.
