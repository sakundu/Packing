"""Boundary-aware macro packer.

Given a set of macros, a bounding box, and which side(s) of the box the macros
should hug, place the macros (lower-left coordinates) so they pack tightly toward
the aligned boundaries, improving utilization of those edges.

Design (per user spec):
  * Fixed orientation     -- macros are NOT rotated; their (w, h) are used as-is.
  * Zero spacing          -- tight abutment, no channels/halo.
  * Edge-anchored skyline  -- macros are anchored at the corner where the aligned
                             edges meet and packed with a Skyline-Bottom-Left
                             heuristic that grows inward, so they hug the aligned
                             boundaries and fill the area compactly.
  * Single greedy pass    -- one deterministic pass, macros sorted largest-area
                             first (configurable).

Feasibility is checked BEFORE packing:
  1. Dimensional: every macro must fit the box (w <= box_w and h <= box_h).
  2. Utilization: total macro area / box area. If > 0.95 -> warn "unlikely to pack"
     (we still attempt). Dimensional failures are hard-infeasible.

The packer reports the placement of every macro it could place and the list of
macros it could not place (with the reason).

Coordinate convention: a placement's (x, y) is the macro's lower-left corner;
the box is (llx, lly, urx, ury); +x right, +y up.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Sequence

UTIL_WARN_THRESHOLD = 0.95


@dataclass
class Macro:
    name: str
    w: float
    h: float

    @property
    def area(self) -> float:
        return self.w * self.h


@dataclass
class Placement:
    name: str
    x: float          # lower-left
    y: float
    w: float
    h: float


@dataclass
class PackResult:
    placements: List[Placement] = field(default_factory=list)
    unplaced: List[Tuple[str, str]] = field(default_factory=list)  # (name, reason)
    utilization: float = 0.0           # total macro area / box area
    feasible: bool = True              # dimensional feasibility of ALL macros
    warnings: List[str] = field(default_factory=list)

    @property
    def num_placed(self) -> int:
        return len(self.placements)

    def summary(self) -> str:
        lines = [
            f"placed={self.num_placed}  unplaced={len(self.unplaced)}  "
            f"utilization={self.utilization*100:.1f}%  feasible={self.feasible}",
        ]
        for w in self.warnings:
            lines.append(f"  WARNING: {w}")
        for name, reason in self.unplaced:
            lines.append(f"  UNPLACED {name}: {reason}")
        return "\n".join(lines)


# ----------------------------------------------------------------------------
# Anchor / axis configuration for the aligned sides.
# For each side-set we define the anchor corner (origin of the normalized frame),
# the "along-edge" unit direction (+u) and the "inward" unit direction (+v).
# Macros are filled from the anchor corner along +u, growing inward along +v, so
# they hug the aligned edge(s).
# ----------------------------------------------------------------------------
def _frame(aligned_sides: Sequence[str], box) -> dict:
    llx, lly, urx, ury = box
    s = frozenset(side.lower() for side in aligned_sides)
    cfg = {
        # two adjacent sides -> hug the corner where they meet
        frozenset({"left", "top"}):    ((llx, ury), (1, 0), (0, -1)),
        frozenset({"left", "bottom"}): ((llx, lly), (1, 0), (0, 1)),
        frozenset({"right", "top"}):   ((urx, ury), (-1, 0), (0, -1)),
        frozenset({"right", "bottom"}):((urx, lly), (-1, 0), (0, 1)),
        # single side -> shelf runs parallel to it, grows inward (perpendicular)
        frozenset({"top"}):    ((llx, ury), (1, 0), (0, -1)),
        frozenset({"bottom"}): ((llx, lly), (1, 0), (0, 1)),
        frozenset({"left"}):   ((llx, lly), (0, 1), (1, 0)),
        frozenset({"right"}):  ((urx, lly), (0, 1), (-1, 0)),
    }
    if s not in cfg:
        raise ValueError(f"Unsupported aligned_sides {set(aligned_sides)}; "
                         f"use 1 or 2 adjacent of left/right/top/bottom.")
    origin, along, inward = cfg[s]
    horizontal_primary = along[0] != 0   # along edge is the x axis
    return {
        "origin": origin, "along": along, "inward": inward,
        "horizontal_primary": horizontal_primary,
        "box_w": urx - llx, "box_h": ury - lly,
    }


def _to_real(frame, u, v):
    """Map a normalized point (u along edge, v inward) to real (x, y)."""
    ox, oy = frame["origin"]
    ax, ay = frame["along"]
    ix, iy = frame["inward"]
    return ox + u * ax + v * ix, oy + u * ay + v * iy


# ----------------------------------------------------------------------------
# Skyline-Bottom-Left packing in the normalized frame.
# Normalized frame: u in [0, W] along edge, v in [0, D] inward.
# Skyline = list of [u_left, width, height] segments (height = filled depth v).
# ----------------------------------------------------------------------------
def _skyline_place(skyline, a, b, W, D):
    """Find lowest-v position for a rect of along-size a, inward-size b.

    Returns (u, v) lower-left in normalized frame, or None if it doesn't fit.
    """
    best = None  # (resulting_top, u, v)
    n = len(skyline)
    for i in range(n):
        u = skyline[i][0]
        if u + a > W + 1e-9:
            continue
        # max height over the span [u, u+a]
        span_end = u + a
        v = 0.0
        j = i
        acc = 0.0
        while j < n and acc < a - 1e-9:
            seg_u, seg_w, seg_h = skyline[j]
            if seg_h > v:
                v = seg_h
            acc += seg_w
            j += 1
        if v + b > D + 1e-9:
            continue
        top = v + b
        cand = (top, u, v)
        if best is None or cand < best:
            best = cand
    if best is None:
        return None
    return best[1], best[2]


def _skyline_add(skyline, u, v, a, b):
    """Insert a rect [u, u+a] x [v, v+b]: raise skyline to v+b over [u, u+a]."""
    new_top = v + b
    result = []
    placed = False
    x = 0.0
    for seg_u, seg_w, seg_h in skyline:
        seg_end = seg_u + seg_w
        # Part of this segment entirely left of the rect
        if seg_end <= u + 1e-9:
            result.append([seg_u, seg_w, seg_h])
            continue
        # Part entirely right of the rect
        if seg_u >= u + a - 1e-9:
            if not placed:
                result.append([u, a, new_top])
                placed = True
            result.append([seg_u, seg_w, seg_h])
            continue
        # Segment overlaps the rect span -> split
        left_w = u - seg_u
        if left_w > 1e-9:
            result.append([seg_u, left_w, seg_h])
        if not placed:
            result.append([u, a, new_top])
            placed = True
        right_start = u + a
        if seg_end - right_start > 1e-9:
            result.append([right_start, seg_end - right_start, seg_h])
    if not placed:
        result.append([u, a, new_top])
    # merge adjacent segments with equal height
    merged = []
    for seg in result:
        if merged and abs(merged[-1][2] - seg[2]) < 1e-9:
            merged[-1][1] += seg[1]
        else:
            merged.append(seg)
    return merged


# ----------------------------------------------------------------------------
# Public API
# ----------------------------------------------------------------------------
def pack_macros(macros: Sequence[Macro],
                box: Tuple[float, float, float, float],
                aligned_sides: Sequence[str] = ("left", "top"),
                sort_key: str = "area",
                util_warn: float = UTIL_WARN_THRESHOLD,
                logger=print) -> PackResult:
    """Pack `macros` into `box`, hugging `aligned_sides`.

    macros        : list of Macro(name, w, h)
    box           : (llx, lly, urx, ury)
    aligned_sides : 1 or 2 adjacent of {'left','right','top','bottom'}
    sort_key      : 'area' (default), 'height', 'width', or 'maxdim'
    """
    llx, lly, urx, ury = box
    box_w = urx - llx
    box_h = ury - lly
    box_area = box_w * box_h
    res = PackResult()

    if box_w <= 0 or box_h <= 0:
        res.feasible = False
        res.warnings.append(f"degenerate box (w={box_w}, h={box_h})")
        logger(res.summary())
        return res

    total_macro_area = sum(m.area for m in macros)
    res.utilization = (total_macro_area / box_area) if box_area > 0 else float("inf")

    # ---- feasibility check 1: dimensional ----
    too_big = [m for m in macros if m.w > box_w + 1e-9 or m.h > box_h + 1e-9]
    if too_big:
        res.feasible = False
        for m in too_big:
            res.warnings.append(
                f"macro {m.name} ({m.w}x{m.h}) exceeds box ({box_w:.2f}x{box_h:.2f})")

    # ---- feasibility check 2: utilization ----
    if res.utilization > util_warn:
        res.warnings.append(
            f"utilization {res.utilization*100:.1f}% > {util_warn*100:.0f}% "
            f"-- unlikely to pack")

    frame = _frame(aligned_sides, box)
    horizontal = frame["horizontal_primary"]
    W = frame["box_w"] if horizontal else frame["box_h"]   # along-edge extent
    D = frame["box_h"] if horizontal else frame["box_w"]   # inward extent

    # ---- sort macros (single greedy pass) ----
    keyfn = {
        "area": lambda m: -m.area,
        "height": lambda m: -m.h,
        "width": lambda m: -m.w,
        "maxdim": lambda m: -max(m.w, m.h),
    }.get(sort_key, lambda m: -m.area)
    order = sorted(macros, key=keyfn)

    # ---- skyline pack in the normalized frame ----
    skyline = [[0.0, W, 0.0]]
    for m in order:
        # along-edge size a, inward size b (fixed orientation, just axis mapping)
        if horizontal:
            a, b = m.w, m.h
        else:
            a, b = m.h, m.w
        if a > W + 1e-9 or b > D + 1e-9:
            res.unplaced.append((m.name, "macro larger than box dimension"))
            continue
        pos = _skyline_place(skyline, a, b, W, D)
        if pos is None:
            res.unplaced.append((m.name, "no free region in box"))
            continue
        u, v = pos
        skyline = _skyline_add(skyline, u, v, a, b)
        # map normalized rect corners to real, take lower-left
        corners = [_to_real(frame, u, v), _to_real(frame, u + a, v),
                   _to_real(frame, u, v + b), _to_real(frame, u + a, v + b)]
        rx = min(c[0] for c in corners)
        ry = min(c[1] for c in corners)
        res.placements.append(Placement(m.name, rx, ry, m.w, m.h))

    logger(res.summary())
    return res


# ----------------------------------------------------------------------------
# Shape generation: for a group of macros, find bounding boxes (w, h) into which
# they pack with utilization >= min_util, sweeping aspect ratio in [ar_min, ar_max].
# This is the heterogeneous analogue of OpenROAD's calculateMacroTilings, using the
# real packer to certify feasibility instead of a closed-form factorization.
# ----------------------------------------------------------------------------
import math


@dataclass
class BBoxOption:
    aspect: float        # w / h
    w: float
    h: float
    util: float          # macro_area / (w*h) actually achieved
    packs: bool          # all macros placed at this (w, h)


def find_packable_bboxes(macros: Sequence[Macro],
                         min_util: float = 0.80,
                         ar_min: float = 0.25,
                         ar_max: float = 4.0,
                         n_ar: int = 13,
                         sort_key: str = "area",
                         aligned_sides: Sequence[str] = ("left", "bottom"),
                         logger=print) -> List[BBoxOption]:
    """Return a set of (w, h) boxes, one per sampled aspect ratio, into which all
    `macros` pack with utilization >= `min_util`.

    For each aspect ratio r = w/h sampled geometrically in [ar_min, ar_max]:
      * box area = macro_area / U  ->  w = sqrt(area*r), h = sqrt(area/r)
      * packing feasibility is monotone in box size (bigger box packs more easily),
        so we binary-search the MAX utilization U in [min_util, ~0.98] at which the
        packer still places every macro, and report that tightest box.
      * aspect ratios whose box can't even hold the largest macro, or that fail to
        pack at min_util, are returned with packs=False (so you can see the
        feasible aspect-ratio window).
    """
    A = sum(m.area for m in macros)
    wmax = max(m.w for m in macros)
    hmax = max(m.h for m in macros)

    def box_for(r, U):
        area = A / U
        return math.sqrt(area * r), math.sqrt(area / r)

    def packs(r, U):
        w, h = box_for(r, U)
        if w < wmax - 1e-9 or h < hmax - 1e-9:
            return False, w, h
        res = pack_macros(macros, (0.0, 0.0, w, h), aligned_sides=aligned_sides,
                          sort_key=sort_key, logger=lambda *a, **k: None)
        return (len(res.unplaced) == 0), w, h

    # geometric sweep so ratio and 1/ratio are symmetric
    if n_ar < 2:
        ratios = [1.0]
    else:
        ratios = [ar_min * (ar_max / ar_min) ** (k / (n_ar - 1))
                  for k in range(n_ar)]

    options = []
    for r in ratios:
        ok0, w0, h0 = packs(r, min_util)
        if not ok0:
            options.append(BBoxOption(r, w0, h0, min_util, False))
            continue
        # binary-search highest util that still packs
        lo, hi, best = min_util, 0.985, min_util
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            ok, _, _ = packs(r, mid)
            if ok:
                best = mid
                lo = mid
            else:
                hi = mid
        w, h = box_for(r, best)
        options.append(BBoxOption(r, w, h, best, True))

    feasible = [o for o in options if o.packs]
    logger(f"[bboxes] {len(feasible)}/{len(options)} aspect ratios pack at "
           f">= {min_util*100:.0f}% util (macro_area={A:.0f}, "
           f"largest macro {wmax:.1f}x{hmax:.1f})")
    for o in options:
        tag = f"util={o.util*100:.1f}%" if o.packs else "NO PACK at min_util"
        logger(f"    AR={o.aspect:5.2f}  w={o.w:8.2f} h={o.h:8.2f}  {tag}")
    return options


def edge_fill(res: PackResult, box, aligned_sides) -> dict:
    """Fraction of each aligned edge's length actually covered by macros touching it.

    A macro 'touches' an edge if its corresponding side is flush with it (tol 1e-6).
    Returns {side: covered_length / edge_length}.
    """
    llx, lly, urx, ury = box
    tol = 1e-6

    def union_len(intervals):
        if not intervals:
            return 0.0
        intervals.sort()
        total, cs, ce = 0.0, intervals[0][0], intervals[0][1]
        for s, e in intervals[1:]:
            if s > ce + tol:
                total += ce - cs
                cs, ce = s, e
            else:
                ce = max(ce, e)
        return total + (ce - cs)

    out = {}
    for side in aligned_sides:
        side = side.lower()
        ivs = []
        if side == "left":
            edge_len = ury - lly
            for p in res.placements:
                if abs(p.x - llx) < tol:
                    ivs.append((p.y, p.y + p.h))
        elif side == "right":
            edge_len = ury - lly
            for p in res.placements:
                if abs((p.x + p.w) - urx) < tol:
                    ivs.append((p.y, p.y + p.h))
        elif side == "bottom":
            edge_len = urx - llx
            for p in res.placements:
                if abs(p.y - lly) < tol:
                    ivs.append((p.x, p.x + p.w))
        elif side == "top":
            edge_len = urx - llx
            for p in res.placements:
                if abs((p.y + p.h) - ury) < tol:
                    ivs.append((p.x, p.x + p.w))
        else:
            continue
        out[side] = (union_len(ivs) / edge_len) if edge_len > 0 else 0.0
    return out


def pack_in_best_box(macros: Sequence[Macro],
                     aligned_sides: Sequence[str] = ("left", "top"),
                     min_util: float = 0.80,
                     ar_min: float = 0.25, ar_max: float = 4.0, n_ar: int = 13,
                     sort_key: str = "area", logger=print):
    """Find the tightest packable box (max utilization over the aspect sweep), then
    pack hugging `aligned_sides`. Returns (PackResult, box, BBoxOption, edge_fill).

    This is the "reduce the box to fill the boundaries" workflow: shrinking the box
    raises utilization, which in turn fills the aligned edges.
    """
    opts = find_packable_bboxes(macros, min_util, ar_min, ar_max, n_ar,
                                sort_key=sort_key, aligned_sides=aligned_sides,
                                logger=lambda *a, **k: None)
    feas = [o for o in opts if o.packs]
    if not feas:
        logger("[pack_in_best_box] no box packs at the requested min_util")
        return None, None, None, {}
    best = max(feas, key=lambda o: o.util)
    box = (0.0, 0.0, best.w, best.h)
    res = pack_macros(macros, box, aligned_sides=aligned_sides,
                      sort_key=sort_key, logger=lambda *a, **k: None)
    ef = edge_fill(res, box, aligned_sides)
    logger(f"[pack_in_best_box] box {best.w:.1f}x{best.h:.1f} "
           f"AR={best.aspect:.2f} util={best.util*100:.1f}%  "
           f"edge_fill=" + ", ".join(f"{k}:{v*100:.0f}%" for k, v in ef.items()))
    return res, box, best, ef


# ----------------------------------------------------------------------------
# Optional: visualize a packing result.
# ----------------------------------------------------------------------------
def plot_packing(res: PackResult, box, aligned_sides, path, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    llx, lly, urx, ury = box
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.add_patch(patches.Rectangle((llx, lly), urx - llx, ury - lly,
                                   fill=False, ec="black", lw=1.5))
    # highlight aligned edges
    edge = {"left": [(llx, lly), (llx, ury)], "right": [(urx, lly), (urx, ury)],
            "top": [(llx, ury), (urx, ury)], "bottom": [(llx, lly), (urx, lly)]}
    for sde in aligned_sides:
        (x0, y0), (x1, y1) = edge[sde.lower()]
        ax.plot([x0, x1], [y0, y1], color="red", lw=3, alpha=0.7)
    cmap = plt.get_cmap("tab20")
    for i, p in enumerate(res.placements):
        ax.add_patch(patches.Rectangle((p.x, p.y), p.w, p.h, fill=True,
                                       fc=cmap(i % 20), ec="black", lw=0.5,
                                       alpha=0.75))
    ax.set_xlim(llx - 0.05 * (urx - llx), urx + 0.05 * (urx - llx))
    ax.set_ylim(lly - 0.05 * (ury - lly), ury + 0.05 * (ury - lly))
    ax.set_aspect("equal")
    ax.set_title(title or f"pack: {res.num_placed} placed, "
                          f"{len(res.unplaced)} unplaced, "
                          f"util {res.utilization*100:.1f}%")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    # Demo: one tile's 16 banks (256x32 -> 60.61 x 169.4 um) + 4 icache
    # (64x64 -> 56.05 x 102.2 um) packed into a sample box hugging left+top.
    banks = [Macro(f"bank{i}", 60.61, 169.4) for i in range(16)]
    icache = [Macro(f"ic{i}", 56.05, 102.2) for i in range(4)]
    macros = banks + icache
    total = sum(m.area for m in macros)
    # size the box for ~80% utilization, squarish
    import math
    side = math.sqrt(total / 0.80)
    box = (0.0, 0.0, side, side)
    print(f"box {side:.1f} x {side:.1f}, macro area {total:.0f}, "
          f"box area {side*side:.0f}")
    res = pack_macros(macros, box, aligned_sides=("left", "top"))
    plot_packing(res, box, ("left", "top"), "out_pack_demo.png")
    print("wrote out_pack_demo.png")

    print("\n--- find_packable_bboxes (min_util=0.80, AR 0.25..4) ---")
    opts = find_packable_bboxes(macros, min_util=0.80, ar_min=0.25, ar_max=4.0,
                                n_ar=13)
    # pick the tightest feasible box and visualize it
    feas = [o for o in opts if o.packs]
    if feas:
        best = max(feas, key=lambda o: o.util)
        print(f"\ntightest packable box: {best.w:.1f} x {best.h:.1f} "
              f"(AR={best.aspect:.2f}, util={best.util*100:.1f}%)")
        r2 = pack_macros(macros, (0, 0, best.w, best.h),
                         aligned_sides=("left", "bottom"), logger=lambda *a: None)
        plot_packing(r2, (0, 0, best.w, best.h), ("left", "bottom"),
                     "out_bbox_best.png",
                     title=f"tightest box {best.w:.0f}x{best.h:.0f} "
                           f"util {best.util*100:.1f}%")
        print("wrote out_bbox_best.png")
