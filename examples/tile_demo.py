#!/usr/bin/env python
"""Example: shape-curve generation + tightest-box packing for a tile's macros.

A mempool tile has 16 bank SRAMs (256x32 -> 60.61 x 169.4 um) and
4 icache SRAMs (64x64 -> 56.05 x 102.2 um). We:
  1. generate the set of packable bounding boxes (>= 80% util, AR 0.25..4), and
  2. pack the macros into the tightest one, hugging the left+top boundaries.

Run:  python examples/tile_demo.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from packing import (Macro, find_packable_bboxes, pack_in_best_box,
                     plot_packing)


def main():
    banks = [Macro(f"bank{i}", 60.61, 169.4) for i in range(16)]
    icache = [Macro(f"ic{i}", 56.05, 102.2) for i in range(4)]
    macros = banks + icache

    print("== packable bounding-box set (>=80% util, AR 0.25..4) ==")
    opts = find_packable_bboxes(macros, min_util=0.80, ar_min=0.25, ar_max=4.0,
                                n_ar=13, logger=lambda *a, **k: None)
    print(f"{'aspect(w/h)':>11} {'width':>9} {'height':>9} {'util':>7}  packable")
    print("-" * 52)
    for o in opts:
        flag = "YES" if o.packs else "no (needs <80%)"
        print(f"{o.aspect:11.2f} {o.w:9.2f} {o.h:9.2f} {o.util*100:6.1f}%  {flag}")

    print("\n== pack into tightest box, hug left+top ==")
    res, box, best, ef = pack_in_best_box(macros, aligned_sides=("left", "top"),
                                          min_util=0.80)
    if res is not None:
        print(res.summary())
        try:
            plot_packing(res, box, ("left", "top"), "out_tile_best.png",
                         title=f"tile macros: {best.w:.0f}x{best.h:.0f} "
                               f"util {best.util*100:.1f}%")
            print("wrote out_tile_best.png")
        except Exception as e:
            print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()
