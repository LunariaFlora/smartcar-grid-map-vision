# Algorithm Notes

Detailed notes on how the vision firmware works, what each parameter is for, and where it can fail.
This document is the English version of the design and audit notes kept with the original project; the
sections describing current behaviour were checked against `main.py` and `tracker.py` as published here.
Where an earlier version of the project behaved differently, that is called out explicitly, because the
reasoning behind a removed feature is often more useful than the feature was.

Everything described as "measured" was measured by the author during development. No measurement in this
document was repeated while packaging the repository, and no camera board was available then.

---

## 1. Data flow

```
camera init, keep default auto exposure
  |
  +-- calibrate_grid()                  find inner edges of the outer wall ring -> x0, y0, dx, dy
  |
  +-- capture_wall_mask()               3 frames, 5 sample regions per cell -> wall mask for this level
  |
  +-- construct CarTracker              with the geometry just calibrated
  |
  +-- main loop, forever:
        +-- if a UART request arrived: take 3 fresh frames, reply (map and/or position)
        +-- snapshot
        +-- car.update(img)             every frame
        +-- every 30 frames: process_once() -> character map
        +-- detect end of level / start of next level
        +-- draw overlay                last, so it never pollutes sampling
```

Two outputs are produced from the same frames but tracked separately:

1. **Continuous coordinates** - from the tracker's averaged blob centre, used for every position reply.
2. **Character map** - walls, floor, boxes, targets, bombs. The car is not part of it.

That separation is the single most important structural decision in this codebase; section 12 explains
what went wrong before it.

---

## 2. Module responsibilities

| File | Responsibility | On the runtime path |
| --- | --- | --- |
| `main.py` | camera setup, grid calibration, wall mask capture, cell classification, character map, level state machine, position correction, UART protocol, overlay | yes |
| `tracker.py` | per-frame blob search, candidate filtering, box-core exclusion, history filtering, pixel-to-grid conversion | yes |
| `tools/exposure_capture_test.py` | standalone exposure sweep and photo capture for threshold tuning | no |
| `tools/detect_map.py` | desktop: screenshots to character maps (RGB rules) | no |
| `tools/crop_map.py` | desktop: one cropped image to a printed character map | no |
| `tools/sokoban_generator.html` | offline level generator, solver check, exporter | no |
| `tests/*.py` | regression tests for calibration and tracker filtering | no |

`tracker.py` never touches the camera. `main.py` owns the sensor and passes frames in. This keeps the
two concerns separable and is what makes the tracker testable with fake images.

---

## 3. Grid model

- Fixed map size: `COLS = 16`, `ROWS = 12`, i.e. 192 cells.
- Cell centre in pixels: `x = x0 + c * dx`, `y = y0 + r * dy`.
- Fallback geometry used if calibration never converges: `x0 = 33`, `y0 = 35`, `dx = 16.5`, `dy = 15.8`.
- The playable interior is the region strictly inside the outer wall ring:
  `x0 + 0.5dx .. x0 + (COLS - 1.5)dx` horizontally, `y0 + 0.5dy .. y0 + (ROWS - 1.5)dy` vertically.
  This rectangle is what the code calls the red frame, and it is the only region where a car centre is
  allowed to be.

Grid coordinates reported over UART use a different convention: the origin is the bottom-left interior
corner, X grows to the right, Y grows upward. Since image Y grows downward, the conversion flips the
axis:

```
x_grid = (pixel_x - x0) / dx
bottom_y = y0 + (ROWS - 1) * dy
y_grid = (bottom_y - pixel_y) / dy
```

Both values are rounded to 0.1 cell and multiplied by 10 for transmission, so `X15Y23` means 1.5 cells
right and 2.3 cells up.

---

## 4. Grid calibration

Purpose: recover the screen geometry of the mat's **inner wall boundary**. Note the emphasis - the
algorithm looks for where the outer wall band ends and the playable area begins, not the outside outline
of the mat. That inner boundary is what the cell lattice is anchored to.

### 4.1 Search window

Candidate lines are scanned only within `search_margin = 45` px of the fallback positions, in steps of
`CALIB_SCAN_STEP = 2` px. This is a deliberate limitation: it makes calibration fast and robust for
"the camera was re-seated and the mat shifted slightly", and it makes free placement impossible. A mat
moved by more than roughly three cells will not be found.

### 4.2 Scoring an edge

For a candidate vertical line at `x`, the code samples `y` every `EDGE_SAMPLE_STEP = 6` px, and compares
the 3x3 LAB mean at `x - EDGE_OFFSET` and `x + EDGE_OFFSET` (`EDGE_OFFSET = 4`). Each sample returns a
(score, coverage) pair from `edge_evidence()`:

| Outside is calibration-wall | Inside is | Score | Counts as coverage |
| --- | --- | --- | --- |
| yes | floor | 8 | yes |
| yes | any other known map content | 4 | yes |
| yes | unrecognised | 1 | no |
| no | anything | 0 | no |

Two separate sums are accumulated, one assuming the wall is on the left and the playable area on the
right, and one the other way round, so a line can be recognised as a left edge or a right edge.

Calibration uses its own wall threshold (`15..100, -13..59, -62..4`) rather than the daily wall threshold.
The calibration version is much wider because it runs before the lighting has settled and because
mis-detecting the grid is fatal, while mis-detecting a wall mid-level merely costs a cell.

### 4.3 From scores to candidate lines

A raw scan produces a smooth score profile, and near a real boundary the profile has a several-pixel-wide
plateau (because the sample offsets straddle the transition). The code:

1. Keeps positions with `coverage >= MIN_EDGE_COVERAGE = 6`.
2. Ranks them by `score + coverage * 12`, then groups positions that are within `CALIB_SCAN_STEP + 1` of
   each other.
3. Replaces each group by its score-weighted centre. Without this step, the two shoulders of one plateau
   would be reported as two boundaries.
4. Keeps the best `EDGE_CANDIDATE_N = 14` groups, then re-scans a small window around each for a finer
   position (`refine_*_candidates`).

### 4.4 Pairing and rectangle validation

Left candidates are paired with right candidates, and top with bottom. For each pair the cell size is
`span / inner_cells`, where `inner_cells` is 14 horizontally and 10 vertically (the interior count). A
pair survives only if `MIN_CELL = 10 <= cell <= MAX_CELL = 24` px.

The best `RECT_CANDIDATE_N = 12` rectangles are then scored by `rectangle_quality()`, which validates
each of the four sides independently. Sampling runs from the boundary outward (into the wall band) and
inward (into the first ring of interior cells), at `0.38` of a cell on each side of the line:

- outside must be calibration-wall for at least 68% of samples,
- inside must be floor for at least 50%,
- inside must be known map content for at least 75%.

Each of these has to hold per side, so a long internal wall abutting one edge, or clutter just outside
the mat, cannot be compensated by the other three sides. Passing rectangles get

```
quality = 50 * wall_ratio + 30 * floor_ratio + 20 * known_ratio - 35 * |cell_x/cell_y - 1|
```

and must reach `MIN_RECT_QUALITY = 60`. The runner-up must be at least `MIN_RECT_MARGIN = 4` points
worse, otherwise the frame is treated as ambiguous and discarded - this is what stops the algorithm from
flip-flopping between two plausible rectangles.

The published result is shifted outward by half a cell so that `x0`/`y0` denote the *first cell centre*
rather than the boundary, which is what the rest of the code expects.

### 4.5 Consensus

Detection is repeated until `CALIB_CONSENSUS_N = 5` results agree within `CALIB_POSITION_TOL = 3.0` px
and `CALIB_CELL_TOL = 0.8` px. The median of the five is used. Any result that breaks the cluster resets
the count. Frames are spaced by `CALIB_INTERVAL_MS = 80` ms.

This phase is the slowest part of boot: each candidate line costs a full pass of `get_statistics()` calls
along that line, and the scan considers up to ~45 positions per side before refinement. It runs once, so
the cost is acceptable; it would not be acceptable per frame.

### 4.6 Known limits

- Axis-aligned, near-orthographic views only. There is no homography, no rotation estimate, and no
  four-corner fit, so a tilted camera or a rotated mat cannot be calibrated.
- The 45 px window means the mat must be roughly where the fallback expects it.
- Calibration refuses ambiguous frames rather than guessing, and the boot loop then keeps waiting. If the
  board never converges it will not proceed on its own.

---

## 5. Wall mask

Walls are the only map content that is captured once and then treated as ground truth for the level.

`capture_wall_mask()`:

1. Start with border cells (`r == 0`, `r == ROWS-1`, `c == 0`, `c == COLS-1`) marked as walls.
2. Take three frames, 80 ms apart.
3. For each interior cell, test five regions: the centre plus four offsets of about `0.18` of a cell in
   each direction. Each region is a 3x3 LAB mean.
4. A sample region votes "wall" if it matches the daily wall threshold. Regions that match either car
   threshold are skipped entirely rather than counted as non-wall, so the car sitting on the mat at boot
   cannot be promoted to a wall.
5. A cell becomes a wall if at least 3 of 5 regions vote wall - in *any* of the three frames.

The union-over-frames rule is intentionally biased toward recall: a missing wall is much worse than a
spurious one, because walls feed the position correction. The cost is that one bad frame can permanently
add a phantom wall for that level.

The mask is captured per level. `main.py` does this at boot and again each time a new level is confirmed.

---

## 6. Cell classification and map maintenance

`classify_lab()` tests thresholds in a fixed order and returns the first match:

1. target `.`
2. box `$`
3. bomb `*`
4. wall `#` (only when walls are allowed)
5. floor `-`

If nothing matches, the cell is reported as floor. This is a deadlock-avoidance fallback, not a
statement that the cell is empty: a weakly lit box or an unrecognised object silently becomes floor.

Because the order decides overlaps, widening one threshold can steal cells from another with no warning.
There is no confidence score and no notion of "uncertain".

`process_once()` produces two maps from one image:

- **observed map** - what this frame looks like, used only to decide "the level is empty" and "a new
  level has appeared".
- **official map** - what gets reported. Differences from the observed map:
  - a cell is a wall only if the level's wall mask says so; if the image looks like a wall but the mask
    does not, the cell is re-classified with walls disabled;
  - the cell the car occupies keeps the previous map's content (or floor) and is never overwritten.

The second rule exists because the car's own body colour overlaps with the box and wall thresholds. It
also means the reported map can lag reality by one update in the single cell under the car, which is
acceptable because the consumer of the map is a planner that also receives the car position.

---

## 7. CarTracker

### 7.1 What it maintains

| State | Meaning |
| --- | --- |
| `history_x`, `history_y` | last up to 3 successful blob centres (`FILTER_N = 3`) |
| `center_x`, `center_y` | average of that history: drives the overlay and the search window |
| `rel_x`, `rel_y` | tenths-of-a-cell coordinates computed from the same average, sent over UART |
| `confirmed_boxes` | boxes found in the 3x3 neighbourhood of the current cell this frame |
| `fail_count` | consecutive frames without an accepted candidate |
| `found` | whether this frame produced a valid car |

### 7.2 Search region

- No history, or `fail_count >= 3`: the whole interior region.
- Otherwise: a window around the averaged centre, half-size `max(24, 2.2 * dx)` by `max(24, 2.2 * dy)`,
  clipped to the interior.

This is the main performance lever - a local `find_blobs()` costs a fraction of a full-frame one. It is
also a source of path dependence: if the tracker locks onto a wrong object, the window follows it and the
error is self-sustaining, because a wrong-but-plausible blob never increments `fail_count` far enough to
trigger whole-area recovery.

### 7.3 Candidate generation and filtering

```python
img.find_blobs([threshold], roi=roi, pixels_threshold=50, area_threshold=50, merge=True)
```

`merge=True` lets fragmented car pixels join into one blob, which is what you want for a car partly
occluded by a shadow; it is also what lets the car fuse with an adjacent box or a wall highlight into a
blob whose centre is not the car centre.

Filters applied per candidate, in order:

1. Centre inside the playable area (excludes the wall band and anything off the mat).
2. Size: 8-35 px per side for a normal one-cell car. Near a confirmed box the limit relaxes to
   7 px .. `max(35, 2.4 * dx)` per side, which is what allows a merged car+box blob to survive long
   enough to be handled by the recovery path.
3. Aspect ratio 0.6-1.6 normally, 0.4-2.5 near a box, 0.65-1.55 when re-acquiring after a long loss.
4. Not inside a confirmed box core (section 7.4).
5. Jump limit: distance from the averaged history must not exceed `MAX_JUMP_CELLS = 1.3` cells. When
   re-acquiring, a far candidate is allowed, but must not be box-coloured at its centre, and must be
   confirmed twice (section 7.5).

Selection: nearest to the averaged history wins; on ties, the one with more pixels. With no history, the
largest candidate wins.

### 7.4 Box exclusion and merged-blob recovery

Boxes are the main source of confusion because their threshold can overlap the car's fallback threshold.
Two mechanisms handle them.

**Confirming nearby boxes.** `scan_nearby_boxes()` looks at the 3x3 neighbourhood of the current grid
cell and marks a cell as a box if its 5x5 centre mean matches the box threshold. Nine tiny statistics
calls per frame - much cheaper than a second full `find_blobs()`.

**Removing box cores from candidates.** A candidate whose centre falls within `BOX_CORE_HALF = 0.46`
cells of a confirmed box centre is dropped. This kills a pure box masquerading as the car.

**Recovering a merged car+box blob.** Dropping the whole blob would mean `LOST` whenever the car touches
a box, so candidates that overlap a box core take a different path:

1. Pick the overlapping box nearest the last known position.
2. Choose a search direction from the history offset relative to that box: left, right, above, or below.
3. Build three overlapping strips on that side of the box (the "5 is the box" half-ring: 7-8-9 / 4-6 /
   1-2-3 and the analogous arrangements). Three strips rather than five single cells means the car is
   still one connected blob when it straddles a cell boundary.
4. Cut each strip where it meets another box or a target, so the search cannot merge across special cells.
5. Re-run `find_blobs()` inside each remaining run's bounding box, with the same size, ratio, jump, and
   box-centre filters.

The final centre therefore comes from the residual real car blob, not from the merged shape. If the
history is empty, or the history itself already lies inside the box core (direction unknowable), or no
candidate survives the filters, the frame returns no target.

The extra `find_blobs()` calls only happen when a candidate actually covers a box core, so ordinary
tracking pays nothing.

### 7.5 Re-acquisition after a long loss

After `REACQUIRE_AFTER_FAILS = 5` consecutive failures the tracker allows far candidates. A far
candidate is not accepted immediately:

- It must be within `REACQUIRE_MATCH_CELLS = 0.65` cells of the previous far candidate, repeated
  `REACQUIRE_CONFIRM_N = 2` times.
- Until confirmed, `fail_count` keeps increasing and the frame reports nothing.
- On confirmation, the history and averaged centre are cleared, so the new position starts a fresh
  history instead of being blended with a stale one.

This is what prevents a single spurious far blob from teleporting the reported position, while still
allowing recovery when the car really was picked up and moved.

### 7.6 Per-frame cost

In the common case: up to 9 small `get_statistics()` calls, one local `find_blobs()`, then arithmetic.
Additional costs, in decreasing order of impact:

1. a second `find_blobs()` because the main threshold failed,
2. the search window degenerating to the whole interior,
3. per-strip `find_blobs()` calls during box recovery,
4. the 192-cell map sweep (only every 30 frames).

---

## 8. Position correction against walls

`get_serial_coords()` takes the tracker's grid coordinates and, if they overlap a wall cell, nudges them
out:

- car modelled as 0.98 cell square (`CAR_HALF = 0.49`), wall cell as 1.0 (`CELL_HALF = 0.5`);
- minimum separation therefore 0.98 cell;
- for every wall cell in the mask, if both axes are closer than that, the axis with the smaller overlap
  decides the push direction;
- only the single smallest push is applied, capped at `MAX_CORRECTION = 0.1` cell;
- if the coordinate sits exactly on a wall centre, no direction is guessed and nothing is changed.

Only walls participate. Boxes, targets, and bombs are excluded on purpose: a car may legitimately stand
on a target, and an earlier version that corrected against targets produced a visible bias when the car
was on one.

Correction changes the reported string only. It does not feed back into the tracker's history, the search
window, or the candidate centres, so if the underlying tracking is biased the correction is reapplied
every frame rather than fixing the cause. It also cannot know which side of the wall the car came from,
so a coordinate that has already crossed the wall centre can be pushed out on the wrong side.

---

## 9. Serial protocol

Timing constants in `main.py`:

| Constant | Value | Purpose |
| --- | --- | --- |
| `SERIAL_SETTLE_MS` | 200 | minimum time from request to reply |
| `SERIAL_TRACK_SAMPLES` | 3 | fresh frames taken before replying |
| `MAP_UPDATE_INTERVAL` | 30 | frames between map sweeps in normal operation |
| `NEW_LEVEL_SCAN_INTERVAL` | 3 | frames between map sweeps while waiting for a new level |
| `NEW_LEVEL_CONFIRM_N` | 2 | consecutive non-empty observations needed to accept a new level |

Requests are handled before the frame is taken for tracking, so a request is answered immediately with
data from the three frames it triggered itself, rather than with the previous frame's result.

On request `1`:

```
send nothing if no valid map is known
0xAA
192 ASCII characters, row-major
"\r\n"
0xFF
position line + "\n"
```

On request `2`: `position line + "\n"` only.

`LOST` is emitted when all three fresh frames fail. `main.py` still records `last_good_serial_coord`, but
only uses it for the debug print on the terminal - it is never sent as if it were current. That is the
right behaviour for a planner that can skip an update, and it means the controller must be able to
tolerate a `LOST` answer at any time.

Note that the freshly taken frames are averaged, so the reply trades up to ~200 ms of latency for a
noticeably steadier centre than a single frame would give.

---

## 10. Level state machine

```
normal operation
   |  observed interior strictly empty twice in a row  (clear_map_streak >= 2)
   v
waiting for new level   (map scanned every 3 frames; observed map only)
   |  observed interior has any non-floor cell twice in a row
   v
capture_wall_mask()  ->  one map sweep  ->  car.reset_history()  ->  normal operation
```

Details that matter:

- Grid calibration is not repeated on level change. The camera has not moved, so the geometry is still
  valid; re-running the slow calibration between levels would cost seconds.
- `reset_history()` clears failure count, history, averaged centre, coordinates, and confirmed boxes,
  while keeping geometry and thresholds. Without it, the new level's search window starts around the
  previous level's last position and can produce a run of `LOST` frames or lock onto the wrong object.
- The "level is empty" test requires the interior to be strictly floor, excluding the car's own cell.
  A single misclassified cell (a box misread as a bomb, say) will keep the board from declaring the
  level finished.
- The "new level appeared" test is deliberately loose: any non-floor interior cell counts. If the car's
  body is misread as a wall or a box at the start of a level, two consecutive frames can satisfy it
  early. In practice this shows up as a map that starts filling in before the car is placed.

---

## 11. Failure chains worth knowing

These are the paths that actually produced wrong positions during development. They are documented
because each one is a plausible-looking behaviour, not an obvious crash.

**A. Drift toward an internal wall.** The car hugs an internal wall, its threshold merges with a wall
highlight, `merge=True` produces one blob whose centre sits between car and wall, the tracker has no wall
exclusion, the candidate is close to the previous position so the jump filter allows it, and the reported
position slides toward the wall. Fixing this needs the wall map inside the candidate filter, which is the
main improvement this code does not have.

**B. Stuck on a wrong object.** A wrong blob appears every frame, so `fail_count` never grows, whole-area
recovery never triggers, and the averaged history follows the wrong object.

**C. `LOST` while touching a box.** Car and box merge, the blob centre lands inside the box core, the
blob is dropped, no second candidate exists, and the frame reports nothing.

**D. Offset while touching a box.** Same merge, but the centre falls just outside the core, so the
candidate is kept and the relaxed near-box size limits accept it; the reported centre is the merged one.

**E. Vanishing at the edge.** Part of the car extends beyond the interior rectangle, the blob is clipped,
size and ratio filters reject it, and the search window - which is itself clipped to the interior - cannot
recover the missing part.

---

## 12. What changed in the July 2026 cleanup, and why

The original firmware had a fixed 900 us exposure for wall capture, an auto-exposure restore step, a car
marker written into the character map, and a position correction that included boxes, bombs, and targets.
Those were removed for concrete reasons:

- **Fixed exposure and restore removed.** Wall capture now runs under the same default auto exposure as
  everything else. The fixed-exposure path required a reset, a 2.5 s wait, a brightness check, and up to
  three retries, costing up to ~7.5 s of boot, and it still could not guarantee that gain, white balance,
  and local brightness matched the values the thresholds were tuned for. The daily thresholds were
  retuned for auto exposure, which is also why the exposure sweep tool is now a standalone diagnostic
  rather than a boot step.
- **Car marker removed from the character map.** Previously the tracker's cell was written into the map
  as a car marker, unconditionally, after the map had been built. When the tracked cell drifted onto an
  internal wall, the wall character was overwritten. The obstacle list for position correction was then
  built from that map, so the wall that most needed correcting against disappeared exactly when the
  tracker was drifting into it. Removing the marker removed the whole chain.
- **Correction narrowed to walls only.** Correcting against targets pushed the car off cells it is
  allowed to stand on; correcting against boxes and bombs at 0.1 cell per frame produced jitter without
  improving accuracy. The corrected result is also no longer allowed to skip a coordinate that lands
  exactly on a wall centre - it now declines to guess rather than choosing arbitrarily.
- **Per-level wall mask made authoritative.** Ordinary classification may no longer create walls. The
  accepted cost is that a destroyed wall cannot open a path mid-level; if that is needed, the correct
  form is a one-way rule that lets the mask shrink, not a return to free wall creation.
- **Serial replies re-sample instead of trusting history.** Replies used to reuse whatever the tracker
  had; they now force three fresh frames and average them, so a settled car does not report a
  mid-movement artefact.

Dead code removed at the same time: an unused wall-deviation constant, unused RGB reference colours, an
always-zero rotation parameter and its trigonometric rotation path, unused `image` imports, and a
duplicated matrix-validation function.

---

## 13. Regression checklist

Useful when touching the tracker or the thresholds. All of it requires the board and the mat.

Normal area:
- stationary car: reported position stays stable;
- crossing a cell boundary horizontally and vertically updates within one successful frame.

Next to a wall (all four sides):
- reported position stays on the side the car is actually on;
- a coordinate that lands exactly on a wall centre does not jump to the far side.

Corner:
- two adjacent walls do not produce a shoving match between successive corrections.

Boxes:
- a box alone is never reported as the car;
- touching a box does not produce sustained `LOST`;
- the reported position does not sit inside the box.

Targets:
- a car standing on a target is not pushed off it.

Level change:
- the previous level's last position does not leak into the new level's history;
- request `1` on the new level returns a complete 192-character map.

`LOST`:
- short losses and long losses are distinguishable by the consumer.

---

## 14. Scope and assumptions

- The 25-28 FPS figure quoted in the original notes was measured before the cleanup; treat it as an
  earlier observation and re-measure if the exposure handling changes.
- The tracker tests assert invariants of the recovery path (a side search happens, and no side region
  covers the box centre) rather than a fixed `find_blobs()` call count, because the number of strip
  runs follows the grid geometry.
- `tools/detect_map.py` and `tools/crop_map.py` were run during development against reference
  screenshots. Their RGB rules are tuned to that render, so a photograph of a real mat may need
  recoloured thresholds.
