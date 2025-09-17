# ir3v2-klipper-bed-mesh
Updated bed_mesh.py for the IR3V2 Belt Printer.

# Axis-Selectable Bed Mesh (Klipper Patch)

> **Version:** axis_patch `0.1.0`  
> **Status:** Experimental (NOT upstream Klipper)  
> **License:** GPLv3 (derivative work of upstream `bed_mesh.py`)

This patched `bed_mesh.py` adds the ability to apply Klipper's mesh compensation to an axis other than Z (e.g. Y on belt / infinite-Z printers). It introduces two new configuration options:

| Option | Purpose | Default |
|--------|---------|---------|
| `apply_to_axis` | Which axis receives mesh compensation (`X`, `Y`, or `Z`) | `Z` |
| `fade_reference_axis` | Axis used to compute fade-out factor (optional; defaults to `apply_to_axis`) | (same as `apply_to_axis`) |

The rest of the behavior (probing, interpolation, profile saving/loading) remains consistent with upstream Klipper.

---

## Why Would I Use This?

If you have a non-standard geometry (e.g. a belt printer where **Y** controls the nozzle–belt gap while “Z” in G-code represents belt advance), the stock Klipper mesh is misapplied: it adjusts Z when you actually need to adjust Y. This patch lets you:

- Keep using normal `BED_MESH_CALIBRATE` workflows.
- Redirect the compensation to Y (or even X for exotic builds).
- Keep profile management and interpolation algorithms (Lagrange, Bicubic, Direct).

If you only need a 1‑D compensation (e.g. a single probe row across width), consider a lighter external extra (like a `belt_gap_comp` module) instead of patching the core file.

---

## Features Added

- Axis remapping (`apply_to_axis: Y`) without editing many call sites.
- Optional separate axis to determine fade (`fade_reference_axis`).
- Status dictionary includes:
  - `apply_to_axis`
  - `axis_patch_version`
- Version constant (`AXIS_PATCH_VERSION`) for easier tracking during future Klipper updates.
- Fully backward compatible *when* `apply_to_axis: Z` (acts like stock aside from benign log additions).

---

## Warnings & Caveats

| Risk | Description | Mitigation |
|------|-------------|------------|
| Upstream divergence | Future Klipper updates may change internals of `bed_mesh.py` | Re-apply patch after each update; keep a diff |
| Other modules assume Z | Macros expecting Z-only mesh (Z hop logic, safety moves) may behave unexpectedly | Audit start/end macros, disable unneeded Z-lift logic |
| Fade semantics | Fading along Y may be undesirable on belt printers (long travel) | Disable fade via huge `fade_start`/`fade_end` or omit |
| Sign confusion | Mechanical direction may invert perceived correction | If gap worsens where mesh is “higher,” invert inside `calc_z` (see below) |
| 1-row mesh edge cases | Some interpolation methods expect 2D; `y_count: 1` is OK but restrict to direct/lagrange effectively | Keep `probe_count: N,1` and avoid bicubic with single row |

---

## Installation

1. **Backup original:**
   ```bash
   cp klippy/extras/bed_mesh.py klippy/extras/bed_mesh.py.stock
   ```
2. **Replace file** with the patched version (your updated `bed_mesh.py`).
3. **Restart Klipper:**
   ```
   RESTART
   ```
4. **Verify log** (`klippy.log`) contains a line like:  
   `bed_mesh axis patch v0.1.0: apply_to_axis=Y fade_ref_axis=Y`

---

## Configuration Examples

### 1. Belt / Infinite-Z (Apply to Y)

```ini
[bed_mesh]
mesh_min: 0, 3
mesh_max: 250, 3      # Single row
probe_count: 6,1
mesh_pps: 5,1
algorithm: lagrange
apply_to_axis: Y
fade_reference_axis: Y
fade_start: 9999      # disables fade effectively
fade_end: 10000
```

### 2. Apply to Y but Fade by (True) Z

If your G-code “Z” is virtual belt advance but you want fading relative to vertical build height (rare):

```ini
[bed_mesh]
apply_to_axis: Y
fade_reference_axis: Z
fade_start: 1
fade_end: 30
```

---

## Workflow

1. `BED_MESH_CALIBRATE`
2. Inspect: `BED_MESH_OUTPUT`
3. `SAVE_CONFIG`
4. Confirm status in e.g. `GET_STATUS` (front-end dependent)
5. Start print

---

## Understanding Output

`BED_MESH_OUTPUT` now includes a line:

```
bed_mesh axis patch version: 0.1.0 (apply=Y fade_ref=Y)
```

If absent, you’re not running the patched file.

---

## Sign Inversion (If Needed)

If after enabling `apply_to_axis: Y`:

- Areas that should receive a positive lift result in *reduced* gap (nozzle too close), or
- Your mesh values are negative where the surface is “high,” but the nozzle moves the wrong way,

You can invert the applied correction by editing the `ZMesh.calc_z` return:

Original (patched file):
```python
return lerp(ty, z0, z1)
```

Invert:
```python
return -lerp(ty, z0, z1)
```

Then `RESTART`.

(Alternatively, add a new config flag; omitted to keep patch minimal.)

---

## Compatibility with Profiles

Profiles saved via:
```
BED_MESH_PROFILE SAVE=<name>
BED_MESH_PROFILE LOAD=<name>
```
are unchanged. Profiles **do not store** the axis remap—so switching `apply_to_axis` later does *not* invalidate old meshes.

---

## Fading Strategy

For belt printers, fading often isn’t useful because:
- You usually want consistent gap compensation across long continuous builds.
- Fading tied to belt travel could prematurely drop correction.

Disable fade by setting:
```ini
fade_start: 9999
fade_end: 10000
```
(or simply keep defaults if compensation zone always remains smaller than `fade_start`).

---

## Troubleshooting

| Symptom | Possible Cause | Fix |
|---------|----------------|-----|
| No compensation effect | Wrong axis in config | Check `apply_to_axis` |
| Log shows “fade complete” early | Fade range too small for chosen reference axis | Enlarge or disable fade |
| First layer inconsistent across width | Mesh not recalibrated after mechanical change | Run `BED_MESH_CALIBRATE` again |
| Crash / unexpected move during probe | Non-zero `y_dist` logic messed with single row params | Ensure `probe_count: N,1` and mesh_min/max share Y |
| Macro positions off | Macro reads transformed axis | Use `GET_POSITION` semantics carefully; understand axis removal in `get_position()` |

---

## When to Use an External Module Instead

Use a *separate extra* (like a `belt_gap_comp` 1D interpolator) if:

- You only need single-row (width) compensation.
- You want to keep upstream `bed_mesh.py` untouched.
- You prefer a lighter maintenance burden.

The module approach also makes upstream pulls painless—no merging required.

---

## Upgrading the Patch Version

When you change functionality:
1. Increment `AXIS_PATCH_VERSION` in the header.
2. Add a short CHANGELOG entry (see below).
3. Communicate version in PR/commit message.

### Example CHANGELOG (inline):

```
0.1.0 - Initial axis-selectable patch (apply_to_axis, fade_reference_axis)
```

---

## Suggested Git Commit Practices

- Commit the patched file alone with a message like:
  ```
  bed_mesh: add axis-selectable compensation (apply_to_axis, fade_reference_axis)
  ```
- Maintain a separate branch (e.g. `axis-mesh-patch`) to rebase after upstream updates.

---

## Minimal Diff Strategy (Future Upstream Updates)

After pulling upstream:
1. Run `git diff upstream/master -- klippy/extras/bed_mesh.py > old_patch.diff`
2. Re-apply logic incrementally:
   - Axis mapping constants
   - New config parsing
   - Modified `move()`, `get_position()`
   - `MoveSplitter` additions
3. Update version constant.

---

## Known Limitations

- Not tested against every combination of `quad_gantry_level`, `z_tilt`, or exotic kinematics.
- Does not dynamically switch axis mid-print.
- Doesn’t expose an API to query compensation at arbitrary coordinates (could be added).

---

## Frequently Asked Questions

**Q: Does this change the mesh math?**  
No—sampling and interpolation are unchanged; only where the resulting scalar is applied.

**Q: Can I compensate X instead?**  
Yes: `apply_to_axis: X` (rare; ensure no mechanical cross-coupling issues).

**Q: Does this interfere with adaptive mesh or custom macros?**  
If macros rely on bed mesh applying to Z, they may need awareness of the axis remap. Test carefully.

---

## License Notice

This file is a derivative of Klipper’s GPLv3 licensed `bed_mesh.py`. Redistribution and modification must comply with GPLv3. Provide attribution and a copy of the license where required.  
- Original Authors: Kevin O'Connor, Eric Callahan  
- Patch Author(s): (Add your name / handle here)

---

## Support / Feedback

- Use at your own risk—this is an advanced customization.
- Report issues by attaching:
  - `printer.cfg` mesh section
  - `klippy.log`
  - Output of `BED_MESH_OUTPUT`
- Consider isolating in a dedicated branch for reproducibility.

---

## Quick Reference (Cheat Sheet)

| Action | Command |
|--------|---------|
| Calibrate Mesh | `BED_MESH_CALIBRATE` |
| Save Profile | `BED_MESH_PROFILE SAVE=mybelt` |
| Load Profile | `BED_MESH_PROFILE LOAD=mybelt` |
| Clear Mesh | `BED_MESH_CLEAR` |
| Show Mesh | `BED_MESH_OUTPUT` |
| Set Offsets | `BED_MESH_OFFSET X=… Y=…` |

---

### Example First-Time Belt Calibration (Single Row)

```
G28
BED_MESH_CALIBRATE
SAVE_CONFIG
BED_MESH_OUTPUT
; Start test print
```

---

