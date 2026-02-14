# Plan: UI-Only Build and Edge Freeze Fix

## 1. Likely Cause of Freeze at Screen Edge

The freeze is likely in **`BoidsFlock::WrapPosition`** (boids.cpp):

```cpp
while (pos.x >= 1.0f) pos.x -= 1.0f;
while (pos.y >= 1.0f) pos.y -= 1.0f;
```

If `pos.x` or `pos.y` ever becomes **`Inf`** (or `-Inf`):

- `Inf >= 1.0f` is true
- `Inf - 1.0f` is still `Inf`
- The `while` loop never terminates → **device freezes**

**How could position become Inf?**

- In **Separation**: `diff / (dist * dist)` with a very small `dist` can produce huge values; `steering.Limit(params.max_force)` and `SetMagnitude` can still leave large values that, over many frames or through other math, blow up to `Inf`.
- In **Vec2::SetMagnitude**: it calls `Normalize()` then `x *= mag`. If the vector is zero, `Normalize()` leaves it unchanged; `SetMagnitude` then multiplies by `mag` and you get `(0,0)`, which is fine. But if `Normalize()` or other steps ever see `Inf`/`NaN`, it can propagate.
- Any `Inf`/`NaN` in position or velocity will eventually reach `WrapPosition`; the `while` loops are the only place that can loop forever.

**Conclusion:** Harden `WrapPosition` to handle non-finite values (and optionally make the wrap itself more robust) so the device cannot hang even if upstream math misbehaves.

---

## 2. UI-Only Mode: What to Disable

| Component | In UI-only? | Notes |
|-----------|-------------|--------|
| `patch.Init()` | ✅ Keep | Required for hardware |
| `patch.StartAdc()` | ✅ Keep | Knobs, CV, gates, encoder |
| `buffer.Init()` | ✅ Keep | Needed for `DrawWaveform`, `buffer.IsRecording()`, `SetRecording`; `audio_buffer` exists as global; Init just zeros it and sets `write_pos_`/`recording_` |
| `grain_pool.Init()` | ❌ Skip | Only used in `AudioCallback` |
| `scheduler.Init()` | ❌ Skip | Only used in `AudioCallback` |
| `patch.StartAudio(AudioCallback)` | ❌ Skip | **Main change:** no audio callback, no DMA, no I2S |
| `AudioCallback` | — | Leave defined; it is never called when `StartAudio` is not used |

**Keep as-is (no audio I/O in practice):**

- Main loop: `UpdateControls`, boids `Update`, `UpdateDisplay`, `led_grid`
- `UpdateDisplay` / `DrawWaveform`: can keep using `audio_buffer` and `buffer.GetWritePosition()`; in UI-only the buffer stays zeroed → flat line, which is fine for testing.

---

## 3. How to Switch: Compile-Time Flag

Use a **preprocessor flag** so you can build either full or UI-only without maintaining two source files.

### 3.1 Makefile

The libDaisy `core/Makefile` uses `C_DEFS` in `CFLAGS`/`CPPFLAGS`. Add an `ui-only` target that appends `-DMURMUR_UI_ONLY` and depends on `all`. In `murmur/Makefile`, add **after** the `include $(SYSTEM_FILES_DIR)/Makefile` line:

```makefile
# Build UI-only (no audio). Usage: make ui-only
ui-only: C_DEFS += -DMURMUR_UI_ONLY
ui-only: all
```

Alternatively, without a new target: `make C_DEFS="-DMURMUR_UI_ONLY"` (this overwrites `C_DEFS` if the core uses `?=`, so prefer `make C_DEFS="... $(C_DEFS) -DMURMUR_UI_ONLY"` only if you know the order; the `ui-only` target is simpler).

### 3.2 MurmurBoids.cpp

In `main()`, guard the audio-only initialisation and `StartAudio`:

```cpp
    // ...

    patch.StartAdc();

#ifndef MURMUR_UI_ONLY
    grain_pool.Init();
    scheduler.Init(sample_rate);
    patch.StartAudio(AudioCallback);
#endif

    while (1) {
```

- `buffer.Init()` stays where it is (before this block).
- `sample_rate = patch.AudioSampleRate()` can stay; it’s only used for `scheduler.Init`, which is skipped in UI-only.

---

## 4. Optional: Stub / Hide Waveform in UI-Only

If you prefer the waveform page to show “No audio” instead of a flat line:

- Define `DrawWaveform` (or a wrapper) to take an extra `bool ui_only` or to check `#ifdef MURMUR_UI_ONLY` and, in that case, draw a short message like “No audio (UI-only)” and return.
- Otherwise, drawing the zeroed `audio_buffer` is acceptable and requires no display changes.

---

## 5. Fixing the Edge Freeze (WrapPosition)

In `boids.cpp`, make `WrapPosition` robust to non-finite values and keep the wrap logic for normal values.

Add `#include <cmath>` at the top of `boids.cpp` if not already present (Vec2 already uses `sqrtf` etc.; `std::isfinite` is in `<cmath>`).

Replace:

```cpp
void BoidsFlock::WrapPosition(Vec2& pos) {
    while (pos.x < 0.0f) pos.x += 1.0f;
    while (pos.x >= 1.0f) pos.x -= 1.0f;
    while (pos.y < 0.0f) pos.y += 1.0f;
    while (pos.y >= 1.0f) pos.y -= 1.0f;
}
```

with something like:

```cpp
void BoidsFlock::WrapPosition(Vec2& pos) {
    // Avoid infinite loops from Inf/NaN (can come from bad forces or FP edge cases)
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y)) {
        pos.x = 0.5f;
        pos.y = 0.5f;
        return;
    }
    while (pos.x < 0.0f) pos.x += 1.0f;
    while (pos.x >= 1.0f) pos.x -= 1.0f;
    while (pos.y < 0.0f) pos.y += 1.0f;
    while (pos.y >= 1.0f) pos.y -= 1.0f;
}
```

`std::isfinite` is true only for finite numbers (false for `Inf`, `-Inf`, `NaN`). Resetting to the center avoids hang and gives a safe, visible state.

---

## 6. Optional: Harden Force Math

To reduce the chance of `Inf`/`NaN` in the first place:

- In **Separation**, the `dist > 0.0001f` check is good. You can add an upper clamp on the scaled diff, e.g. `diff = diff / (dist * dist)` then `diff.Limit(some_max)` or clamp `dist` from below to a small epsilon so `dist * dist` never underflows in a bad way. This is secondary to fixing `WrapPosition`.
- In **Vec2::Limit** and **SetMagnitude**, if `Magnitude()` or `sqrtf(mag_sq)` is `Inf`/`NaN`, you could add an `isfinite` check and no-op or reset, but `WrapPosition` already prevents a hard freeze.

---

## 7. Suggested Order of Work

1. **Harden `WrapPosition`** (Section 5)  
   - Easiest, targets the freeze directly.

2. **Add `MURMUR_UI_ONLY` and `#ifndef` in `main()`** (Sections 3.1–3.2)  
   - Lets you test UI (OLED, LED grid, knobs, encoder, gates) without audio.

3. **Add `ui-only` Makefile target** (or equivalent)  
   - So you can run `make ui-only` (or `make C_DEFS="-DMURMUR_UI_ONLY"`) without editing the Makefile each time.

4. **(Optional)** Stub `DrawWaveform` in UI-only; or leave it as a flat line.

5. **(Optional)** Add extra guards in Separation / Vec2 if you still see odd behaviour after the `WrapPosition` fix.

---

## 8. Build and Flash

- **Full build:**  
  `make clean && make`  
  then `make program-dfu` (or your usual flash method).

- **UI-only build:**  
  `make clean && make ui-only`  
  (or `make clean && make C_DEFS="-DMURMUR_UI_ONLY"`)  
  then `make program-dfu`.

**Note:** Use `make clean` before switching between full and UI-only so object files are rebuilt with the correct `MURMUR_UI_ONLY` define.

---

## 9. Testing UI-Only

- Knobs, encoder, gates: should still be read via `StartAdc` and `ProcessAnalogControls` / `ProcessDigitalControls`.
- OLED (flock, params, waveform) and LED grid: driven from the main loop; no audio callback needed.
- Boids: `flock.Update` runs in the main loop; `scheduler.Process` and `grain_pool.Process` never run.
- No audio input or output; device should not depend on I2S or DMA for the UI.

If the freeze was in `WrapPosition`, the `isfinite` fix should prevent it even when a boid reaches or crosses the edge. If you still see freezes in UI-only, the next place to check is `UpdateSpatialGrid` / `GetNeighbors` (e.g. `%` with negative `gx`/`gy` on the target toolchain) or long loops in `CircularBuffer::WrapIndex` for extreme `index` values; those are less likely than `WrapPosition` with `Inf`.
