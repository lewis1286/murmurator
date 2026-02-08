# Debugging Notes

## UI Freeze Bug — DrawLine Unsigned Integer Overflow

**Symptom:** The Daisy Patch device freezes after running for a while. Appears to happen when a boid reaches the edge of the OLED display.

**Root Cause:** In `ui/display.cpp`, `DrawBoid()` computes triangle vertices using `cosf`/`sinf` offsets from the boid's center position. The center `x`/`y` was clamped to display bounds, but the three triangle vertices were not. When a boid was near the display edge, a vertex coordinate could go negative (e.g. `x1 = -3`).

libDaisy's `DrawLine()` in `display.h` takes `uint_fast8_t` parameters (which is `uint32_t` on ARM). A negative `int` cast to `uint32_t` wraps to a huge value (~4 billion). Bresenham's line algorithm then tries to walk from one huge coordinate to another, one pixel at a time — effectively an infinite loop that freezes the device.

**Fix:** Clamp all triangle vertex coordinates to display bounds before passing them to `DrawLine()`:

```cpp
auto clampX = [](int v) { return v < 0 ? 0 : (v > 127 ? 127 : v); };
auto clampY = [](int v) { return v < 10 ? 10 : (v > 63 ? 63 : v); };
x1 = clampX(x1); y1 = clampY(y1);
x2 = clampX(x2); y2 = clampY(y2);
x3 = clampX(x3); y3 = clampY(y3);
```

**Debugging approach:**
1. Built in UI-only mode (`make ui-only`) to isolate from audio code.
2. Used ST-Link + OpenOCD with Cortex Debug in VSCode.
3. When the device froze, hit pause in the debugger.
4. Call stack showed execution stuck in `DrawLine` at `display.h:244` — the Bresenham `while` loop.
5. Parameter values confirmed the overflow: `x1=183792181`, `x2=4294967295`, `y1=61264091`, `y2=32`.

**Lesson:** Any time you pass coordinates to libDaisy display functions, clamp them to valid bounds first. The `uint_fast8_t` parameters silently wrap negative values to huge unsigned values.

---

## Secondary Fix — ReadBufferInterpolated Guard

**Location:** `audio/grain_voice.cpp`, `ReadBufferInterpolated()`

**Issue:** Same class of bug as the `WrapPosition` edge freeze. The `while` loops wrapping the buffer index would spin forever on `NaN`/`Inf` values. Added `std::isfinite()` guard to return silence instead.
