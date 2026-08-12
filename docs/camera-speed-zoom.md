# The speed-coupled field of view, and why it can't run backwards

The 3D chase and cockpit cameras (`src/render3d.rs`) pull back as the
boat gathers way by widening their **vertical field of view** — the
standard way to make a straight run read as motion, and on open water the
main thing that carries a sense of speed at all.

Done naively it produces a specific, very noticeable artifact **in the
moment of acceleration**: the scenery stops streaming past and starts
drifting back toward the horizon, which the eye reads as the camera (and
the boat with it) sliding backwards, exactly when the boat is
accelerating hardest.

This document states the condition under which that can never happen,
proves it, and records how `src/camera.rs` enforces it.

## Why it happens

A perspective camera has a vanishing point (strictly, a focus of
expansion: the screen point the direction of travel projects to). Moving
forward pushes every static feature *away* from it — that outward
streaming **is** the sensation of motion. Widening the FOV pulls every
feature *toward* it. Widen faster than the camera advances and the sum
runs the wrong way: the water, jetties and shores slide back toward the
horizon while the boat accelerates.

## Setup

For a vertical FOV $\varphi$, write

$$f = \cot(\varphi/2),$$

the projection scale: a point at camera-space $(X, Y, Z)$ (with $Z$
forward) lands at screen $\big(f X/Z,\; f Y/Z\big)$ in units where the
frame's vertical edges are $\pm 1$. Let the camera advance $d$ along its
own view axis over a frame ($d < 0$ = sternway), with no rotation, and
let primes denote the next frame.

## Theorem (no flow reversal)

A static point at depth $Z$ has its screen vector scaled by

$$\lambda \;=\; \frac{f'}{f}\cdot\frac{Z}{Z-d}. \tag{3}$$

The flow is outward iff $\lambda \ge 1$ and inward iff $\lambda \le 1$,
so the flow keeps the sign the camera's own motion gives it — outward
while advancing, inward while backing — for every static point at depth
at most $Z_{\text{reach}}$ **iff**

$$\boxed{\;f' \;\ge\; f\Big(1 - \frac{d}{Z_{\text{reach}}}\Big)\;\text{ when } d \ge 0,
\qquad f' \;\le\; f\Big(1 - \frac{d}{Z_{\text{reach}}}\Big)\;\text{ when } d \le 0.\;}$$

In FOV-angle terms, since $\mathrm{d}(\ln f)/\mathrm{d}\varphi = -1/\sin\varphi$, the
widening case reads

$$\Delta\varphi \;\le\; \sin(\varphi)\,\frac{d}{Z_{\text{reach}}}$$

to first order — *the FOV may open by at most $\sin\varphi$ radians per
$Z_{\text{reach}}$ metres the camera advances.*

### Proof

With no rotation, the static point's camera-space position goes from
$(X, Y, Z)$ to $(X, Y, Z-d)$, so its screen vector goes from
$f(X,Y)/Z$ to $f'(X,Y)/(Z-d)$ — the *same* vector scaled by
$\lambda$ as in (3). Two consequences, both used below: the flow is
purely radial about the focus of expansion, and $\lambda$ depends on the
point only through its depth $Z$.

Take $d \ge 0$ (advancing; the $d \le 0$ case is identical with the
inequalities reversed). Then

$$\lambda \ge 1
\iff \frac{f'}{f} \ge \frac{Z-d}{Z} = 1 - \frac{d}{Z}.$$

The right-hand side is increasing in $Z$, so requiring it at
$Z = Z_{\text{reach}}$ implies it for every $Z \le Z_{\text{reach}}$,
and conversely a violation at $Z_{\text{reach}}$ is a violation. $\blacksquare$

### Reading the result

* **Only one direction ever binds.** While advancing, *narrowing* the FOV
  raises $f$, which reinforces the outward flow — it cannot reverse
  anything, so it needs no bound at all. Backing, the roles swap. This is
  unlike the orthographic case, where both directions are constrained.
* **No bound protects every depth.** As $Z \to \infty$, $d/Z \to 0$
  while the zoom term stays finite: the far field *must* drift inward
  when you widen — that is what widening means. So the guarantee comes
  with an explicit reach, and beyond it the residual inward drift of a
  point at screen radius $|s|$ is
  $|s|\,\varepsilon\,(1 - Z_{\text{reach}}/Z)$ with
  $\varepsilon = 1 - f'/f$: zero at the reach, rising smoothly to the
  full zoom rate at infinity. This is the honest structural difference
  from the orthographic (top-down) projection, where the screen's own
  half-diagonal gives a finite, intrinsic reach and the bound protects
  every visible point without a chosen parameter.
* **$d$ is the camera's advance, not the boat's speed.** The chase rig
  deliberately hangs back under acceleration (`CHASE_POS_TAU`, the
  distance band), so the camera moves *slower* than the boat exactly when
  the FOV most wants to open. Measuring $d$ from the realized camera path
  charges that lag to the zoom automatically.
* **The boat is not a static feature.** It is allowed to recede in frame
  as the rig hangs back; the theorem is about the world, which is what
  carries the motion cue.

## What `src/camera.rs` does with it

`FovZoom::update` runs after the rig has resolved this frame's eye and
aim, so the advance it spends is the real one:

1. **Base.** The rig's own base FOV (chase 45°, cockpit 58°) is eased
   with `FOV_TAU` and is **exempt** — it changes only on a view-mode
   switch, which cuts the camera to a different place entirely, and there
   is no continuous optical flow across a cut to protect. (Same status a
   user-commanded zoom would have: their hand explains the motion.)
2. **Speed target.** `FOV_SPEED_GAIN_DEG` (5°) at `FOV_SPEED_REF`
   (3 m/s), capped at 1.5× that, eased with the same `FOV_TAU` so it
   reads as gathering way rather than as the throttle lever.
3. **Guarantee.** The eased value is then clamped into the interval the
   theorem allows at `FLOW_REACH_M` = 150 m, computed exactly in
   $f$-space (no small-angle approximation). At $d = 0$ the interval
   collapses to a point and the FOV is frozen.

`FLOW_REACH_M` is the one design parameter: 150 m covers the water, the
jetties, the moored fleet and both shores of a 125 m-wide channel —
everything whose sweep past the camera reads as speed. It prices the
zoom: opening the full 5° gain costs
$\varepsilon\,Z_{\text{reach}} \approx 0.12 \times 150 \approx 19$ m of
travel, about 6 s at cruise.

### Measured, before and after

A full-throttle start from the berth through the real `Sim`, with the
chase rig reproduced frame for frame (60 fps), reporting the depth beyond
which the flow reverses — i.e. everything *further away than this drifts
backwards*:

| into the run | SOG | old law | with the bound |
| --- | --- | --- | --- |
| first frames | ~0 | **everything** (the FOV opens while the camera has not yet moved) | — (frozen) |
| 3 s | 1.0 m/s | beyond **53 m** | ≥ 150 m |
| 6 s | 2.3 m/s | beyond 137 m | ≥ 150 m |
| 9 s | 2.8 m/s | beyond 472 m | ≥ 150 m |
| 12 s+ | 2.9 m/s | — (FOV settled) | — |

53 m is *past the boat itself* (the chase camera sits 22 m astern), so
under the old law the entire scene the helmsman is steering into crept
backwards for the first several seconds of every acceleration, while the
near water streamed correctly — the inconsistency is what makes it read
as a camera fault rather than as speed.

## Tests

| test | what it pins |
| --- | --- |
| `no_acceleration_can_make_the_scenery_drift_toward_the_horizon` | the headline property at four depths, both base FOVs, accelerations to 50 m/s² |
| `sternway_bounds_narrowing_instead_of_widening` | the mirrored case: backing contracts, so narrowing is what is bounded |
| `a_camera_that_is_not_advancing_cannot_widen` | the $d = 0$ degenerate case |
| `the_full_speed_gain_costs_the_travel_the_bound_prices_it_at` | the 19 m price, from both sides |
| `the_fov_reaches_its_speed_target_and_returns`, `a_stopped_boat_settles_back_to_the_base_fov` | the bound is a cap, not the behaviour |
| `a_view_mode_switch_glides_the_base_without_needing_travel` | the exempt base glide |
| `a_respawn_teleport_does_not_pay_for_fov` | a teleport is not travel |

The checks evaluate $\lambda$ from (3) directly on the FOV pair the
renderer would use, at depths from 5 m to the reach; since $\lambda$ is
monotone in $Z$, the endpoints bound everything between. They hold to
`1e-6`, which is the `f32` round-trip precision of
$\varphi \to f \to \varphi$ — at the very start of a run the frame-to-frame
numbers involved are a 0.06 mm advance and a 0.0001° FOV step, where
rounding, not the bound, is the limit.
