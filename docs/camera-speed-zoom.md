# Speed-adaptive camera distance, and why it can't run backwards

Both of this sim's cameras pull back as the boat gathers way — the
top-down view widens its visible width, the 3D chase and cockpit views
widen their field of view. More reach at speed, more detail when
manoeuvring. Done naively, both produce the same very noticeable
artifact **in the moment of acceleration**: the scenery stops streaming
past and starts creeping the wrong way, which the eye reads as the
camera (and the boat with it) sliding backwards, exactly when the boat
is accelerating hardest.

This document states the condition under which that can never happen,
proves it in both projections, and records how `src/camera.rs` enforces
it.

* [Part 1 — the orthographic (top-down) camera](#part-1--orthographic-top-down)
  — `SpeedZoom`, the bound `ρ·|ΔW| ≤ δ`.
* [Part 2 — the perspective (chase/cockpit) camera](#part-2--perspective-chasecockpit)
  — `FovZoom`, the bound `f' ≥ f·(1 − d/Z)`.

The orthographic case is the simpler of the two and worth reading first;
the perspective one is where the artifact was actually reported.

# Part 1 — orthographic (top-down)

## Why it happens

The camera is centred on the boat, so the boat is nailed to the screen
centre and the only motion cue on screen is the scenery streaming past.
Zooming out pulls every feature *toward* the screen centre — and for
everything astern, "toward the centre" is *forwards*. Two effects
therefore fight over the features behind the boat:

* the boat's own travel, which pushes them backwards, at a rate set by
  **speed × scale**;
* the widening view, which pulls them forwards, at a rate set by **how
  far astern they are × how fast the scale is shrinking**.

The second wins whenever the zoom-out is fast relative to the boat's
travel — which is precisely what a naive "visible width follows speed"
rule does during hard acceleration, since speed is then changing much
faster than position. Note that the failure is *not* caused by zooming
out per se, and is *not* fixed by smoothing per se: a lag filter still
widens arbitrarily fast if the target moves far enough. What is needed is
a bound tying the zoom rate to the camera's actual travel.

## Setup and notation

Per rendered frame $n$:

* $c_n \in \mathbb{R}^2$ — camera world position (metres). This is the
  *realized* camera: boat pose, plus the pan offset, after the world-rect
  clamp.
* $k_n > 0$ — scale in px per metre.
* $W_n = s_w / k_n$ — visible width in metres, $s_w, s_h$ the screen size
  in px.
* $R_n$ — the visible **half-diagonal** in metres, i.e. the radius of the
  circle circumscribing the visible rect:
  $R_n = \tfrac{1}{2}\sqrt{s_w^2 + s_h^2}\,/\,k_n = \rho\,W_n$ with
  $$\rho = \tfrac{1}{2}\sqrt{1 + (s_h/s_w)^2}.$$
  Every point on screen is within $R_n$ of $c_n$. ($\rho \approx 0.57$ at
  16:9 landscape, $\approx 0.95$ at 9:16 portrait.)
* $\delta = \lVert c_{n+1} - c_n \rVert$ — the camera's travel over the
  frame, and $\hat u$ its direction (defined when $\delta > 0$).

A world-static point $p$ has scaled screen offset

$$X_n = (p - c_n)\,k_n$$

(the actual screen position is $X_n$ flipped in $y$ and translated to the
screen centre — an isometry, so "forwards" means the same thing in both).
Write $\xi_n = (p - c_n)\cdot\hat u$ for how far ahead of the camera the
point lies, negative astern.

**The artifact** is a static feature moving *forwards* on screen, i.e.
$\Delta X \cdot \hat u > 0$ where $\Delta X = X_{n+1} - X_n$.

## Theorem (no forward flow)

Let $\Delta W = W_{n+1} - W_n$. If

$$\boxed{\;\rho\,\lvert \Delta W \rvert \;\le\; \delta\;}$$

then no world-static point visible at frame $n$ moves forwards:
$\Delta X \cdot \hat u \le 0$ for every $p$ with
$\lVert p - c_n \rVert \le R_n$.

More precisely, if $\rho\,\lvert\Delta W\rvert \le \mu\,\delta$ for some
$\mu \in [0,1]$, then for every such point

$$\Delta X \cdot \hat u \;\le\; -(1-\mu)\,\delta\,k_{n+1} \;\le\; 0,$$

i.e. every visible feature keeps at least a $(1-\mu)$ fraction of the
backward screen flow it would have had at a frozen zoom — and the same
conclusion holds out to $\lVert p - c_n \rVert \le R_n/\mu$, well beyond
the screen edge.

### Proof

Since $p$ is static,

$$
\Delta X = (p - c_{n+1})k_{n+1} - (p - c_n)k_n
        = (p - c_n)\,\Delta k \;-\; (c_{n+1} - c_n)\,k_{n+1},
$$

with $\Delta k = k_{n+1} - k_n$ (add and subtract $(p-c_n)k_{n+1}$).
Projecting on $\hat u$ and using $(c_{n+1}-c_n)\cdot\hat u = \delta$:

$$\Delta X \cdot \hat u = \xi_n\,\Delta k \;-\; \delta\,k_{n+1}. \tag{1}$$

The first term is the zoom's contribution, linear in $\xi_n$; the second
is the streaming from the camera's own travel, the same for every point.

Bound the first term. Whatever the sign of $\Delta k$,

$$\xi_n\,\Delta k \;\le\; \lvert \xi_n\rvert\,\lvert\Delta k\rvert
  \;\le\; R_n\,\lvert\Delta k\rvert$$

for any visible point (a point on screen has
$\lvert\xi_n\rvert \le \lVert p - c_n\rVert \le R_n$). Now the key
identity — exact, no linearization. From $k = s_w/W$,

$$\Delta k = \frac{s_w}{W_{n+1}} - \frac{s_w}{W_n}
          = -\,\frac{s_w\,\Delta W}{W_n W_{n+1}},
\qquad k_{n+1} = \frac{s_w}{W_{n+1}},$$

so

$$\frac{\lvert\Delta k\rvert}{k_{n+1}} = \frac{\lvert\Delta W\rvert}{W_n}. \tag{2}$$

Substituting $R_n = \rho W_n$ and (2):

$$R_n\,\lvert\Delta k\rvert
 = \rho\,W_n \cdot k_{n+1}\frac{\lvert\Delta W\rvert}{W_n}
 = \rho\,k_{n+1}\,\lvert\Delta W\rvert
 \;\le\; \mu\,\delta\,k_{n+1}$$

by hypothesis. Putting that into (1),

$$\Delta X\cdot\hat u \;\le\; \mu\,\delta\,k_{n+1} - \delta\,k_{n+1}
  = -(1-\mu)\,\delta\,k_{n+1} \;\le\; 0. \qquad\blacksquare$$

The extension past the screen edge is the same computation with
$\lvert\xi_n\rvert \le R_n/\mu$, which leaves
$\Delta X\cdot\hat u \le 0$.

### Reading the result

* **The bound contains no $W$.** It is a statement about *rates*, not
  about how far out the camera is: the visible width may change by at
  most $1/\rho \approx 1.7$ metres per metre the camera travels
  (landscape; $\approx 1.05$ in portrait, where the diagonal is a larger
  multiple of the width). Any zoom level, any user zoom, same bound.
* **No frame rate, speed or acceleration appears either.** The
  discrete-time statement is exact, so nothing is assumed about how
  smooth the motion is between frames, and no acceleration — however
  violent, including a collision — can break it.
* **It degenerates correctly.** $\delta = 0$ (boat stopped, or the camera
  pinned by the world-rect clamp) forces $\Delta W = 0$: a camera that
  isn't travelling may not change its zoom at all, because there is no
  backward flow left to stay under.
* **It is symmetric.** Widening ($\Delta W > 0$) is bounded because
  features *astern* would run forwards; narrowing is bounded because
  features *ahead* would. Deceleration gets the same protection as
  acceleration.
* **It is tight.** At $\mu = 1$ the worst-case corner feature is exactly
  stationary, so no larger zoom step is admissible. A design that wants a
  faster pull-back has to travel further, not step harder.

## What `src/camera.rs` does with it

`SpeedZoom::update` holds a multiplier $m \in [1, \texttt{SPEED\_ZOOM\_MAX}]$
on the user-selected visible width $W_1$ (so $W = W_1 m$), and each frame:

1. **Target.** $m^\* = 1 + (M-1)\cdot\mathrm{clamp}\big((v -
   \texttt{LO})/(\texttt{HI}-\texttt{LO})\big)$ from speed-over-ground,
   capped so the total width never passes the user zoom's own outer limit.
2. **Feel.** A low-pass toward $m^\*$ that e-folds over
   `SETTLE_M` metres **of camera travel** rather than seconds — the
   "momentum" the artifact suggests, expressed in the same currency as the
   bound.
3. **Guarantee.** The step is then hard-capped at
   $\mu\,\delta/(\rho W_1)$ with $\mu = \texttt{FLOW\_MARGIN} = 0.5$,
   which is the theorem's hypothesis written for $\Delta W = W_1\Delta m$.
   This cap, not the low-pass, is what carries the proof.

$\mu = \tfrac12$ is not a fudge factor: by the theorem it means every
visible feature keeps at least **half** the backward flow it would have
at a frozen zoom, and the no-forward-motion guarantee extends to
$2R_n$ — so nothing can drift forwards *into* frame from off-screen
astern either.

### Where the hypothesis is met exactly

$\delta$ is measured from the camera positions actually rendered, so the
proof covers the real camera including the pan offset and the world-rect
clamp, not an idealization of it. `src/main.rs` therefore resolves the
camera *position* first — using the previous frame's width for the
world-rect margins — then chooses the width against that frame's true
$\delta$. The only cost is that those margins lag the width by one step,
i.e. by at most $\mu\delta/(2\rho)$ ≈ a couple of centimetres of world,
against a 6 m pad that is re-clamped every frame.

### Deliberate exemptions

* **The user's own zoom** (pinch, wheel, `+`/`-`) changes $W_1$ directly
  and is not rate-limited. Their hand is the explanation for the motion;
  a camera that resisted the pinch would be the bug.
* **Window resize** likewise changes $W_1$ and $\rho$ outside the model.
* **Idle relaxation.** With $\delta = 0$ the theorem freezes the zoom, so
  a boat that stops dead (a crash stop against a jetty) would keep a
  pulled-back view until it moved again. `IDLE_RELAX` lets the view creep
  back **in** — never out — at up to 0.12 multiplier/s, faded out
  entirely by `SPEED_ZOOM_LO` (0.5 m/s). Below that speed the camera is
  effectively static, there is no streaming scenery for a zoom to
  contradict, and the direction $\hat u$ the theorem is stated in is not
  even well defined. Above `SPEED_ZOOM_LO` — the whole regime where the
  artifact exists — the bound is the only thing in force. Widening is
  never granted this allowance, and cannot want it: the target is
  $m^\* = 1$ (the close-up) at any speed below `SPEED_ZOOM_LO`.

## Tests (Part 1)

`src/camera.rs`'s test module pins the properties above against the real
update:

| test | what it pins |
| --- | --- |
| `no_acceleration_can_make_the_scenery_run_forwards` | the headline property, swept from a gentle spool-up to a 50 m/s² slam |
| `the_guarantee_extends_to_twice_the_visible_radius` | the $\mu = \tfrac12$ corollary: nothing drifts forwards into frame |
| `decelerating_never_pushes_the_scenery_forwards` | the mirror case (features ahead) on the way down |
| `the_width_change_never_exceeds_the_travel_bound` | $\rho\lvert\Delta W\rvert \le \delta$ directly |
| `a_stationary_camera_can_never_widen` | the degenerate case, incl. a world-clamped camera at full speed |
| `the_zoom_still_reaches_its_target_and_returns`, `a_stopped_boat_relaxes_back_to_the_close_up` | the bound is a cap, not the behaviour |
| `the_speed_zoom_is_relative_to_the_user_zoom`, `the_speed_zoom_respects_the_outer_zoom_limit` | composition with the user zoom |
| `a_respawn_teleport_does_not_pay_for_a_zoom_step` | a teleport is not travel |

The screen-flow checks evaluate (1) at $\xi = \pm\text{reach}\cdot R_n$
each frame; since (1) is linear in $\xi$, the two extremes bound every
point in between, so this is a check of the property itself and not a
sample of it.

# Part 2 — perspective (chase/cockpit)

The 3D views (`src/render3d.rs`) pull back by widening their **vertical
field of view** with speed — the standard way to make a straight run read
as motion, and on open water the main thing that carries a sense of speed
at all. The artifact is the same one, and this is where it was actually
reported.

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
  from Part 1, where the screen's own half-diagonal $R_n$ gives a finite,
  intrinsic reach and the bound protects every visible point without a
  chosen parameter.
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
   is no continuous optical flow across a cut to protect. (Same status as
   the user's pinch in Part 1.)
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

## Tests (Part 2)

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
