# VPE Refactoring Plan — No Macros, Strong Types

Goal: modernize `include/VPE.hpp` (~3,600 LOC) for code quality: eliminate all
function-like macros, replace weakly typed constructs (`void*`, raw scalars with
hidden invariants, magic ints) with strong types, improve encapsulation — while
preserving simulation behavior exactly.

A compilable prototype of the central idea (typed coordinate spaces) lives in
`proposals/typed_spaces/`. Build and run:

```
g++ -std=c++20 -I extern proposals/typed_spaces/clip_face_face_demo.cpp -o demo && ./demo
```

---

## Phase 0 — Safety net (before touching anything)

1. **Golden-run regression tests.** Deterministic scenes (fixed seed, fixed
   `m_sim_frequency`): pyramid stack, single hinge with motor, ball-socket chain,
   cloth sheet. Run N ticks, serialize all body positions/orientations, commit as
   golden files. Every later phase must reproduce them bit-exactly (same
   precision, same compiler) or explain the diff.
2. **Tooling.** Add to CMake: `clang-format` config, `clang-tidy`
   (`modernize-*`, `cppcoreguidelines-*`, `readability-*`),
   `-Wall -Wextra -Wconversion` (warnings-as-errors once clean), CI job that
   builds tests + runs golden runs on MSVC and gcc/clang.
3. Commit the currently uncommitted changes on `V2` first (CMakeLists.txt,
   VPE.hpp, tests) so refactoring starts from a clean, tagged baseline.

**Exit criterion:** green CI, golden files committed, `git tag pre-refactor`.

**Status: implemented.** `tests/goldenrun.cpp` (scenes: stack, pendulum, hinge,
cloth; `--generate`/`--compare`), golden files in `tests/golden/`, `.clang-format`,
`.clang-tidy`, warning flags (`VPE_WARNINGS_AS_ERRORS` option), CI in
`.github/workflows/ci.yml`.

Findings from implementation:

- **Determinism requires ASLR off.** Contact pairs are keyed by body addresses
  (`void*`), so address layout changes `unordered_map` iteration order, hence
  impulse application order, hence float results — the same binary produces
  different stack-scene results per run. CTest therefore runs goldenrun under
  `setarch -R` on Linux. This is a concrete correctness argument for Phase 2:
  index-based handles make iteration order address-independent.
  Confirmed on macOS too (no ASLR opt-out exists there), so `build_cmake.sh`
  skips goldenrun on Darwin — the Linux CI job is the golden-run authority
  until Phase 2 makes the simulation address-independent.
- The golden files are reference-environment artifacts (ubuntu/gcc/Release);
  the MSVC and clang CI jobs run unit tests only.
- Two portability fixes were needed to build with gcc at all (VPE.hpp compiled
  only with MSVC transitive includes/overloads): added `#include <memory>`, and
  replaced `fabs` as a `real(*)(real)` argument in `createEdgeContact` with a
  lambda (glibc's `::fabs` is `double(double)` only).

## Phase 1 — Transform macros → typed coordinate spaces

Replace the 17 macros (`ITORP`, `RTOWN`, `WTOTIP`, …, VPE.hpp lines 203–223)
with the tagged types from `proposals/typed_spaces/vpe_spaces.hpp`:

- `enum class Space { World, Ref, Inc, RefTangent, IncTangent }`
- `Point<S>`, `Vec<S>`, `Normal<S>` — one `glmvec3` each, zero overhead
- `Transform<From, To>` — the 4×4 plus its normal (inverse-transpose) matrix;
  composition `operator*` and `.inv()` keep space tags consistent
- `ContactFrame` — replaces `Contact::BodyPtr::m_to_other / m_to_other_it`;
  built once per contact pair
- `dot`/`cross`/`operator-` only accept operands from the same space

What the compiler then catches: applying a transform to a point from the wrong
space, using a point transform on a normal, mixing spaces in dot products.
Today these are prevented only by macro naming discipline.

Scope: `sat_query`, `queryFaceDirections`, `queryEdgeDirections`, `SAT`,
`createFaceContact`, `createEdgeContact`, `clipFaceFace`, `positionBias`,
contact-point storage in `Contact::ContactPoint`. See macro→typed mapping table
at the top of `vpe_spaces.hpp`; `clip_face_face_demo.cpp` shows `clipFaceFace`
fully rewritten (side effect: hoists two matrix products out of per-vertex
loops that the macros silently recomputed).

**Exit criterion:** zero `#define` with arguments in VPE.hpp; golden runs
unchanged.

## Phase 2 — `void*` owners → strong handles

- Replace `body_map = MapWrapper<void*, shared_ptr<Body>>` and
  `unordered_map<void*, ...>` for cloth/collider callbacks with
  `BodyHandle` / `ClothHandle`: `struct BodyHandle { uint32_t index, generation; }`
  over slot-map storage. Fixes type safety, gives O(1) stable lookup, makes
  dangling detection explicit (generation check), and is serializable.
- `voidppair_t` (broadphase pair key) → `struct BodyPair { BodyHandle a, b; }`
  with canonical ordering + hash.
- The renderer-owner association (`m_owner`) becomes an opaque
  `OwnerId` newtype or stays a user-supplied `uint64_t`; the engine never
  dereferences it, so it should not be a pointer type.

**Exit criterion:** no `void*` in VPE.hpp.

## Phase 3 — Scalar newtypes and enums

- `m_mass_inv` (`0` secretly means infinite mass) →
  `class InverseMass { real value; static InverseMass infinite(); bool isStatic() const; }`.
  Also subsumes the `m_body1_factor` workaround in `Constraint` (the
  epsilon-threshold static check becomes `isStatic()`).
- `Restitution`, `Friction` newtypes clamping to valid range in the constructor.
- Time: unify the `double dt` / `real` mix on `std::chrono::duration<real>`;
  `m_sim_frequency` → a `Hertz` newtype or `duration` directly.
- Angles in `HingeJoint` limits → `Radians` newtype.
- `m_solver` (0/1) → `enum class Solver { Combined, Separate }`;
  edge signs ±1 in `Polytope` faces → `enum class Winding { CCW, CW }`;
  feature-flag ints (`m_use_vbias`, `m_use_warmstart`, …) → `bool` or a
  `constexpr`-friendly `WorldConfig` struct grouping all tunables.

**Exit criterion:** no raw `real` parameter whose meaning depends on a comment.

## Phase 4 — Encapsulation and pointers

- `Body`: make computed state (`m_model`, `m_model_inv`, `m_model_it`,
  `m_inertiaW`, `m_inertia_invW`, `m_grid_x/z`) private; public setters
  `setPosition/setOrientation/setVelocity` keep matrices consistent
  (eliminates the "only change if you know what you are doing" comment class
  of bugs).
- `Polytope` internals: `Vertex*/Edge*/Face*` cross-references → indices into
  the polytope's own vectors (`VertexIdx`, `EdgeIdx`, `FaceIdx` newtypes).
  Stable under copy, cache-friendly, serializable.
- Polytope ownership: `Body::m_polytope` raw pointer → `shared_ptr<const Polytope>`
  or a `PolytopeHandle` into a world-owned registry (polytope data is immutable
  shared geometry — make that explicit with `const`).
- Reconsider `Body::m_physics` back-pointer: pass `VPEWorld&` (or just the
  needed config) into the few methods that use it.

## Phase 5 — Structure

- Split VPE.hpp internally: `vpe/math.hpp` (real, glm aliases, spaces),
  `vpe/polytope.hpp`, `vpe/body.hpp`, `vpe/broadphase.hpp`, `vpe/narrowphase.hpp`
  (SAT + clipping), `vpe/solver.hpp` (contacts + warm start),
  `vpe/constraints.hpp`, `vpe/cloth.hpp`, `vpe/world.hpp`. Keep `VPE.hpp` as
  umbrella header so user code is unaffected.
- Last (touches every signature): `VPE_SINGLE/DOUBLE_ACCURACY` preprocessor
  switch → `template<std::floating_point Real> class VPEWorld`, with
  `using VPEWorldF = VPEWorld<float>;` defaults. Enables float/double
  cross-checking in tests. Skip if compile-time cost is judged too high for
  the course setting.

## Order and risk

Phases 1–3 are independent of each other after Phase 0 and can be merged
separately; each is mechanically verifiable against the golden runs. Phase 4
changes API surface (examples + Vulkan engine integration must be adapted).
Phase 5 is optional polish.

Main risk: golden runs are compiler/precision sensitive — pin one
compiler+flags combination as the reference. Main cost: typed spaces add
ceremony in narrowphase code; the prototype suggests readability improves
rather than degrades (macro call chains become named, composable transforms).
