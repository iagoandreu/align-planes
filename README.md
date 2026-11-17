# Align Planes

A Fusion 360 add-in that brings a **moving plane onto a fixed plane** and carries
any number of bodies and components with it on the same rigid transform. Each
plane is a construction plane or a planar face. Typical uses: a saw blade or a
cutting slot whose plane has to land on a planned osteotomy plane, or a plate whose
contact face has to sit on a resection plane.

A plane on a plane fixes only the **tilt** and the **distance**. The slide within
the plane and the turn about its normal are left as they are: the part turns the
least it can, about the point of the moving plane nearest to it, and does not
slide.

## How to use

### Where the button is

**Design** workspace: **Solid** tab, **Modify** panel, and **Mesh** tab,
**Modify** panel. The button is **Align Planes**. It is not promoted, so open the
**Modify** drop-down to find it.

### Before you start

- **A fixed plane**: the plane that stays. A construction plane or a planar face of
  a solid.
- **A moving plane**: the plane to bring onto it. A construction plane or a planar
  face.
- **What should move**: if the moving plane is a face, its body moves
  automatically. If it is a construction plane, select the bodies or components
  that go with it; a construction plane moves nothing by itself.
- **No external Python.** Everything runs inside Fusion's own Python.

### Step by step

1. Click **Align Planes**. The OK button is labelled **Align**.
2. **Fixed plane**: select one construction plane or planar face.
3. **Moving plane**: select one construction plane or planar face, different from
   the fixed one.
4. **Move along**: select the solid, mesh or surface bodies and the components
   that travel with the moving plane. Any number. Optional when the moving plane is
   a face (its body is added for you).
5. Set **Flip** and **Offset** if needed (table below).
6. Read the text box. It previews the result before anything moves:
   - **now**: the angle between the planes and the distance, in deg and mm;
   - **turn**: how far the part will rotate, in deg;
   - **shift**: how far it will move along the fixed plane's normal, in mm;
   - **moves**: everything that will move.
7. Click **Align**. The report opens.

The **Align** button stays disabled until both planes are selected and there is
something to move.

| Input | Default | What it does |
|---|---|---|
| Fixed plane | (empty) | The plane that stays. Construction plane or planar face. One. |
| Moving plane | (empty) | The plane to bring onto it. Construction plane or planar face. One. |
| Move along | (empty) | Solid bodies, mesh bodies, surface bodies and components that travel with the moving plane. Any number. |
| Flip | off | By default the part turns the least it can, so the normals end up pointing whichever way is nearer (same way or opposite). Flip turns the part over onto the same plane instead (the other way round). |
| Offset | 0 mm | The moving plane ends this far from the fixed plane, measured along the fixed plane's normal. For a planar face that normal points out of the body, so a positive offset leaves a gap on the outside of the fixed face. |

Settings are not saved between runs; each time the dialog opens with these
defaults.

### What you get

- The moved bodies get one **Move** feature named **Align Planes** per component
  and per body type (mesh, solid), and a separate one for solids inside a base
  feature. Components are moved by changing their position.
- If the moving plane is a face, its body moves with it, so the face ends on the
  fixed plane.
- If the moving plane is a construction plane, it only moves if it is defined by
  something that moved. Otherwise it stays where it was, and the report says so.
- The report opens in a message box and is appended to `AlignPlanes.log` beside
  the installed add-in.

### Checking the result

The report reads:

- **BEFORE**: the angle between the two planes (deg) and the distance of the
  moving plane from the fixed plane (mm).
- **TRANSFORM**: the rotation (deg), the shift along the fixed normal (mm), the
  pivot point on the moving plane (mm), whether the normals now point the same way
  or face each other, and *(flipped)* when Flip was on.
- **AFTER**: angle and distance after the move. This line is computed from the
  transform, so it shows what the transform does, not a measurement of the bodies.
- **MOVED**: one line per entity, **measured** after the move: `ok (off ... mm)`,
  `DID NOT MOVE`, `moved wrongly`, or `not measurable`. Any entity that could not
  be moved is listed with the reason.

Before you trust it:

1. Check that every line under **MOVED** says `ok`. That is the measured check.
2. Check **normals now ...**: if the part ended up facing the wrong way, undo
   (Ctrl+Z) and run again with **Flip** changed.
3. If you used **Offset**, check the sign: measure the gap with Fusion's
   **Inspect > Measure**.
4. Remember that only tilt and distance were set. Position within the plane and
   rotation about the normal are unchanged; set them separately if they matter.

### Where it fits in a patient-specific workflow

Use it after the bone is registered and converted to a solid, when an instrument
or implant has to sit on a planned plane: a saw blade or a guide's cutting slot on
the osteotomy plane, a plate on the resection surface. The plane alignment is
exact; the in-plane position is still yours to set.

### Known limits

- Only planes: a construction plane or a planar face. A curved face is refused.
- A face's plane point is the face centroid; the distance in **BEFORE** is
  measured from that point.
- Construction geometry cannot be moved by a move feature in a parametric design,
  so a construction plane used as the moving plane does not follow unless it is
  defined by something that moved. To carry construction geometry, put it in a
  component and select the component.
- When the two planes are exactly opposite and **Flip** turns the part 180 deg,
  the turn is about the moving plane's own u direction.

## How the transform is chosen

`ap_math.align` returns the smallest rotation that makes the moving normal parallel
to the fixed normal (or anti-parallel, whichever is nearer; **Flip** takes the
other one), followed by a shift along the fixed normal.

The pivot is not the plane's own origin, which can be far away. It is the centre
of everything that moves (solid centre of mass, mesh node centroid), projected onto
the moving plane. The part therefore turns about its own foot on the plane and
travels as little as the alignment allows.

`ap_math` is plain Python, no numpy and no `adsk`, so it is tested offline.

## How it moves things

The move code follows the one in Global Registration, where each trap was
measured:

- **One move feature per (component, kind, owning base feature).** A collection
  that mixes mesh bodies and solids moves the meshes and silently leaves the solids
  behind, with no error from the API.
- **A solid inside a base feature** needs `targetBaseFeature` set and the base
  feature reopened, or the move is refused.
- **Occurrences** get the transform composed onto their own placement.
- **The matrix is checked before anything moves.** It is built as a coordinate
  system (origin `t`, axes the columns of `R`), then three probe points are pushed
  through Fusion's own `Point3D.transformBy` and compared with the arithmetic done
  here. A mismatch refuses the move.
- **Every entity is checked after the move** against where the transform should
  have put it, using a point the move takes exactly (a solid's centre of mass, a
  mesh body's own nodes), read live rather than from the cached display mesh.

## Install

1. Clone or copy this folder anywhere.
2. Run `powershell -File install.ps1`. It copies the add-in into
   `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\AlignPlanes`.
3. In Fusion: **Utilities → ADD-INS → Scripts and Add-Ins → Add-Ins** →
   `Align Planes` → tick **Run on Startup** → **Run**.
4. The button appears in **Solid → Modify** and **Mesh → Modify**.

No dependencies. It runs entirely inside Fusion's bundled Python, stdlib only.

## Repository layout

```
AlignPlanes.py     add-in shell: lifecycle and the dev bridge
ap_core.py         reads the planes, applies the transform, the dialog; hot-reloads
ap_math.py         the transform, plain Python, no adsk
install.ps1        push into Fusion's AddIns folder
resources/         toolbar icons
tools/             dev-bridge client
tests/             offline tests, no Fusion required
```

`ap_core.py` and `ap_math.py` **hot-reload**: they are re-imported on every
command, so pushing them takes effect on the next click. Only `AlignPlanes.py`
needs a Stop/Run.

## Tests

```
py -3.10 tests/test_math.py
```

11 checks, no Fusion needed: 500 random plane pairs (the transform is a proper
rotation, the moved plane ends parallel to the fixed one and at the asked offset,
and the pivot does not slide in the plane, all to 1e-9), plus the least-motion
cases: parallel planes give a pure shift, opposite normals need no turn, Flip turns
180 deg, a 45 deg tilt turns by 45 deg, and the turn is about the part's foot on
the plane rather than the plane origin.

```
py -3.10 tools/ap.py ping
py -3.10 tools/ap.py inspect
py -3.10 tools/ap.py align '{"fixed": "Plane1", "moving": "Plane2", "along": ["Saw"]}'
```

runs it inside a live Fusion through the dev bridge. Off by default: create a
`devbridge.on` file next to the installed add-in.

## Licence

MIT — see [LICENSE](LICENSE).
