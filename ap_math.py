"""Align Planes - the transform, in plain Python (no adsk, no numpy).

A plane-on-plane alignment fixes three of the six degrees of freedom: the tilt
(two) and the distance along the normal (one). The other three - sliding in the
plane and turning about its normal - are free, and this module spends nothing on
them: the rotation is the SMALLEST one that brings the moving normal onto the
fixed one, and it turns about a pivot on the moving plane, so the part travels
as little as the alignment allows.

Points are (x, y, z) tuples in any one unit; normals are unit vectors.
"""

import math


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def unit(a):
    n = math.sqrt(dot(a, a))
    if n == 0:
        raise ValueError('zero vector')
    return (a[0] / n, a[1] / n, a[2] / n)


def matvec(R, v):
    return tuple(R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2] for i in range(3))


def perpendicular(n):
    """Any unit vector square to n."""
    h = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    return unit(cross(n, h))


def rotation(axis, angle):
    """Rodrigues: rotation by `angle` (rad) about the unit `axis`, as 3x3 rows."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def project(q, p, n):
    """q projected onto the plane through p with normal n."""
    return sub(q, scale(n, dot(sub(q, p), n)))


def align(moving, fixed, pivot_hint=None, flip=False, offset=0.0, spin_axis=None):
    """The rigid transform that puts the moving plane on the fixed one.

    moving, fixed: (point, unit normal).
    pivot_hint: where the moved things are (their centre); the pivot is its
        projection onto the moving plane, so they turn about the nearest point
        of the plane rather than about some far-off plane origin. None: the
        moving plane's own point.
    flip: by default the moving normal is brought onto whichever of +/- the
        fixed normal is nearer (the least rotation). flip takes the other one -
        the part turned over onto the same plane.
    offset: the moving plane ends this far from the fixed one, along the fixed
        plane's normal.
    spin_axis: for the 180 degree case only, the axis to turn about (must lie in
        the moving plane); otherwise any axis in the plane.

    Returns a dict: R (3x3 rows), t (translation), so x -> R x + t; and what it
    does - the angle in degrees, the pivot, the distance travelled along the
    fixed normal, and which side the normals ended on.
    """
    pm, nm = moving[0], unit(moving[1])
    pf, nf = fixed[0], unit(fixed[1])
    same = dot(nm, nf) >= 0.0
    if flip:
        same = not same
    target = nf if same else scale(nf, -1.0)
    c = max(-1.0, min(1.0, dot(nm, target)))
    angle = math.acos(c)
    ax = cross(nm, target)
    if math.sqrt(dot(ax, ax)) < 1e-12:
        if c > 0:
            R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            angle = 0.0
        else:
            u = unit(project(spin_axis, (0.0, 0.0, 0.0), nm)) if spin_axis else perpendicular(nm)
            R = rotation(u, math.pi)
            angle = math.pi
    else:
        R = rotation(unit(ax), angle)
    pivot = project(pivot_hint, pm, nm) if pivot_hint is not None else pm
    # after turning about the pivot it is still on the (turned) moving plane;
    # slide it along the fixed normal onto the fixed plane, plus the offset
    travel = dot(sub(pf, pivot), nf) + offset
    T = scale(nf, travel)
    t = add(sub(pivot, matvec(R, pivot)), T)
    return {'R': R, 't': t, 'angle_deg': math.degrees(angle), 'pivot': pivot,
            'travel': travel, 'same_side': same}


def apply(res, p):
    return add(matvec(res['R'], p), res['t'])


def relation(a, b):
    """How plane a sits against plane b: the angle between their normals
    (0..90 deg, sign-free) and the distance of a's point from b."""
    na, nb = unit(a[1]), unit(b[1])
    x = cross(na, nb)
    # atan2, not acos: acos near 1 cannot resolve angles below ~1e-6 deg
    ang = math.degrees(math.atan2(math.sqrt(dot(x, x)), abs(dot(na, nb))))
    return ang, dot(sub(a[0], b[0]), nb)
