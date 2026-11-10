"""Offline checks of the plane alignment.  py -3.10 tests/test_math.py"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ap_math as A                                            # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    print('  %s %s%s' % ('ok  ' if cond else 'FAIL', name,
                         '   %s' % (detail,) if detail != '' else ''))
    if not cond:
        FAILED.append(name)


def rand_unit(rng):
    return A.unit((rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1)))


def rigid(res):
    R = res['R']
    ortho = max(abs(sum(R[i][k] * R[j][k] for k in range(3)) - (i == j))
                for i in range(3) for j in range(3))
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    return ortho < 1e-12 and abs(det - 1.0) < 1e-12


rng = random.Random(3)
worst_ang = worst_d = worst_slide = 0.0
all_rigid = True
for _ in range(500):
    pm = tuple(rng.uniform(-500, 500) for _ in range(3))
    pf = tuple(rng.uniform(-500, 500) for _ in range(3))
    nm, nf = rand_unit(rng), rand_unit(rng)
    hint = tuple(rng.uniform(-500, 500) for _ in range(3))
    off = rng.uniform(-5, 5)
    res = A.align((pm, nm), (pf, nf), hint, offset=off)
    all_rigid &= rigid(res)
    # the moved plane: its point and normal through the transform
    p2 = A.apply(res, pm)
    n2 = A.matvec(res['R'], nm)
    ang, d = A.relation((p2, n2), (pf, nf))
    worst_ang = max(worst_ang, ang)
    worst_d = max(worst_d, abs(d - off))
    # the pivot only travels along the fixed normal
    piv = res['pivot']
    mv = A.sub(A.apply(res, piv), piv)
    slide = A.sub(mv, A.scale(nf, A.dot(mv, nf)))
    worst_slide = max(worst_slide, math.sqrt(A.dot(slide, slide)))
print('random planes')
check('500 random pairs: the transform is a rotation', all_rigid)
check('the moved plane is parallel to the fixed one', worst_ang < 1e-9, worst_ang)
check('and at the asked offset from it', worst_d < 1e-9, worst_d)
check('the pivot does not slide in the plane', worst_slide < 1e-9, worst_slide)

print('least motion')
res = A.align(((0, 0, 0), (0, 0, 1)), ((0, 0, 10), (0, 0, 1)))
check('parallel planes: a pure shift of 10', res['angle_deg'] == 0.0
      and A.apply(res, (3, 4, 0)) == (3, 4, 10.0), A.apply(res, (3, 4, 0)))
res = A.align(((0, 0, 0), (0, 0, 1)), ((0, 0, 10), (0, 0, -1)))
check('opposite normals are already aligned: no turn', res['angle_deg'] == 0.0
      and not res['same_side'])
res = A.align(((0, 0, 0), (0, 0, 1)), ((0, 0, 10), (0, 0, -1)), flip=True)
check('flip turns it over (180 deg)', abs(res['angle_deg'] - 180.0) < 1e-12
      and res['same_side'])
n2 = A.matvec(res['R'], (0, 0, 1))
check('and the normal then points the fixed way', abs(n2[2] + 1.0) < 1e-12, n2)
res = A.align(((0, 0, 0), (0, 0, 1)), ((0, 0, 0), A.unit((0, 1, 1))))
check('a 45 deg tilt turns by 45 deg', abs(res['angle_deg'] - 45.0) < 1e-12)
res = A.align(((0, 0, 0), (0, 0, 1)), ((0, 0, 0), A.unit((0, 1, -1))))
check('...and an obtuse one by 45 deg too, onto the other side',
      abs(res['angle_deg'] - 45.0) < 1e-12 and not res['same_side'])
# pivot: a part at (100, 0, 5) on a plane z = 0 tilted onto x = 0... the part's
# foot on the plane (100, 0, 0) is what stays put in the plane
res = A.align(((0, 0, 0), (0, 0, 1)), ((0, 0, 0), A.unit((0, 0.1, 1))),
              pivot_hint=(100, 0, 5))
check('it turns about the part\'s foot on the moving plane, not the plane origin',
      res['pivot'] == (100, 0, 0.0) and
      math.dist(A.apply(res, (100, 0, 0)), (100, 0, 0)) < 1e-9)

print()
if FAILED:
    print('%d FAILED: %s' % (len(FAILED), ', '.join(FAILED)))
    raise SystemExit(1)
print('all plane-alignment checks passed')
