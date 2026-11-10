"""Align Planes - all the working code. RE-IMPORTED ON EVERY COMMAND EVENT.

Because of that reload, nothing may keep state in a module global between events.

WHAT THIS DOES
    Puts a moving plane onto a fixed plane and carries any number of bodies and
    components with it on the same rigid transform - a saw blade whose cutting
    plane has to land on a planned osteotomy plane, a plate onto its seat.
    Each plane is a construction plane or a planar face.

    A plane on a plane fixes only the tilt and the distance. The slide in the
    plane and the turn about its normal are left as they are: the rotation is the
    smallest one that makes the normals parallel, about the point of the moving
    plane nearest to what is moved (ap_math.align).

HOW IT MOVES THINGS
    The move code is Global Registration's, which was measured against the traps:
    one move feature per (component, kind, owning base feature), because a mixed
    collection moves the meshes and silently leaves the solids; a solid inside a
    base feature needs targetBaseFeature and the base reopened; occurrences get
    transform2 composed. Every entity is checked afterwards against where the
    transform should have put it.
"""

import importlib
import os
import traceback

import adsk.core
import adsk.fusion

import ap_math

A = importlib.reload(ap_math)      # hot reload reaches the maths too

ADDIN_DIR = os.path.dirname(os.path.realpath(__file__))
RES_DIR = os.path.join(ADDIN_DIR, 'resources', 'AlignPlanes')
LOG_PATH = os.path.join(ADDIN_DIR, 'AlignPlanes.log')

CMD_ID = 'AlignPlanesRun'
CMD_NAME = 'Align Planes'

CM = 10.0          # internal cm -> mm
MM = 0.1           # mm -> internal cm

VERSION = '1.0.0'


def log(msg):
    try:
        with open(LOG_PATH, 'a', encoding='utf-8') as fh:
            fh.write('%s\n' % msg)
    except Exception:
        pass


def short_error(tb):
    for line in reversed(tb.strip().splitlines()):
        line = line.strip()
        if line and not line.startswith(('File "', 'Traceback', '^', '~')):
            return line
    return tb.strip().splitlines()[-1]


def entity_label(e):
    try:
        return e.name
    except Exception:
        pass
    face = adsk.fusion.BRepFace.cast(e)
    if face:
        return 'face of %s' % face.body.name
    return e.objectType


def selected(sel_input):
    return [sel_input.selection(i).entity for i in range(sel_input.selectionCount)]


# --------------------------------------------------------------------------- #
# reading a plane
# --------------------------------------------------------------------------- #

def _t(p):
    return (p.x, p.y, p.z)


def plane_of(e):
    """(point, unit normal) of a construction plane or a planar face, in cm,
    plus what it is. A face's normal is its OUTWARD normal; its point is the
    face centroid, so the pivot sits on the face rather than at the far-off
    origin of the infinite plane."""
    cp = adsk.fusion.ConstructionPlane.cast(e)
    if cp:
        g = cp.geometry
        return {'point': _t(g.origin), 'normal': A.unit(_t(g.normal)),
                'u': _t(g.uDirection), 'kind': 'construction plane',
                'label': cp.name, 'body': None}
    face = adsk.fusion.BRepFace.cast(e)
    if face:
        if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
            raise ValueError('the face of %s is not planar' % face.body.name)
        try:
            p = face.centroid
        except Exception:
            p = face.pointOnFace
        ok, n = face.evaluator.getNormalAtPoint(face.pointOnFace)
        if not ok:
            n = face.geometry.normal
            if face.isParamReversed:
                n.scaleBy(-1.0)
        return {'point': _t(p), 'normal': A.unit(_t(n)), 'u': _t(face.geometry.uDirection),
                'kind': 'planar face', 'label': 'face of %s' % face.body.name,
                'body': face.body}
    raise ValueError('%s is not a plane or a planar face' % entity_label(e))


# --------------------------------------------------------------------------- #
# the matrix, built the unambiguous way and checked (from Global Registration)
# --------------------------------------------------------------------------- #

def matrix_from(R, t):
    """x -> R x + t as a Matrix3D. setWithArray's order is undocumented and a
    wrong guess applies the transpose silently, so the matrix is built as the
    coordinate system with origin t and axes the columns of R, then three probe
    points are pushed through Fusion's own transformBy and compared."""
    m = adsk.core.Matrix3D.create()
    ok = m.setWithCoordinateSystem(
        adsk.core.Point3D.create(*t),
        adsk.core.Vector3D.create(R[0][0], R[1][0], R[2][0]),
        adsk.core.Vector3D.create(R[0][1], R[1][1], R[2][1]),
        adsk.core.Vector3D.create(R[0][2], R[1][2], R[2][2]))
    if not ok:
        raise RuntimeError('Fusion refused the transform matrix')
    for probe in ((1.0, 0.0, 0.0), (0.0, 2.5, 0.0), (-3.0, 1.0, 7.0)):
        p = adsk.core.Point3D.create(*probe)
        p.transformBy(m)
        want = A.add(A.matvec(R, probe), t)
        if max(abs(a - b) for a, b in zip(want, _t(p))) > 1.0e-9:
            raise RuntimeError('the transform Fusion built does not match the one '
                               'computed; refusing to move anything')
    return m


# --------------------------------------------------------------------------- #
# moving (from Global Registration, where each trap was measured)
# --------------------------------------------------------------------------- #

def body_points(body):
    calc = body.meshManager.createMeshCalculator()
    calc.surfaceTolerance = 0.01
    tm = calc.calculate()
    return tm.nodeCoordinatesAsDouble


def probe_point(e):
    """A point the move takes EXACTLY, read LIVE: a solid's centre of mass, a
    mesh body's own nodes. Not a bounding-box centre (a rotation moves it
    elsewhere) and not the display mesh (cached: it returns the old place)."""
    mb = adsk.fusion.MeshBody.cast(e)
    if mb:
        try:
            pm = mb.mesh or mb.displayMesh
            co = pm.nodeCoordinatesAsDouble
            n = len(co) // 3
            return (sum(co[0::3]) / n, sum(co[1::3]) / n, sum(co[2::3]) / n)
        except Exception:
            pass
    body = adsk.fusion.BRepBody.cast(e)
    if body:
        try:
            if body.isSolid:
                c = body.physicalProperties.centerOfMass
                if c:
                    return _t(c)
        except Exception:
            pass
        try:
            co = body_points(body)
            n = len(co) // 3
            return (sum(co[0::3]) / n, sum(co[1::3]) / n, sum(co[2::3]) / n)
        except Exception:
            pass
    occ = adsk.fusion.Occurrence.cast(e)
    if occ:
        try:
            c = occ.physicalProperties.centerOfMass
            if c:
                return _t(c)
        except Exception:
            pass
        return _t(occ.transform2.translation)
    return None


def refetch(design, e, token, name):
    if token:
        try:
            found = design.findEntityByToken(token)
            if found and len(found):
                return found[0]
        except Exception:
            pass
    try:
        comp = e.parentComponent
        for coll in (comp.bRepBodies, comp.meshBodies):
            for cand in coll:
                if cand.name == name:
                    return cand
    except Exception:
        pass
    return e


def _kind(e):
    if adsk.fusion.MeshBody.cast(e):
        return 'mesh'
    if adsk.fusion.BRepBody.cast(e):
        return 'brep'
    if adsk.fusion.Occurrence.cast(e):
        return 'occurrence'
    return 'other'


def apply_transform(design, entities, matrix):
    groups, occs, order = {}, [], []
    for e in entities:
        k = _kind(e)
        if k == 'occurrence':
            occs.append(e)
            continue
        base = None
        if k == 'brep':
            try:
                base = e.baseFeature
            except Exception:
                base = None
        comp = getattr(e, 'parentComponent', None) or design.rootComponent
        key = (comp.entityToken if hasattr(comp, 'entityToken') else id(comp),
               k, base.entityToken if base else None)
        if key not in groups:
            groups[key] = {'comp': comp, 'kind': k, 'base': base, 'ents': []}
            order.append(key)
        groups[key]['ents'].append(e)

    tokens, names = {}, {}
    for e in entities:
        try:
            tokens[id(e)] = e.entityToken
        except Exception:
            tokens[id(e)] = None
        names[id(e)] = entity_label(e)
    before = {id(e): probe_point(e) for e in entities}
    notes = []
    for key in order:
        g = groups[key]
        col = adsk.core.ObjectCollection.create()
        for e in g['ents']:
            col.add(e)
        label = '%d %s' % (len(g['ents']), g['kind'])
        try:
            mi = g['comp'].features.moveFeatures.createInput2(col)
            base = g['base']
            if base is not None:
                mi.targetBaseFeature = base
                base.startEdit()
            try:
                if not mi.defineAsFreeMove(matrix):
                    raise RuntimeError('defineAsFreeMove refused the matrix')
                f = g['comp'].features.moveFeatures.add(mi)
                try:
                    f.name = 'Align Planes'
                except Exception:
                    pass
            finally:
                if base is not None:
                    base.finishEdit()
            notes.append('move feature over %s%s'
                         % (label, ' (inside a base feature)' if base else ''))
        except Exception:
            notes.append('%s could NOT be moved: %s'
                         % (label, short_error(traceback.format_exc())))
    for occ in occs:
        try:
            t = occ.transform2
            t.transformBy(matrix)
            occ.transform2 = t
            notes.append('occurrence "%s" transform composed' % occ.name)
        except Exception:
            notes.append('occurrence "%s" could NOT be moved: %s'
                         % (occ.name, short_error(traceback.format_exc())))
    adsk.doEvents()

    checks = []
    for e in entities:
        b = before.get(id(e))
        cur = refetch(design, e, tokens.get(id(e)), names.get(id(e)))
        a = probe_point(cur)
        row = {'entity': entity_label(cur), 'kind': _kind(cur)}
        if b is None or a is None:
            row['status'] = 'not measurable'
        else:
            q = adsk.core.Point3D.create(*b)
            q.transformBy(matrix)
            want = _t(q)
            err = max(abs(a[i] - want[i]) for i in range(3)) * CM
            asked = max(abs(want[i] - b[i]) for i in range(3)) * CM
            row['error_mm'] = round(err, 5)
            row['asked_mm'] = round(asked, 4)
            row['status'] = 'ok' if err < 1.0e-3 else (
                'DID NOT MOVE' if err > asked * 0.5 else 'moved wrongly')
        checks.append(row)
    return notes, checks


# --------------------------------------------------------------------------- #
# the alignment
# --------------------------------------------------------------------------- #

def to_move(moving_e, along):
    """What travels: the move-along entities, plus the body a moving FACE belongs
    to (a face cannot go anywhere without it). Duplicates removed."""
    out, seen = [], set()
    pm = plane_of(moving_e)
    extra = [pm['body']] if pm['body'] is not None else []
    for e in list(along) + extra:
        try:
            key = e.entityToken
        except Exception:
            key = id(e)
        if key not in seen:
            seen.add(key)
            out.append(e)
    return out


def plan(fixed_e, moving_e, along, flip=False, offset_mm=0.0):
    """The transform, without moving anything (cm)."""
    pf, pm = plane_of(fixed_e), plane_of(moving_e)
    ents = to_move(moving_e, along)
    pts = [p for p in (probe_point(e) for e in ents) if p is not None]
    hint = tuple(sum(p[i] for p in pts) / len(pts) for i in range(3)) if pts else None
    res = A.align((pm['point'], pm['normal']), (pf['point'], pf['normal']), hint,
                  flip=flip, offset=offset_mm * MM, spin_axis=pm['u'])
    ang0, d0 = A.relation((pm['point'], pm['normal']), (pf['point'], pf['normal']))
    res.update({'fixed': pf, 'moving': pm, 'entities': ents,
                'before_angle_deg': ang0, 'before_distance_mm': d0 * CM})
    return res


def align_planes(app, fixed_e, moving_e, along, flip=False, offset_mm=0.0):
    design = adsk.fusion.Design.cast(app.activeProduct)
    P = plan(fixed_e, moving_e, along, flip, offset_mm)
    if not P['entities']:
        raise ValueError('nothing to move: select the bodies or components that '
                         'travel with the moving plane')
    try:
        mtoken = moving_e.entityToken
    except Exception:
        mtoken = None
    m = matrix_from(P['R'], P['t'])
    notes, checks = apply_transform(design, P['entities'], m)
    # the plane as the transform put it - exact by construction
    p2 = A.apply(P, P['moving']['point'])
    n2 = A.matvec(P['R'], P['moving']['normal'])
    ang, dist = A.relation((p2, n2), (P['fixed']['point'], P['fixed']['normal']))
    out = {'fixed': P['fixed']['label'], 'moving': P['moving']['label'],
           'moving_kind': P['moving']['kind'],
           'angle_deg': P['angle_deg'], 'travel_mm': P['travel'] * CM,
           'pivot_mm': [v * CM for v in P['pivot']], 'same_side': P['same_side'],
           'flip': flip, 'offset_mm': offset_mm,
           'before_angle_deg': P['before_angle_deg'],
           'before_distance_mm': P['before_distance_mm'],
           'after_angle_deg': ang, 'after_distance_mm': dist * CM,
           'notes': notes, 'checks': checks}
    # did the moving plane itself come along? A face does, with its body. A
    # construction plane only if it is defined by something that moved.
    try:
        live = refetch(design, moving_e, mtoken, P['moving']['label'])
        lp = plane_of(live)
        la, ld = A.relation((lp['point'], lp['normal']),
                            (P['fixed']['point'], P['fixed']['normal']))
        out['plane_followed'] = la < 1e-4 and abs(ld * CM - offset_mm) < 1e-3
        out['plane_live_angle_deg'] = la
        out['plane_live_distance_mm'] = ld * CM
    except Exception:
        out['plane_followed'] = None
    return out


def format_report(r):
    L = ['%s  ->  %s' % (r['moving'], r['fixed']), '']
    L.append('BEFORE      %.4f deg apart, %.4f mm from the fixed plane'
             % (r['before_angle_deg'], r['before_distance_mm']))
    L.append('TRANSFORM   turned %.4f deg, then moved %.4f mm along the fixed normal'
             % (r['angle_deg'], r['travel_mm']))
    L.append('            about (%.3f, %.3f, %.3f) mm on the moving plane'
             % tuple(r['pivot_mm']))
    L.append('            normals now %s%s' % (
        'point the same way' if r['same_side'] else 'face each other',
        ' (flipped)' if r['flip'] else ''))
    if abs(r['offset_mm']) > 0:
        L.append('            offset %.4f mm' % r['offset_mm'])
    L.append('AFTER       %.6f deg apart, %.6f mm from the fixed plane'
             % (r['after_angle_deg'], r['after_distance_mm']))
    L.append('')
    L.append('MOVED')
    for c in r['checks']:
        if 'error_mm' in c:
            L.append('  %-34s %-13s %s  (off %.5f mm)'
                     % (c['entity'][:34], c['kind'], c['status'], c['error_mm']))
        else:
            L.append('  %-34s %-13s %s' % (c['entity'][:34], c['kind'], c['status']))
    for n in r['notes']:
        if 'NOT' in n:
            L.append('  ' + n)
    if r.get('plane_followed') is False and r['moving_kind'] == 'construction plane':
        L.append('')
        L.append('%s itself stayed where it was: it is not defined by anything that '
                 'moved, and construction geometry cannot be moved on its own.'
                 % r['moving'])
    return '\n'.join(L)


# --------------------------------------------------------------------------- #
# dialog
# --------------------------------------------------------------------------- #

def _esc(t):
    return str(t).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def _rows(pairs):
    out = []
    for k, v in pairs:
        out.append('<b>%s</b> %s' % (_esc(k), v) if k else v)
    return '<br>'.join(out)


def on_created(app, cmd):
    cmd.okButtonText = 'Align'
    cmd.isRepeatable = False
    inputs = cmd.commandInputs
    for cid, name, prompt in (
            ('selFixed', 'Fixed plane', 'The plane that stays - a construction '
                                        'plane or a planar face'),
            ('selMoving', 'Moving plane', 'The plane to bring onto it - a '
                                          'construction plane or a planar face')):
        si = inputs.addSelectionInput(cid, name, prompt)
        si.addSelectionFilter('ConstructionPlanes')
        si.addSelectionFilter('PlanarFaces')
        si.setSelectionLimits(0, 1)
    sa = inputs.addSelectionInput('selAlong', 'Move along',
                                  'The bodies and components that travel with the '
                                  'moving plane')
    for f in ('SolidBodies', 'MeshBodies', 'SurfaceBodies', 'Occurrences'):
        sa.addSelectionFilter(f)
    sa.setSelectionLimits(0, 0)
    fl = inputs.addBoolValueInput('flip', 'Flip', True, '', False)
    fl.tooltip = ('By default the moving plane turns the least it can, so the '
                  'normals end up pointing the way that is nearer. Flip turns the '
                  'part over onto the same plane instead.')
    inputs.addValueInput('offset', 'Offset', 'mm', adsk.core.ValueInput.createByReal(0.0))
    info = inputs.addTextBoxCommandInput('info', '', '', 8, True)
    info.isFullWidth = True
    refresh(inputs)


def _options(inputs):
    return inputs.itemById('flip').value, inputs.itemById('offset').value * CM


def refresh(inputs):
    fx = selected(inputs.itemById('selFixed'))
    mv = selected(inputs.itemById('selMoving'))
    al = selected(inputs.itemById('selAlong'))
    pairs = []
    if not fx or not mv:
        pairs.append(('', 'Select the fixed plane and the moving plane, then what '
                          'moves with it. A planar face carries its body along.'))
    else:
        try:
            flip, off = _options(inputs)
            P = plan(fx[0], mv[0], al, flip, off)
            pairs.append(('now', '%.3f deg apart, %.3f mm away'
                          % (P['before_angle_deg'], P['before_distance_mm'])))
            pairs.append(('turn', '%.3f deg' % P['angle_deg']))
            pairs.append(('shift', '%.3f mm along the fixed normal' % (P['travel'] * CM)))
            pairs.append(('moves', _esc(', '.join(entity_label(e) for e in P['entities'])
                                        or 'nothing yet')))
            if P['moving']['kind'] == 'construction plane' and not al:
                pairs.append(('', 'A construction plane moves nothing by itself: '
                                  'select what goes with it.'))
        except Exception as exc:
            pairs.append(('', _esc(exc)))
    inputs.itemById('info').formattedText = _rows(pairs)


def on_input_changed(app, inputs, changed_id):
    refresh(inputs)


def on_validate(inputs):
    fx = selected(inputs.itemById('selFixed'))
    mv = selected(inputs.itemById('selMoving'))
    if not fx or not mv or fx[0] == mv[0]:
        return False
    try:
        return bool(to_move(mv[0], selected(inputs.itemById('selAlong'))))
    except Exception:
        return False


def on_execute(app, ui, inputs):
    fx = selected(inputs.itemById('selFixed'))[0]
    mv = selected(inputs.itemById('selMoving'))[0]
    al = selected(inputs.itemById('selAlong'))
    flip, off = _options(inputs)
    try:
        r = align_planes(app, fx, mv, al, flip, off)
    except ValueError as exc:
        ui.messageBox(str(exc), CMD_NAME)
        return None
    text = format_report(r)
    log(text)
    ui.messageBox(text, CMD_NAME)
    return r


def on_destroy(app):
    pass


# --------------------------------------------------------------------------- #
# toolbar
# --------------------------------------------------------------------------- #

TOOLTIP = ('Bring a moving plane onto a fixed plane, carrying bodies and components '
           'with it.\n\n'
           'Each plane is a construction plane or a planar face. Only the tilt and '
           'the distance change: the part turns the least it can, about the point '
           'of the moving plane nearest to it, and does not slide in the plane. '
           'Flip turns it over onto the same plane; Offset leaves a gap.')

PANELS = ['SolidModifyPanel', 'ParaMeshModifyPanel']


def install(app, ui, created_handler_factory):
    old = ui.commandDefinitions.itemById(CMD_ID)
    if old:
        old.deleteMe()
    folder = RES_DIR if os.path.isdir(RES_DIR) and os.listdir(RES_DIR) else ''
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, TOOLTIP,
                                                        folder)
    cmd_def.commandCreated.add(created_handler_factory())
    placed = []
    for pid in PANELS:
        panel = ui.allToolbarPanels.itemById(pid)
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)
            placed.append(pid)
    return placed


def uninstall(ui):
    for pid in PANELS:
        panel = ui.allToolbarPanels.itemById(pid)
        if panel:
            c = panel.controls.itemById(CMD_ID)
            if c:
                c.deleteMe()
    d = ui.commandDefinitions.itemById(CMD_ID)
    if d:
        d.deleteMe()


# --------------------------------------------------------------------------- #
# dev bridge commands
# --------------------------------------------------------------------------- #

def _all_planes(des):
    out = []
    for comp in des.allComponents:
        for cp in comp.constructionPlanes:
            out.append(cp)
    return out


def _all_bodies(des):
    out = []
    for comp in des.allComponents:
        out.extend(comp.bRepBodies)
        out.extend(comp.meshBodies)
    return out


def _find(des, name, planes=False):
    pool = _all_planes(des) if planes else _all_bodies(des)
    for e in pool:
        if e.name == name:
            return e
    raise RuntimeError('%r not found. Available: %s'
                       % (name, ', '.join(e.name for e in pool)))


def dev_ping(app, job):
    des = adsk.fusion.Design.cast(app.activeProduct)
    return {'version': VERSION, 'document': app.activeDocument.name,
            'planes': [p.name for p in _all_planes(des)]}


def dev_inspect(app, job):
    des = adsk.fusion.Design.cast(app.activeProduct)
    planes = []
    for cp in _all_planes(des):
        row = {'name': cp.name, 'component': cp.parentComponent.name}
        try:
            g = cp.geometry
            row['origin_mm'] = [round(v * CM, 4) for v in _t(g.origin)]
            row['normal'] = [round(v, 6) for v in _t(g.normal)]
        except Exception:
            row['geometry'] = short_error(traceback.format_exc())
        try:
            d = cp.definition
            row['definition'] = d.objectType.split('::')[-1]
            for attr in ('planarEntity', 'planeOne', 'planeTwo', 'point', 'edge',
                         'pointOne', 'pointTwo', 'pointThree', 'linearEntity'):
                v = getattr(d, attr, None)
                if v is not None:
                    row[attr] = '%s %s' % (v.objectType.split('::')[-1], entity_label(v))
        except Exception:
            pass
        try:
            row['visible'] = cp.isLightBulbOn
        except Exception:
            pass
        planes.append(row)
    bodies = []
    for b in _all_bodies(des):
        row = {'name': b.name, 'kind': _kind(b), 'component': b.parentComponent.name}
        p = probe_point(b)
        if p:
            row['centre_mm'] = [round(v * CM, 3) for v in p]
        bodies.append(row)
    return {'document': app.activeDocument.name, 'planes': planes, 'bodies': bodies,
            'design_type': 'parametric' if des.designType ==
            adsk.fusion.DesignTypes.ParametricDesignType else 'direct'}


def dev_plan(app, job):
    des = adsk.fusion.Design.cast(app.activeProduct)
    fx = _find(des, job['fixed'], True)
    mv = _find(des, job['moving'], True)
    al = [_find(des, n) for n in job.get('along', [])]
    P = plan(fx, mv, al, bool(job.get('flip')), float(job.get('offset_mm', 0.0)))
    return {'angle_deg': P['angle_deg'], 'travel_mm': P['travel'] * CM,
            'pivot_mm': [v * CM for v in P['pivot']], 'same_side': P['same_side'],
            'before_angle_deg': P['before_angle_deg'],
            'before_distance_mm': P['before_distance_mm'],
            'moves': [entity_label(e) for e in P['entities']]}


def dev_align(app, job):
    des = adsk.fusion.Design.cast(app.activeProduct)
    fx = _find(des, job['fixed'], True)
    mv = _find(des, job['moving'], True)
    al = [_find(des, n) for n in job.get('along', [])]
    r = align_planes(app, fx, mv, al, bool(job.get('flip')),
                     float(job.get('offset_mm', 0.0)))
    r['text'] = format_report(r)
    return r


def dev_new_test_doc(app, job):
    """A throwaway document: two construction planes at a known angle and a box
    to carry, so the alignment can be checked without touching a real design."""
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    p1 = root.constructionPlanes.createInput()
    p1.setByOffset(root.xYConstructionPlane, adsk.core.ValueInput.createByReal(2.0))
    cp1 = root.constructionPlanes.add(p1)
    cp1.name = 'Fixed'
    p2 = root.constructionPlanes.createInput()
    p2.setByAngle(root.xConstructionAxis, adsk.core.ValueInput.createByString('30 deg'),
                  root.xYConstructionPlane)
    cp2 = root.constructionPlanes.add(p2)
    cp2.name = 'Moving'
    tbm = adsk.fusion.TemporaryBRepManager.get()
    obb = adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(3.0, 1.0, 0.5), adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0), 2.0, 1.0, 0.2)
    base = root.features.baseFeatures.add()
    base.startEdit()
    root.bRepBodies.add(tbm.createBox(obb), base)
    base.finishEdit()
    b = base.bodies.item(0)
    b.name = 'Blade'
    return {'document': doc.name}


def dev_close_doc(app, job):
    doc = app.activeDocument
    name = doc.name
    doc.close(False)
    return {'closed': name, 'now': app.activeDocument.name if app.activeDocument else None}


DEV_COMMANDS = {
    'ping': dev_ping,
    'inspect': dev_inspect,
    'plan': dev_plan,
    'align': dev_align,
    'new_test_doc': dev_new_test_doc,
    'close_doc': dev_close_doc,
}
