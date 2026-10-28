#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Export PADS parser JSON to KiCad .pretty; conservative geometry validation."""
import argparse
import json
import math
import re
import sys
from pathlib import Path

UNITS_PER_MM = 1_500_000
CIRCLE_KEEPOUT_SEGMENTS = 64

def mm(v):
    return f'{float(v)/UNITS_PER_MM:.6f}'.rstrip('0').rstrip('.') or '0'

def clean_name(name):
    return re.sub(r'[^\w.+-]', '_', name).strip('._') or 'unnamed'

def q(s):
    return '"'+str(s).replace('\\', '\\\\').replace('"', '\\"').replace('\n', ' ')+'"'

def hidden_property(name, value):
    """Emit a hidden KiCad footprint property carrying source-only PADS metadata."""
    return [
        f'  (property {q(name)} {q(value)}',
        '    (at 0 0 0)',
        '    (layer "F.Fab")',
        '    (hide yes)',
        '    (effects (font (size 1 1) (thickness 0.15)))',
        '  )',
    ]



def label_property(name, value, label, layer_map, fallback_at, fallback_layer, force_hide=False):
    """Emit KiCad Reference/Value property using PADS decal label geometry when available."""
    if label:
        x = mm(label.get('x', 0))
        y = mm(-float(label.get('y', 0)))
        rot = -float(label.get('rotation', 0) or 0)
        layer = layer_map.get(int(label.get('layer', 0) or 0), fallback_layer)
        height = float(label.get('height', 0) or 0)
        stroke = float(label.get('width', 0) or 0)
        font_size = mm(height if height > 0 else UNITS_PER_MM)
        thickness = mm(stroke if stroke > 0 else 225000)
        visible = bool(label.get('visible', True))
        hjust = str(label.get('hjust', 'CENTER')).upper()
        vjust = str(label.get('vjust', 'CENTER')).upper()
        mirrored = bool(label.get('mirrored', False))
    else:
        x, y, rot = fallback_at
        layer = fallback_layer
        font_size = '1'
        thickness = '0.15'
        visible = True
        hjust = vjust = 'CENTER'
        mirrored = False

    justify = []
    if hjust == 'LEFT': justify.append('left')
    elif hjust == 'RIGHT': justify.append('right')
    if vjust == 'UP': justify.append('top')
    elif vjust == 'DOWN': justify.append('bottom')
    if mirrored: justify.append('mirror')

    lines = [
        f'  (property {q(name)} {q(value)}',
        f'    (at {x} {y} {rot:g})',
        f'    (layer {q(layer)})',
    ]
    if force_hide or not visible:
        lines.append('    (hide yes)')
    lines.extend([
        '    (effects',
        f'      (font (size {font_size} {font_size}) (thickness {thickness}))',
    ])
    if justify:
        lines.append('      (justify ' + ' '.join(justify) + ')')
    lines.extend(['    )', '  )'])
    return lines

def label_user_text(text, label, layer_map, fallback_layer='F.SilkS'):
    """Emit a visible fp_text user using the PADS decal-label geometry."""
    if not label or not bool(label.get('visible', True)):
        return []

    x = mm(label.get('x', 0))
    y = mm(-float(label.get('y', 0)))
    rot = -float(label.get('rotation', 0) or 0)
    layer = layer_map.get(int(label.get('layer', 0) or 0), fallback_layer)
    height = float(label.get('height', 0) or 0)
    stroke = float(label.get('width', 0) or 0)
    font_size = mm(height if height > 0 else UNITS_PER_MM)
    thickness = mm(stroke if stroke > 0 else 225000)
    hjust = str(label.get('hjust', 'CENTER')).upper()
    vjust = str(label.get('vjust', 'CENTER')).upper()
    mirrored = bool(label.get('mirrored', False))

    justify = []
    if hjust == 'LEFT': justify.append('left')
    elif hjust == 'RIGHT': justify.append('right')
    if vjust == 'UP': justify.append('top')
    elif vjust == 'DOWN': justify.append('bottom')
    if mirrored: justify.append('mirror')

    lines = [
        f'  (fp_text user {q(text)}',
        f'    (at {x} {y} {rot:g})',
        f'    (layer {q(layer)})',
        '    (effects',
        f'      (font (size {font_size} {font_size}) (thickness {thickness}))',
    ]
    if justify:
        lines.append('      (justify ' + ' '.join(justify) + ')')
    lines.extend(['    )', '  )'])
    return lines

def arc_point(arc, angle_deg):
    angle = math.radians(float(angle_deg))
    radius = float(arc['radius'])
    return {
        'x': float(arc['cx']) + radius * math.cos(angle),
        'y': float(arc['cy']) + radius * math.sin(angle),
    }

def graphic_segments(a, b, arc, width, dst):
    """Return one line or one/more KiCad arcs.  Major PADS arcs are split at <=180 deg."""
    def xy(p):
        return f'{mm(p["x"])} {mm(-float(p["y"]))}'
    if not arc:
        return [f'  (fp_line (start {xy(a)}) (end {xy(b)}) (stroke (width {width}) (type solid)) (layer {dst}))']

    start_angle = float(arc['start_angle'])
    delta = float(arc['delta_angle'])
    pieces = max(1, int(math.ceil(abs(delta) / 180.0)))
    step = delta / pieces
    result = []

    for part in range(pieces):
        part_start_angle = start_angle + step * part
        part_end_angle = start_angle + step * (part + 1)
        start = a if part == 0 else arc_point(arc, part_start_angle)
        end = b if part == pieces - 1 else arc_point(arc, part_end_angle)
        mid = arc_point(arc, part_start_angle + step / 2.0)
        result.append(
            f'  (fp_arc (start {xy(start)}) (mid {xy(mid)}) (end {xy(end)}) '
            f'(stroke (width {width}) (type solid)) (fill none) (layer {dst}))'
        )

    return result

def restriction_code(parent):
    """Return the canonical PADS restriction letters in P,R,C,V,T,A order."""
    restrictions = set(parent.get('restrictions') or [])
    mapping = (
        ('placement', 'P'),
        ('trace', 'R'),
        ('copper', 'C'),
        ('via', 'V'),
        ('testpoint', 'T'),
        ('accordion', 'A'),
    )
    return ''.join(letter for name, letter in mapping if name in restrictions)

def sexp_pad(number, xy, size, shape, drill=None, plated=True, rotation=0, layers='"F.Cu" "F.Paste" "F.Mask"', rounding=None):
    if drill:
        pad_type = "thru_hole" if plated else "np_thru_hole"
        out_number = number if plated else ""
    else:
        pad_type = "smd"
        out_number = number
    s = f'  (pad {q(out_number)} {pad_type} {shape} (at {mm(xy[0])} {mm(-xy[1])} {rotation:g}) (size {mm(size[0])} {mm(size[1])})'
    if drill:
        s += f' (drill {mm(drill)})'
    s += ' (layers '+('"*.Cu" "*.Mask"' if drill else layers)+')'
    if rounding is not None:
        s += f' (roundrect_rratio {rounding:g})'
    return s + ')'

def encode_pad(terminal, defs, number):
    """Map a PADS pad using the same basic shape semantics as KiCad's PADS importer."""
    shapes = [x for x in defs if x.get('sizeA', 0) > 0 and x.get('layer') in (-2, 1, 2)]
    if not shapes:
        raise ValueError('nessuna geometria rame utilizzabile')

    # Component-side/top entry first, then the default component-side definition.
    pad = next((p for p in shapes if p['layer'] == 1), None) or next((p for p in shapes if p['layer'] == -2), None)
    if not pad:
        raise ValueError('pad presente solo sul lato opposto')

    shape = pad['shape']
    size_a = pad['sizeA']
    size_b = pad.get('sizeB', 0) or size_a

    # PADS: sizeA = Y/height, sizeB = X/width.  KiCad uses X,Y.
    sx, sy = size_b, size_a
    rounding = None

    if shape in ('R', 'C', 'A', 'RT'):
        kshape = 'circle'
        sy = sx
    elif shape in ('S', 'ST'):
        kshape = 'rect'
        side = sx if pad.get('sizeB', 0) else sy
        sx = sy = side
    elif shape in ('O', 'OT', 'OF'):
        kshape = 'oval'
    elif shape == 'RF':
        radius = float(pad.get('corner_radius', 0) or 0)
        if radius > 0:
            kshape = 'roundrect'
            rounding = min(radius / max(1.0, min(sx, sy)), 0.5)
        else:
            kshape = 'rect'
    elif shape in ('RC', 'OC'):
        kshape = 'roundrect'
        radius = float(pad.get('corner_radius', 0) or 0)
        rounding = min(radius / max(1.0, min(sx, sy)), 0.5) if radius > 0 else 0.25
    else:
        raise ValueError(f'forma pad PADS {shape!r} non implementata')

    if pad.get('offsetX') or pad.get('offsetY') or pad.get('finger_offset') or pad.get('chamfered'):
        raise ValueError('pad offset/chamfered: conversione geometrica non implementata')

    rot = float(pad.get('rotation', 0))
    if abs(rot) > 360 and rot % 1800000 == 0:
        rot /= 1800000
    if abs(rot) > 360:
        raise ValueError(f'rotazione non riconosciuta: {rot}')

    drill = pad.get('drill') or None
    plated = bool(pad.get('plated', True))
    return sexp_pad(terminal['name'], (terminal['x'], terminal['y']), (sx, sy),
                    kshape, drill, plated, -rot, rounding=rounding)


def build_layer_map(data):
    """Build a footprint-graphics layer map from the PADS layer table in the dump."""
    infos = data.get('parser', {}).get('GetLayerInfos', [])
    copper_count = int(data.get('parser', {}).get('GetLayerCount', 2) or 2)
    result = {0: 'F.SilkS'}  # KiCad PADS loader: decal layer 0 is footprint outline.

    for info in infos:
        try:
            n = int(info.get('number'))
        except (TypeError, ValueError):
            continue
        name = str(info.get('name', '')).lower()
        typ = int(info.get('layer_type', 0) or 0)
        if n == 0:
            continue  # Special decal layer 0 stays F.SilkS.
        is_copper = bool(info.get('is_copper', False))

        # Decal graphics drawn on copper-numbered layers are documentation outlines;
        # KiCad redirects them to the corresponding silkscreen.
        if is_copper or typ in (1, 2, 3):
            result[n] = 'B.SilkS' if (n == copper_count or 'bottom' in name) else 'F.SilkS'
            continue

        if typ == 5:      # SOLDER_MASK
            result[n] = 'B.Mask' if 'bottom' in name else 'F.Mask'
        elif typ == 6:    # PASTE_MASK
            result[n] = 'B.Paste' if 'bottom' in name else 'F.Paste'
        elif typ == 7:    # SILK_SCREEN
            result[n] = 'B.SilkS' if 'bottom' in name else 'F.SilkS'
        elif typ == 8:    # ASSEMBLY
            result[n] = 'B.Fab' if 'bottom' in name else 'F.Fab'
        elif typ == 9:    # DOCUMENTATION
            result[n] = 'Cmts.User'
        elif typ == 10:   # DRILL
            result[n] = 'Dwgs.User'
        elif typ == 4:    # UNASSIGNED: preserve rather than silently lose it.
            result[n] = 'Cmts.User'

    # PADS special decal-side value.  Useful for old footprints and non-keepout graphics.
    result.setdefault(-1, 'B.SilkS')
    return result

def keepout_to_kicad(piece):
    """Map a PADS keepout parent/child to a KiCad rule area plus source metadata."""
    parent = piece.get('parent') or {}
    children = piece.get('children') or []
    if parent.get('type') != 'KEEP_OUT' or len(children) != 1:
        raise ValueError('keepout parent/child structure missing')

    restrictions = set(parent.get('restrictions') or [])
    unknown = int(parent.get('unknown_restriction_bits', 0) or 0)
    unsupported = sorted(restrictions & {'testpoint', 'accordion'})
    if unknown:
        raise ValueError(f'keepout has unknown PADS restriction bits 0x{unknown:x}')

    warnings = []
    if unsupported:
        warnings.append('PADS keepout restrictions preserved only as metadata: ' + ', '.join(unsupported))

    side = int(parent.get('side', 0) or 0)
    if side == 0:
        layers = '"*.Cu"'
    elif side == 1:
        layers = '"F.Cu"'
    elif side == -1:
        layers = '"B.Cu"'
    else:
        raise ValueError(f'keepout side/layer PADS {side} non mappato')

    flags = []
    if 'trace' in restrictions: flags.append('(tracks not_allowed)')
    if 'via' in restrictions: flags.append('(vias not_allowed)')
    if 'copper' in restrictions: flags.append('(copperpour not_allowed)')
    if 'placement' in restrictions: flags.append('(footprints not_allowed)')
    if not flags:
        warnings.append('keepout has no restriction KiCad can enforce; metadata preserved')
        return [], restriction_code(parent), warnings

    child = children[0]
    kind = child.get('type')
    pts = child.get('vertices') or []
    if kind == 'CIRCLE':
        if len(pts) != 2:
            raise ValueError('circular keepout without 2 diameter endpoints')

        a, b = pts
        cx = (float(a['x']) + float(b['x'])) / 2.0
        cy = (float(a['y']) + float(b['y'])) / 2.0
        dx = float(a['x']) - cx
        dy = float(a['y']) - cy
        radius = math.hypot(dx, dy)
        if radius <= 0:
            raise ValueError('circular keepout with zero radius')

        start_angle = math.atan2(dy, dx)
        pts = [
            {
                'x': cx + radius * math.cos(start_angle + 2.0 * math.pi * i / CIRCLE_KEEPOUT_SEGMENTS),
                'y': cy + radius * math.sin(start_angle + 2.0 * math.pi * i / CIRCLE_KEEPOUT_SEGMENTS),
            }
            for i in range(CIRCLE_KEEPOUT_SEGMENTS)
        ]
        warnings.append(f'circular PADS keepout approximated by {CIRCLE_KEEPOUT_SEGMENTS}-sided KiCad polygon')
    elif kind == 'CLOSED':
        if len(pts) < 4:
            raise ValueError('keepout polygon too short')

        # PADS repeats the first point to close KPTCLS; KiCad polygon does not need it.
        if pts[0].get('x') == pts[-1].get('x') and pts[0].get('y') == pts[-1].get('y'):
            pts = pts[:-1]
    else:
        raise ValueError(f'keepout child {kind!r} non supportato')

    if len(pts) < 3:
        raise ValueError('keepout polygon too short')

    xy = ' '.join(f'(xy {mm(p["x"])} {mm(-p["y"])})' for p in pts)
    keepout = ' '.join(flags)
    lines = [
        f'  (zone (net 0) (net_name "") (layers {layers}) (hatch edge 0.508)',
        '    (connect_pads (clearance 0))',
        '    (min_thickness 0.254)',
    ]

    if flags:
        lines.append(f'    (keepout {keepout})')

    lines.extend([
        f'    (polygon (pts {xy}))',
        '  )',
    ])
    return lines, restriction_code(parent), warnings

def graphics_to_kicad(piece, layer_map):
    """Return KiCad footprint primitives or a reason for unsupported geometry."""
    kind=piece['type']; layer=piece['layer']
    if kind in ('KPTCIR', 'KPTCLS'):
        return keepout_to_kicad(piece)
    if piece['line_style'] != 0:
        raise ValueError(f'line style PADS {piece["line_style"]} not yet mapped')
    if layer not in layer_map:
        raise ValueError(f'graphic layer PADS {layer} non mappato')
    dst=q(layer_map[layer]); width=mm(piece['width'])
    pts=piece['vertices']
    if not pts:raise ValueError('empty graphic')
    def xy(p):return f'{mm(p["x"])} {mm(-float(p["y"]))}'
    if kind=='CIRCLE':
        if len(pts)!=2:raise ValueError('CIRCLE without 2 endpoints')
        a,b=pts;cx=(a['x']+b['x'])/2;cy=(a['y']+b['y'])/2
        # PADS BASIC circles encode two opposite points of a diameter.
        return [f'  (fp_circle (center {mm(cx)} {mm(-cy)}) (end {xy(a)}) (stroke (width {width}) (type solid)) (fill none) (layer {dst}))'], None, []
    if kind in ('OPEN','CLOSED'):
        if len(pts)<2:raise ValueError('polyline too short')
        primitives=[]
        for idx in range(1,len(pts)):
            primitives.extend(graphic_segments(pts[idx-1], pts[idx], pts[idx].get('arc_from_previous'), width, dst))
        if kind=='CLOSED' and (pts[0].get('x') != pts[-1].get('x') or pts[0].get('y') != pts[-1].get('y')):
            primitives.extend(graphic_segments(pts[-1], pts[0], piece.get('closing_arc'), width, dst))
        return primitives, None, []
    raise ValueError(f'graphic type {kind} not representable without loss')

def export(data, out, allow_incomplete=False):
    entries=data.get('parser',{}).get('GetPartDecals')
    if not isinstance(entries,list):
        raise ValueError('JSON non contiene parser.GetPartDecals')
    out=Path(out)
    if out.exists() and not out.is_dir():
        raise ValueError('Destinazione non e una directory')
    out.mkdir(parents=True,exist_ok=True)
    issues=[]; warnings=[]; wrote=0; seen={}
    all_names=[(entry['value'].get('name') or str(entry['key'])) for entry in entries]

    # PARTTYPE relationships are diagnostic and retain unused alternatives, but they
    # are not a whitelist: a PCB can contain orphan library decals not referenced by
    # any PARTTYPE.  VIA styles are classified structurally by the DecalHeader parser.
    pt_info = data.get('parttype_decals') or {}
    parttypes = pt_info.get('parttypes') or []
    via_styles = set(pt_info.get('via_styles') or [])
    filtered_entries = [e for e in entries
                        if (e['value'].get('name') or str(e['key'])) not in via_styles]
    excluded = sorted(via_styles & set(all_names))

    used = set()
    for part in data.get('parser', {}).get('GetParts', []) or []:
        if isinstance(part, dict) and part.get('decal'):
            used.add(part['decal'])

    layer_map = build_layer_map(data)
    recovered=data.get("decal_graphics", {})
    recovery_error="C++ binary geometry parser did not identify these decals"
    for entry in filtered_entries:
        obj=entry['value']; name=obj.get('name') or str(entry['key'])
        terminals=obj.get('terminals',[])
        stacks={int(x['key']):x['value'] for x in obj.get('pad_stacks',[])}
        geometry=recovered.get(name)
        labels=(geometry or {}).get('labels') or []
        ref_label=next((x for x in labels if x.get('type')=='REF_DES'), None)
        value_label=next((x for x in labels if x.get('type')=='PART_TYPE'), None)
        sdb_version=int((geometry or {}).get('sdb_version', 0) or 0)
        legacy_reference_fallback = bool(ref_label) and sdb_version and sdb_version <= 0x2022
        lines=['(footprint '+q(name),'  (version 20240108)','  (generator "pads-dump")','  (layer "F.Cu")',
               '  (descr '+q('PADS binary import: verify graphics and pads')+')']
        lines.extend(label_property('Reference','REF**',ref_label,layer_map,('0','-2',0),'F.SilkS',
                                    force_hide=legacy_reference_fallback))
        if legacy_reference_fallback:
            # KiCad preserves the mandatory Reference field state of an already placed
            # footprint during "Update Footprints from Library".  The legacy PADS binary
            # importer leaves that field hidden, so a user text bound to ${REFERENCE}
            # restores the visible PADS decal reference without duplicating the native field.
            lines.extend(label_user_text('${REFERENCE}', ref_label, layer_map, 'F.SilkS'))
        lines.extend(label_property('Value',name,value_label,layer_map,('0','2',0),'F.Fab'))
        invalid=[]
        for idx,t in enumerate(terminals,1):
            # PADS padstack key 0 is default; specific overrides use numeric pin numbers.
            try:
                pin=int(t['name'])
                defs=stacks.get(pin, stacks.get(0))
                if defs is None:raise ValueError('padstack non trovato')
                lines.append(encode_pad(t,defs,pin))
            except (ValueError,KeyError,TypeError) as exc:
                invalid.append(f'pin {t.get("name",idx)}: {exc}')
        keepout_masks=[]
        footprint_warnings=[]
        if geometry is None:
            invalid.append('graphic owner/piece/vertex chain unavailable: '+str(recovery_error or 'owner not found'))
        else:
            for j,piece in enumerate(geometry['items']):
                try:
                    primitive_lines, keepout_mask, graphic_warnings = graphics_to_kicad(piece, layer_map)
                    lines.extend(primitive_lines)
                    if keepout_mask and keepout_mask not in keepout_masks:
                        keepout_masks.append(keepout_mask)
                    footprint_warnings.extend(f'graphic {j}: {warning}' for warning in graphic_warnings)
                except ValueError as exc:invalid.append(f'graphic {j}: {exc}')
        # This exporter is not yet verified for all pad shapes, text or labels;
        # require an explicit opt-in until the full decal is lossless.
        if obj.get('texts') or obj.get('labels'):
            invalid.append('decal texts/labels not exported')
        # Never deliver a silent incomplete footprint unless explicitly requested.
        if invalid and not allow_incomplete:
            issues.append({'decal':name,'status':'skipped','problems':invalid});continue

        if keepout_masks:
            lines.extend(hidden_property('PADS_Keepout_Restrictions', ';'.join(keepout_masks)))

        file_name=clean_name(name)
        if file_name in seen:
            seen[file_name]+=1;file_name=f'{file_name}__{seen[file_name]}'
        else:seen[file_name]=1
        lines.append(')')
        (out/(file_name+'.kicad_mod')).write_text('\n'.join(lines)+'\n',encoding='utf-8')
        wrote+=1
        if invalid: issues.append({'decal':name,'status':'incomplete','problems':invalid})
        if footprint_warnings:
            warnings.append({'decal':name,'status':'written_with_warnings','problems':footprint_warnings})
    exported_names = {e['value'].get('name') or str(e['key']) for e in filtered_entries}
    report={
        'total_decals':len(filtered_entries),
        'source_decal_records':len(entries),
        'written':wrote,
        'parttype_relationships':bool(parttypes),
        'excluded_via_styles':excluded,
        'used_decals':sorted(exported_names & used),
        'unused_library_decals':sorted(exported_names - used),
        'issues':issues,
        'warnings':warnings,
        'incomplete_mode':allow_incomplete
    }
    (out/'pads-dump-export-report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(f'PADS footprint library: {len(filtered_entries)} decals; scritti: {wrote}; problemi: {len(issues)}; warning: {len(warnings)}',file=sys.stderr)
    return 0 if not issues else (0 if allow_incomplete else 3)

if __name__=='__main__':
    ap=argparse.ArgumentParser()
    ap.add_argument('dump_json',type=Path)
    ap.add_argument('pretty_dir',type=Path)
    ap.add_argument('--allow-incomplete',action='store_true',help='esporta solo pad supportati e segnala la grafica mancante')
    a=ap.parse_args()
    try:sys.exit(export(json.loads(a.dump_json.read_text()),a.pretty_dir,a.allow_incomplete))
    except Exception as e:print(f'export_decals: {e}',file=sys.stderr);sys.exit(1)
