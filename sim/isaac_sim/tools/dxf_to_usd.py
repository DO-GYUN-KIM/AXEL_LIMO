#!/usr/bin/env python3
"""대회 트랙 도면(DXF)을 Isaac Sim용 USD 바닥 트랙으로 바꿉니다.

도면의 선(LINE, LWPOLYLINE, POLYLINE, ARC, CIRCLE, SPLINE)을 폭이 있는 평평한 띠(차선 테이프)로 만들고,
그 아래에 충돌이 있는 바닥판을 깝니다. 선 색은 도면의 색(ACI/트루컬러)을 그대로 씁니다.

준비:  python3 -m pip install ezdxf usd-core
사용:
  python3 dxf_to_usd.py 트랙.dxf --list-layers                # 레이어 이름과 선 개수만 보기
  python3 dxf_to_usd.py 트랙.dxf -o track.usda                 # 전부 변환
  python3 dxf_to_usd.py 트랙.dxf -o track.usda --layers 차선,정지선 --line-width 0.03

결과 좌표: 도면 왼쪽 아래 모서리가 (0,0), 단위 m. 실행이 끝나면 트랙 크기를 출력합니다.
scenario.json의 "track_usd"에 결과 파일 경로를 넣으십시오.
"""
import argparse
import math
import sys

import ezdxf
from ezdxf import colors, path as dxfpath
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

UNIT_TO_M = {0: None, 1: 0.0254, 2: 0.3048, 4: 0.001, 5: 0.01, 6: 1.0}  # $INSUNITS
KINDS = ('LINE', 'LWPOLYLINE', 'POLYLINE', 'ARC', 'CIRCLE', 'SPLINE', 'ELLIPSE')


def entities(msp, layers):
    for e in msp:
        if e.dxftype() not in KINDS:
            continue
        if layers and e.dxf.layer not in layers:
            continue
        yield e


def entity_rgb(doc, e):
    try:
        if e.rgb:
            return tuple(c / 255 for c in e.rgb)
        aci = e.dxf.color
        if aci in (0, 256):  # BYBLOCK/BYLAYER
            aci = doc.layers.get(e.dxf.layer).color
        r, g, b = colors.aci2rgb(abs(aci))
        if (r, g, b) == (0, 0, 0):
            return (1.0, 1.0, 1.0)
        return (r / 255, g / 255, b / 255)
    except Exception:
        return (1.0, 1.0, 1.0)


def strip_quads(pts, half_w):
    """점 열을 폭 2*half_w의 사각형 띠들로 만듭니다(꺾이는 곳은 겹쳐 그림)."""
    verts, faces = [], []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        n = math.hypot(dx, dy)
        if n < 1e-9:
            continue
        nx, ny = -dy / n * half_w, dx / n * half_w
        k = len(verts)
        verts += [(x0 + nx, y0 + ny), (x0 - nx, y0 - ny), (x1 - nx, y1 - ny), (x1 + nx, y1 + ny)]
        faces.append((k, k + 1, k + 2, k + 3))
    return verts, faces


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dxf')
    ap.add_argument('-o', '--out', default='track.usda')
    ap.add_argument('--list-layers', action='store_true')
    ap.add_argument('--layers', default='', help='쉼표로 구분한 레이어 이름. 비우면 전부')
    ap.add_argument('--unit', type=float, default=0.0,
                    help='도면 1단위가 몇 m인지. 0이면 도면의 $INSUNITS를 따르고, 그것도 없으면 mm로 봅니다')
    ap.add_argument('--line-width', type=float, default=0.03, help='차선 테이프 폭 m (추정 기본값 3 cm)')
    ap.add_argument('--margin', type=float, default=0.5, help='바닥판 여유 m')
    ap.add_argument('--tolerance', type=float, default=0.005, help='곡선 근사 허용 오차 m')
    ap.add_argument('--floor-rgb', default='0.12,0.12,0.12')
    a = ap.parse_args()

    doc = ezdxf.readfile(a.dxf)
    msp = doc.modelspace()

    if a.list_layers:
        cnt = {}
        for e in entities(msp, None):
            cnt[e.dxf.layer] = cnt.get(e.dxf.layer, 0) + 1
        for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]):
            print(f'{v:6d}  {k}')
        print('$INSUNITS =', doc.header.get('$INSUNITS', 0))
        return

    unit = a.unit or UNIT_TO_M.get(doc.header.get('$INSUNITS', 0)) or 0.001
    layers = set(s for s in a.layers.split(',') if s)

    polylines = []  # (rgb, [(x,y), ...]) 단위 m
    for e in entities(msp, layers):
        try:
            p = dxfpath.make_path(e)
        except Exception:
            continue
        pts = [(v.x * unit, v.y * unit) for v in p.flattening(a.tolerance / unit)]
        if len(pts) >= 2:
            polylines.append((entity_rgb(doc, e), pts))
    if not polylines:
        sys.exit('변환할 선이 없습니다. --list-layers로 레이어 이름을 확인하십시오.')

    xs = [x for _, pts in polylines for x, _ in pts]
    ys = [y for _, pts in polylines for _, y in pts]
    ox, oy = min(xs), min(ys)
    w, h = max(xs) - ox, max(ys) - oy

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/Track')
    stage.SetDefaultPrim(root.GetPrim())

    def material(name, rgb, rough=0.6):
        m = UsdShade.Material.Define(stage, f'/Track/Looks/{name}')
        sh = UsdShade.Shader.Define(stage, f'/Track/Looks/{name}/Shader')
        sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(rough)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
        return m

    # 바닥판 (충돌 있음, 윗면 z=0)
    floor = UsdGeom.Cube.Define(stage, '/Track/Floor')
    floor.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(floor)
    xf.AddTranslateOp().Set(Gf.Vec3d(w / 2, h / 2, -0.01))
    xf.AddScaleOp().Set(Gf.Vec3f(w + 2 * a.margin, h + 2 * a.margin, 0.02))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
        material('Floor', tuple(float(c) for c in a.floor_rgb.split(','))))

    # 선은 색별로 메시 하나 (충돌 없음, 바닥 1 mm 위)
    by_color = {}
    for rgb, pts in polylines:
        key = tuple(round(c, 3) for c in rgb)
        by_color.setdefault(key, []).append([(x - ox, y - oy) for x, y in pts])
    for i, (rgb, plist) in enumerate(by_color.items()):
        V, F = [], []
        for pts in plist:
            v, f = strip_quads(pts, a.line_width / 2)
            base = len(V)
            V += v
            F += [tuple(base + j for j in q) for q in f]
        mesh = UsdGeom.Mesh.Define(stage, f'/Track/Lines_{i}')
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(x, y, 0.001) for x, y in V]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * len(F)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([j for q in F for j in q]))
        mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * len(V)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(f'Line_{i}', rgb, 0.4))

    stage.GetRootLayer().Save()
    print(f'저장: {a.out}')
    print(f'단위 {unit} m/도면단위, 선 {len(polylines)}개, 색 {len(by_color)}종')
    print(f'트랙 크기 {w:.2f} m x {h:.2f} m (도면 원점 이동량 x {-ox:.3f}, y {-oy:.3f} m)')
    if not (3 < w < 20 and 2 < h < 15):
        print('경고: 대회 트랙(10 x 6 m)과 크기가 많이 다릅니다. --unit 값을 확인하십시오.')


if __name__ == '__main__':
    main()
