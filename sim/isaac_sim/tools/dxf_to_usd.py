#!/usr/bin/env python3
"""대회 트랙 도면(DXF)을 Isaac Sim용 바닥 트랙(USD + 바닥 그림 PNG)으로 바꿉니다.

대회 도면(2026Track 도면 시트1.dxf)은 테이프를 '윤곽선 두 줄'로 그려 두었습니다.
그래서 선을 그대로 띠로 만들지 않고, 도면을 고해상도 그림으로 그린 뒤
'폭이 좁은 닫힌 칸'(= 테이프, 횡단보도 블록, 글자 획)만 칠해 실제 노면처럼 만듭니다.
주최 측 PNG(2026Track.png)를 주면 각 테이프의 색(흰색/주황)을 그 그림에서 가져옵니다.

준비:  python3 -m pip install ezdxf usd-core numpy scipy pillow
사용 (대회 도면은 1:30 축척이라 도면 1단위 = 0.03 m):
  python3 dxf_to_usd.py "2026Track 도면 시트1.dxf" --unit 0.03 --ref-png 2026Track.png \
          -o ../limo_ros2_ackermann/limo_ros2_ackermann_python/track/autorace2026.usda

결과: <출력>.usda 와 같은 폴더의 <출력>_floor.png.
좌표: 도면 바깥 테두리(보드) 왼쪽 아래 모서리 = (0,0), x 오른쪽, y 위쪽, 단위 m.
"""
import argparse
import os
import sys

import ezdxf
import numpy as np
from ezdxf import path as dxfpath
from PIL import Image, ImageDraw
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from scipy import ndimage as ndi

KINDS = ('LINE', 'LWPOLYLINE', 'POLYLINE', 'ARC', 'CIRCLE', 'SPLINE', 'ELLIPSE')
FLOOR = (54, 52, 46)      # 주최 측 그림의 바닥색
WHITE = (223, 218, 202)
ORANGE = (229, 170, 67)


def load_polylines(dxf, layers, unit):
    doc = ezdxf.readfile(dxf)
    out = []
    for e in doc.modelspace():
        if e.dxftype() not in KINDS or (layers and e.dxf.layer not in layers):
            continue
        try:
            pts = [(v.x * unit, v.y * unit) for v in dxfpath.make_path(e).flattening(0.0005 / unit)]
        except Exception:
            continue
        if len(pts) >= 2:
            out.append(pts)
    return out


def ref_masks(ref_png, shape):
    """주최 측 그림에서 보드 영역을 찾아 우리 그림 크기로 맞춘 흰색/주황 마스크를 돌려줍니다."""
    im = np.asarray(Image.open(ref_png).convert('RGB')).astype(int)
    floor = (np.abs(im - np.array(FLOOR)).sum(axis=2) < 20)
    ys, xs = np.where(floor)
    crop = im[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    out = []
    for c, tol in ((WHITE, 90), (ORANGE, 90)):
        m = np.abs(crop - np.array(c)).sum(axis=2) < tol
        out.append(np.asarray(Image.fromarray(m.astype(np.uint8) * 255).resize((shape[1], shape[0]), Image.NEAREST)) > 0)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dxf')
    ap.add_argument('-o', '--out', default='track.usda')
    ap.add_argument('--unit', type=float, required=True, help='도면 1단위가 몇 m인지 (대회 도면: 0.03)')
    ap.add_argument('--layers', default='Visible', help='쉼표 구분. 대회 도면은 Visible (치수선 제외)')
    ap.add_argument('--px-mm', type=float, default=2.0, help='바닥 그림 1픽셀 크기 mm')
    ap.add_argument('--max-fill-mm', type=float, default=100.0,
                    help='이 폭 이하의 닫힌 칸만 칠합니다 (테이프 20 mm, 횡단보도 블록 약 70 mm, 마름모 안쪽 약 130 mm는 안 칠함)')
    ap.add_argument('--ref-png', default='', help='주최 측 트랙 그림. 주면 테이프 색을 여기서 가져옴')
    a = ap.parse_args()

    polys = load_polylines(a.dxf, set(s for s in a.layers.split(',') if s), a.unit)
    if not polys:
        sys.exit('선이 없습니다. --layers 를 확인하십시오.')
    xs = [x for p in polys for x, _ in p]
    ys = [y for p in polys for _, y in p]
    ox, oy = min(xs), min(ys)
    W, H = max(xs) - ox, max(ys) - oy
    px = a.px_mm / 1000.0
    nx, ny = int(round(W / px)) + 1, int(round(H / px)) + 1
    print(f'보드 크기 {W:.3f} x {H:.3f} m, 그림 {nx} x {ny} px')

    # 1) 윤곽선 그리기 (그림 위쪽 = +y)
    edge_img = Image.new('L', (nx, ny), 0)
    dr = ImageDraw.Draw(edge_img)
    for p in polys:
        dr.line([((x - ox) / px, (H - (y - oy)) / px) for x, y in p], fill=255, width=1)
    edge = np.asarray(edge_img) > 0

    # 2) 닫힌 칸 중 좁은 것만 칠하기
    lab, n = ndi.label(~edge)
    dist = ndi.distance_transform_edt(~edge)
    maxd = ndi.maximum(dist, lab, index=np.arange(1, n + 1))
    border = set(np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]]))) - {0}
    half = a.max_fill_mm / 2 / a.px_mm
    fill_ids = np.array([i + 1 for i, d in enumerate(maxd) if d <= half and (i + 1) not in border])
    filled = np.isin(lab, fill_ids)
    tape = filled | (edge & ndi.binary_dilation(filled, iterations=2))
    print(f'칠한 칸 {len(fill_ids)}개 / 전체 {n}개')

    # 3) 색 정하기
    rgb = np.empty((ny, nx, 3), np.uint8)
    rgb[:] = FLOOR
    if a.ref_png:
        # 주최 측 그림으로 칸마다 칠할지·무슨 색인지 정합니다.
        #  - 폭 15~30 mm 칸(테이프 한 줄)은 항상 칠함 (점선처럼 주최 그림과 위치가 조금 어긋나도 살림)
        #  - 그보다 넓은 칸은 주최 그림에서 15% 이상이 테이프 색일 때만 칠함 (횡단보도 블록 사이, 글자 속 빈칸 제외)
        #  - 색은 주황 표가 흰색 표보다 많으면 주황, 아니면 흰색
        wm, om = (ndi.binary_dilation(m, iterations=2) for m in ref_masks(a.ref_png, tape.shape)[:2])
        idx = np.arange(1, n + 1)
        area = ndi.sum(np.ones_like(lab), lab, idx)
        wv, ov = ndi.sum(wm, lab, idx), ndi.sum(om, lab, idx)
        region_cls = np.full(n + 1, -1)
        filled[:] = False
        for i in fill_ids:
            width_mm = 2 * maxd[i - 1] * a.px_mm
            frac = (wv[i - 1] + ov[i - 1]) / area[i - 1]
            if (15 <= width_mm <= 30) or frac >= 0.15:
                filled[lab == i] = True
                region_cls[i] = 1 if ov[i - 1] > wv[i - 1] else 0
        tape = filled | (edge & ndi.binary_dilation(filled, iterations=2))
        cls_map = region_cls[lab]
        _, (iy, ix) = ndi.distance_transform_edt(~filled, return_indices=True)
        cls_map = np.where(filled, cls_map, cls_map[iy, ix])  # 윤곽선은 가장 가까운 칠한 칸의 색
        rgb[tape & (cls_map == 0)] = WHITE
        rgb[tape & (cls_map == 1)] = ORANGE
        print('테이프 색: 흰색 %d, 주황 %d 픽셀' % (int((tape & (cls_map == 0)).sum()), int((tape & (cls_map == 1)).sum())))
    else:
        rgb[tape] = WHITE

    out_dir = os.path.dirname(os.path.abspath(a.out))
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(a.out))[0]
    png_name = base + '_floor.png'
    Image.fromarray(rgb).save(os.path.join(out_dir, png_name), optimize=True)

    # 4) USD: 그림을 붙인 바닥판(보이는 것) + 충돌용 얇은 상자(안 보이는 것)
    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/Track')
    stage.SetDefaultPrim(root.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, '/Track/Floor')
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 0), Gf.Vec3f(W, 0, 0), Gf.Vec3f(W, H, 0), Gf.Vec3f(0, H, 0)]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))

    mat = UsdShade.Material.Define(stage, '/Track/Looks/FloorMat')
    sh = UsdShade.Shader.Define(stage, '/Track/Looks/FloorMat/Surface')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.7)
    reader = UsdShade.Shader.Define(stage, '/Track/Looks/FloorMat/StReader')
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    tex = UsdShade.Shader.Define(stage, '/Track/Looks/FloorMat/Texture')
    tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set('./' + png_name)
    tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    tex.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    col = UsdGeom.Cube.Define(stage, '/Track/FloorCollider')
    col.CreateSizeAttr(1.0)
    col.CreateVisibilityAttr(UsdGeom.Tokens.invisible)  # 화면에는 안 보이고 충돌만 함
    xf = UsdGeom.Xformable(col)
    xf.AddTranslateOp().Set(Gf.Vec3d(W / 2, H / 2, -0.0105))
    xf.AddScaleOp().Set(Gf.Vec3f(W + 1.0, H + 1.0, 0.02))
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())

    stage.GetRootLayer().Save()
    print(f'저장: {a.out}, {os.path.join(out_dir, png_name)}')


if __name__ == '__main__':
    main()
