#!/usr/bin/env python3
"""Bake smooth triangulated display outlines from the bundled OFL SDF atlas.
Offline regeneration only: numpy, scikit-image, shapely >= 2.1.
"""
import argparse
from pathlib import Path
import re
import numpy as np
from skimage.measure import find_contours
from shapely import Polygon, constrained_delaunay_triangles
from shapely.geometry import GeometryCollection

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('engine',type=Path,help='JSD engine source directory')
engine=parser.parse_args().engine
root=Path(__file__).resolve().parents[1]
source=engine/'src/runtime/shell_font_atlas.inc'
packed=list(map(int,re.findall(r'\d+',source.read_text().split('\n',1)[1])))
pixels=[]; i=0
while i<len(packed):
    packet=packed[i]; i+=1; count=(packet&127)+1
    if packet&128: pixels.extend(packed[i:i+count]); i+=count
    else: pixels.extend([0]*count)
assert len(pixels)==512*256
atlas=np.array(pixels).reshape(256,512)
vertices=[]; offsets=[0]
for code in range(32,127):
    row,col=divmod(code-32,16)
    cell=atlas[row*40:(row+1)*40,col*32:(col+1)*32]
    shape=GeometryCollection()
    for contour in find_contours(np.pad(cell,1),127.5):
        poly=Polygon([(x-1,y-1) for y,x in contour]).simplify(.12,preserve_topology=True)
        if poly.is_valid and poly.area>.025: shape=shape.symmetric_difference(poly)
    if not shape.is_empty:
        triangles=constrained_delaunay_triangles(shape).geoms
        assert abs(sum(t.area for t in triangles)-shape.area)<1e-5
        for tri in triangles:
            coords=list(tri.exterior.coords)[:3]
            if tri.exterior.is_ccw: coords.reverse()
            vertices.extend((x*6/32,(30.5-y)*7/30.5) for x,y in coords)
    offsets.append(len(vertices))
assert all(offsets[i+1]>offsets[i] for i in range(1,95))
path=root/'game/src/playfield/display_glyphs.inc'
with path.open('w') as f:
    f.write('// Generated from bundled Liberation Mono Bold SDF; third_party/ui_font/OFL.txt.\n')
    f.write('static constexpr unsigned kDisplayOffsets[]={'+','.join(map(str,offsets))+'};\n')
    f.write('static constexpr float kDisplayVertices[][2]={\n')
    for x,y in vertices: f.write(f'    {{{x:.5f}f,{y:.5f}f}},\n')
    f.write('};\n')
print(f'{len(vertices)//3} triangles; all printable ASCII glyphs validated')
