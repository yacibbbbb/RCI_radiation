#!/usr/bin/env python3

# Blender: blender -b -P make_textured_plane.py -- --png ./textures/overlay.png --dae ./meshes/textured_plane.dae
import bpy, argparse, sys, os
for a in bpy.data.objects: a.select_set(True)
bpy.ops.object.delete()

# 블렌더를 통해 Rviz에 Radiation Map을 시각화 하기 위해 
# 기본 1x1 plane에 png 파일을 texture로 설정

# Blender 인자 구분
argv = sys.argv
if "--" in argv:
    argv = argv[argv.index("--") + 1:]
else:
    argv = []  # 인자가 없으면 빈 리스트

parser = argparse.ArgumentParser()
parser.add_argument("--png", required=True)
parser.add_argument("--dae", required=True)
args = parser.parse_args(argv)

# 1x1 평면 생성
bpy.ops.mesh.primitive_plane_add(size=1.0, enter_editmode=False, location=(0,0,0))
plane = bpy.context.active_object

# 이미지 로드 & 머티리얼 생성
img = bpy.data.images.load(os.path.abspath(args.png))
mat = bpy.data.materials.new(name="PNGMat")
mat.use_nodes = True
nodes = mat.node_tree.nodes
links = mat.node_tree.links
for n in nodes: nodes.remove(n)

out = nodes.new('ShaderNodeOutputMaterial')
bsdf = nodes.new('ShaderNodeBsdfPrincipled')
tex = nodes.new('ShaderNodeTexImage')
tex.image = img
links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
links.new(tex.outputs['Alpha'], bsdf.inputs['Alpha'])
links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
mat.blend_method = 'BLEND'  # PNG 알파 사용
mat.shadow_method = 'NONE'

plane.data.materials.append(mat)

# UV (기본 평면은 UV가 깔끔히 들어감. 그래도 안전하게 한번 언랩)
bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.uv.smart_project(); bpy.ops.object.mode_set(mode='OBJECT')

# Collada로 내보내기 (텍스처를 dae 옆 하위 'textures/'로 복사)
dae_path = os.path.abspath(args.dae)
os.makedirs(os.path.dirname(dae_path), exist_ok=True)
bpy.ops.wm.collada_export(filepath=dae_path, check_existing=False, use_texture_copies=True)
print("Exported:", dae_path)
