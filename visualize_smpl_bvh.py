import argparse
import inspect
import sys
import numpy as np

# ============================================================
# 【最关键的补丁】必须在 import smplx 之前执行
# 修复新版 NumPy (1.20+) 移除旧类型名导致的 chumpy 崩溃
# ============================================================
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

for name in ['bool', 'int', 'float', 'complex', 'object', 'str', 'unicode']:
    if name not in np.__dict__:
        if name == 'bool': setattr(np, name, bool)
        elif name == 'int': setattr(np, name, int)
        elif name == 'float': setattr(np, name, float)
        elif name == 'complex': setattr(np, name, complex)
        elif name == 'object': setattr(np, name, object)
        elif name == 'str': setattr(np, name, str)
        elif name == 'unicode': setattr(np, name, str)

# pyrender 0.1.45 still uses the NumPy 1.x spelling internally.
if 'infty' not in np.__dict__:
    np.infty = np.inf

import zmq
import torch
import smplx
import pyrender
import trimesh
import time
import json
import os

HEADER_SIZE = 1280
DEFAULT_MODEL_PATH = (
    r"G:\SMPL_python_v.1.1.0\smpl\models\basicmodel_neutral_lbs_10_207_0_v1.1.0.pkl"
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Render SMPL poses from the local ZMQ publisher")
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH)
    parser.add_argument("--zmq-url", default="tcp://127.0.0.1:5556")
    return parser.parse_args(argv)

def manual_unpack_pose(msg):
    """解析 ZMQ 传输的二进制协议"""
    try:
        header_start = msg.find(b'{')
        if header_start == -1: return None
        header_bytes = msg[header_start : header_start + HEADER_SIZE].rstrip(b'\x00')
        header = json.loads(header_bytes.decode('utf-8'))
        payload = msg[header_start + HEADER_SIZE :]
        
        data = {}
        curr_offset = 0
        dtype_map = {
            "f32": (np.float32, 4), "f64": (np.float64, 8), 
            "i32": (np.int32, 4), "i64": (np.int64, 8), 
            "bool": (np.bool_, 1), "u8": (np.uint8, 1)
        }
        for field in header["fields"]:
            name = field["name"]; dtype_str = field["dtype"]; shape = field["shape"]
            dtype, item_size = dtype_map.get(dtype_str, (np.float32, 4))
            num_elements = np.prod(shape)
            byte_len = int(num_elements * item_size)
            data[name] = np.frombuffer(payload[curr_offset : curr_offset + byte_len], dtype=dtype).copy().reshape(shape)
            curr_offset += byte_len
        return data
    except Exception: return None

class RemoteSMPLViz:
    def __init__(self, model_path=DEFAULT_MODEL_PATH, zmq_url="tcp://127.0.0.1:5556"):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"[*] 使用设备: {self.device}")
        
        # The official SMPL 1.1.0 filename does not follow smplx.create naming.
        try:
            if not os.path.isfile(model_path):
                raise FileNotFoundError(model_path)
            self.model = smplx.SMPL(model_path, gender='neutral').to(self.device)
        except Exception as e:
            print(f"[错误] 无法加载 SMPL 模型: {e}")
            sys.exit(1)
            
        self.faces = self.model.faces
        
        # ZMQ 订阅端
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(zmq_url)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "") 
        self.socket.setsockopt(zmq.CONFLATE, 1)          
        
        # 场景设置
        self.scene = pyrender.Scene(ambient_light=[0.5, 0.5, 0.5], bg_color=[0.1, 0.1, 0.1])
        
        # 机器人坐标轴 (X-红, Y-绿, Z-蓝)
        axis_mesh = trimesh.creation.axis(origin_size=0.04, axis_radius=0.015, axis_length=0.5)
        render_axis = pyrender.Mesh.from_trimesh(axis_mesh, smooth=False)
        axis_pose = np.eye(4)
        axis_pose[:3, 3] = [-3.0, -1.15, -3.0] 
        self.scene.add(render_axis, pose=axis_pose)

        # XZ 平面参考地面 
        grid_mesh = trimesh.creation.box(extents=[6, 0.001, 6]) 
        grid_mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[0.3, 0.3, 0.3, 0.5], alphaMode='BLEND')
        render_grid = pyrender.Mesh.from_trimesh(grid_mesh, material=grid_mat)
        grid_pose = np.eye(4)
        grid_pose[1, 3] = -1.15
        self.scene.add(render_grid, pose=grid_pose)
        
        # 初始相机：稍微拉远一点
        self.camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
        camera_pose = np.eye(4)
        camera_pose[:3, 3] = [0, 0, 4] # 正对原点后退 4 米
        self.camera_node = self.scene.add(self.camera, pose=camera_pose)
        
        # 启动渲染器 (移除 caption 避免报错)
        self.viewer = pyrender.Viewer(self.scene, run_in_thread=True, use_raymond_lighting=True)
        self.mesh_node = None

    def run(self):
        print("[*] 正在实时渲染 SMPL ...")
        try:
            while self.viewer.is_active:
                try:
                    raw_payload = self.socket.recv(flags=zmq.NOBLOCK)
                    data = manual_unpack_pose(raw_payload)
                    if data is None: continue

                    go = torch.from_numpy(data["smpl_global_orient"][-1:]).float().to(self.device)
                    bp = torch.from_numpy(data["smpl_body_pose"][-1:]).float().to(self.device).view(1, 69)
                    tr = torch.zeros((1, 3)).to(self.device)

                    with torch.no_grad():
                        output = self.model(body_pose=bp, global_orient=go, transl=tr)
                    
                    vertices = output.vertices[0].cpu().numpy()

                    material = pyrender.MetallicRoughnessMaterial(
                        baseColorFactor=[0.1, 0.9, 0.5, 1.0], 
                        metallicFactor=0.3, 
                        roughnessFactor=0.6
                    )
                    
                    tri_mesh = trimesh.Trimesh(vertices, self.faces)
                    render_mesh = pyrender.Mesh.from_trimesh(tri_mesh, material=material)

                    self.viewer.render_lock.acquire()
                    if self.mesh_node:
                        self.scene.remove_node(self.mesh_node)
                    self.mesh_node = self.scene.add(render_mesh)
                    self.viewer.render_lock.release()

                except zmq.Again:
                    time.sleep(0.002)
                except Exception as e:
                    pass 
        finally:
            if self.viewer.is_active:
                self.viewer.close_external()

if __name__ == "__main__":
    args = parse_args()
    viz = RemoteSMPLViz(model_path=args.model_path, zmq_url=args.zmq_url)
    viz.run()
