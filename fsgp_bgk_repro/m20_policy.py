"""Shared M20 checkpoint reader and NumPy actor for independent policy tests.

This module contains no route planner, scene generator or obstacle benchmark.
"""
import collections
import hashlib
import io
import os
import pickle
from pathlib import Path
import site
import sys
import zipfile

import numpy as np

EXTERNAL = Path('/home/jiang/m20-repro')
DEFAULT_CHECKPOINT = EXTERNAL/'outputs/official-rough-local/logs/rsl_rl/deeprobotics_m20_rough/2026-10-18_22-26-49_official_rough_resume_15k/model_14999.pt'
DEFAULT_XML = EXTERNAL/'src/rl_training/deep_robotics_model/M20/mjcf/M20.xml'
LEGS = [f'{leg}_{joint}_joint' for leg in ('fl', 'fr', 'hl', 'hr') for joint in ('hipx', 'hipy', 'knee')]
WHEELS = [f'{leg}_wheel_joint' for leg in ('fl', 'fr', 'hl', 'hr')]
JOINTS = LEGS+WHEELS
DEFAULT_Q = np.array([0, -.3, .6, 0, -.3, .6, 0, .3, -.6, 0, .3, -.6, 0, 0, 0, 0.])


def _mujoco():
    """Use installed headless MuJoCo without replacing the loaded NumPy/SciPy."""
    try:
        import mujoco
    except ModuleNotFoundError:
        os.environ.setdefault('MUJOCO_GL','disable')
        sys.path.append(site.getusersitepackages())
        import mujoco
    return mujoco


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_checkpoint(path):
    """Read plain float tensor state dicts; reject arbitrary pickle globals."""
    with zipfile.ZipFile(path) as archive:
        entry = next(n for n in archive.namelist() if n.endswith('/data.pkl'))
        prefix = entry.rsplit('/', 1)[0]
        storage = {}

        def tensor(raw, offset, shape, stride, *_):
            return np.lib.stride_tricks.as_strided(raw[offset:], shape=shape,
                strides=tuple(s*raw.dtype.itemsize for s in stride)).copy()

        class Reader(pickle.Unpickler):
            def find_class(self, module, name):
                allowed = {('collections', 'OrderedDict'): collections.OrderedDict,
                    ('torch', 'FloatStorage'): np.dtype('<f4'),
                    ('torch._utils', '_rebuild_tensor_v2'): tensor}
                if (module, name) not in allowed:
                    raise ValueError(f'Unsupported checkpoint global: {module}.{name}')
                return allowed[module, name]

            def persistent_load(self, pid):
                kind, dtype, key, _, count = pid
                if kind != 'storage' or dtype != np.dtype('<f4'):
                    raise ValueError('Only ordinary float32 tensor storage is supported')
                if key not in storage:
                    storage[key] = np.frombuffer(archive.read(f'{prefix}/data/{key}'), dtype=dtype)
                if len(storage[key]) != count:
                    raise ValueError('Invalid tensor storage length')
                return storage[key]

        return Reader(io.BytesIO(archive.read(entry))).load()


class NumpyActor:
    def __init__(self, path):
        checkpoint = read_checkpoint(path)
        state = checkpoint.get('actor_state_dict', checkpoint.get('model_state_dict'))
        self.layers = [(state[f'mlp.{i}.weight'], state[f'mlp.{i}.bias']) for i in (0, 2, 4, 6)]
        self.iteration = checkpoint['iter']
        if self.layers[0][0].shape[1] != 57 or self.layers[-1][0].shape[0] != 16:
            raise ValueError('This adapter requires the 57-observation / 16-action blind velocity policy')

    def __call__(self, observation):
        x = np.asarray(observation, dtype=np.float32)
        for i, (w, b) in enumerate(self.layers):
            x = w@x+b
            if i < len(self.layers)-1:
                x = np.maximum(x, 0)+np.expm1(np.minimum(x, 0))
        return np.clip(x, -100, 100)

