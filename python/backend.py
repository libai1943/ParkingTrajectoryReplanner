"""Public ABI wrapper for the protected parking foundation; no LIOM/search source."""
from __future__ import annotations

import ctypes as ct
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
POINTER = ct.POINTER(ct.c_double)
_dll_directories = []


def load_library():
    if os.name != "nt":
        raise RuntimeError("This protected backend build targets Windows x64. See README for deployment scope.")
    import casadi

    if casadi.__version__ != "3.7.2":
        raise RuntimeError("The supplied binary requires casadi==3.7.2.")
    runtime = Path(casadi.__file__).resolve().parent
    os.environ["PATH"] = str(runtime) + os.pathsep + os.environ.get("PATH", "")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    _dll_directories.append(os.add_dll_directory(str(runtime)))
    library = ct.CDLL(str(ROOT / "native" / "win64" / "parking_backend.dll"))
    library.ps_error.restype = ct.c_char_p
    library.ps_load.argtypes = [ct.c_char_p, ct.c_int]
    library.ps_load.restype = ct.c_void_p
    library.ps_free.argtypes = [ct.c_void_p]
    library.ps_metadata.argtypes = [ct.c_void_p, POINTER]
    for name in ["ps_original", "ps_evasive"]:
        getattr(library, name).argtypes = [ct.c_void_p, POINTER, ct.c_int]
    library.ps_connect.argtypes = [POINTER, POINTER, POINTER, ct.c_int]
    library.ps_collision_free.argtypes = [ct.c_void_p, POINTER, ct.c_int]
    return library


def pointer(array):
    return array.ctypes.data_as(POINTER)


class ParkingBackend:
    def __init__(self, case_id=1):
        self.library = load_library()
        self.handle = self.library.ps_load(os.fsencode(ROOT / "data" / "cases_data.mat"), case_id)
        if not self.handle:
            raise RuntimeError(self.error())
        self.metadata = np.empty(4)
        self.library.ps_metadata(self.handle, pointer(self.metadata))

    def error(self):
        return self.library.ps_error().decode("utf-8", errors="replace")

    def close(self):
        if self.handle:
            self.library.ps_free(self.handle)
            self.handle = None

    def original(self):
        rows = self.library.ps_original(self.handle, None, 0)
        output = np.empty((rows, 8))
        self.library.ps_original(self.handle, pointer(output), rows)
        return output

    def evasive(self):
        output = np.empty((100, 8))
        count = self.library.ps_evasive(self.handle, pointer(output), len(output))
        if count < 0:
            raise RuntimeError(self.error())
        return output[:count]

    def connect(self, start, finish):
        start, finish = np.ascontiguousarray(start, dtype=float), np.ascontiguousarray(finish, dtype=float)
        output = np.empty((100, 8))
        count = self.library.ps_connect(pointer(start), pointer(finish), pointer(output), len(output))
        if count < 0:
            raise RuntimeError(self.error())
        return output[:count]

    def collision_free(self, trajectory):
        trajectory = np.ascontiguousarray(trajectory, dtype=float)
        return bool(self.library.ps_collision_free(self.handle, pointer(trajectory), len(trajectory)))
