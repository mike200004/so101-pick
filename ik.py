# Python wrapper around ik.c (run `make` first)
import ctypes
import os

lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), "libik.so"))
lib.so101_ik.argtypes = [ctypes.c_double] * 3 + [ctypes.POINTER(ctypes.c_double)]
lib.so101_ik.restype = ctypes.c_int


def ik(x, y, z):
    """Returns [pan, lift, elbow, wrist_flex] or None if out of reach."""
    q = (ctypes.c_double * 4)()
    if lib.so101_ik(x, y, z, q) != 0:
        return None
    return list(q)
