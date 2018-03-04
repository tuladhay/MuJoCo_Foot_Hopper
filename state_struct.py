# State pointer
import ctypes

class State(ctypes.Structure):
    _fields_ = [('q', ctypes.c_double*6),
                ('qd', ctypes.c_double*6),
                ('qdd', ctypes.c_double*6),
                ('u', ctypes.c_double*3),
                ('t', ctypes.c_double),
                ('cpos', ctypes.c_double*2)]