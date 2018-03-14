# State pointer
import ctypes

class State(ctypes.Structure):
    _fields_ = [('q', ctypes.c_double*6), # x(starts at 0), z(starts at 1.2m), hip, leg_slide(0.45 to 0.6), spring, ankle
                ('qd', ctypes.c_double*6),
                ('qdd', ctypes.c_double*6),
                ('u', ctypes.c_double*3), # hip, leg, ankle
                ('t', ctypes.c_double),
                ('cpos', ctypes.c_double*2)] # heel, toe (z positions)