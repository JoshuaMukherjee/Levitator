from ctypes import CDLL, POINTER
import ctypes

import torch

from acoustools.Utilities import create_points, get_convert_indexes
from acoustools.Solvers import wgs_wrapper


levitatorLib = CDLL('x64\Debug\Levitator.dll')

ids = (ctypes.c_int * 3)(999,1000)

matBoardToWorld =  (ctypes.c_float * 32) (
	-1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, -1, 0.24,
	0, 0, 0, 1,

	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1,
)

levitatorLib.connect_to_levitator.restype = ctypes.c_void_p
controller = levitatorLib.connect_to_levitator(ids,matBoardToWorld,2,True)

# levitatorLib.disconnect.argtypes = [ctypes.c_void_p]
# levitatorLib.disconnect(ctypes.c_void_p(controller))

p = create_points(1,1,x=0,y=0,z=0)
x = wgs_wrapper(p)
x = torch.angle(x)

IDX = get_convert_indexes()
x = x[:,IDX]
x_list = x[0].squeeze().cpu().detach().tolist()

phases = (ctypes.c_float * 512)(*x_list)


amp = (ctypes.c_float * 1)(0.0)

rel_amp = ctypes.c_float(1.0)

print("Setting Phases...")
levitatorLib.set_phase_amplitude.argtypes = [ctypes.c_void_p, POINTER(ctypes.c_float), POINTER(ctypes.c_float), ctypes.c_float]
levitatorLib.set_phase_amplitude(controller,phases,None,rel_amp)

print("Sening Messages to Board...")
levitatorLib.send_message.argtypes = [ctypes.c_void_p]
levitatorLib.send_message(controller)


input('Press enter to Stop')

levitatorLib.turn_off.argtypes = [ctypes.c_void_p]
levitatorLib.turn_off(controller)
