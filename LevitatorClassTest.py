from ctypes import CDLL, POINTER
import ctypes

import torch

from acoustools.Utilities import create_points, get_convert_indexes, add_lev_sig
from acoustools.Solvers import wgs_wrapper



class LevitatorController():

    def __init__(self, dll_path = 'x64\Debug\Levitator.dll', ids = (999,1000), matBoardToWorld=None, print=False):
        self.levitatorLib = CDLL(dll_path)

        self.ids = (ctypes.c_int * 3)(999,1000)
        self.board_number = len(ids)

        if matBoardToWorld is None:
            self.matBoardToWorld =  (ctypes.c_float * (16*self.board_number)) (
                -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, -1, 0.24,
                0, 0, 0, 1,

                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1,
            )
        else:
            self.matBoardToWorld =  (ctypes.c_float * (16*self.board_number))(matBoardToWorld)

        self.levitatorLib.connect_to_levitator.restype = ctypes.c_void_p
        self.controller = self.levitatorLib.connect_to_levitator(self.ids,self.matBoardToWorld,self.board_number,print)

        self.IDX = get_convert_indexes()
    
    def set_phase_amplitude(self, phases, amplitudes=None, relative_amplitude=1):
        self.levitatorLib.set_phase_amplitude.argtypes = [ctypes.c_void_p, POINTER(ctypes.c_float), POINTER(ctypes.c_float), ctypes.c_float]
        self.levitatorLib.set_phase_amplitude(self.controller,phases,amplitudes,relative_amplitude)
    
    def send_message(self):
        self.levitatorLib.send_message.argtypes = [ctypes.c_void_p]
        self.levitatorLib.send_message(self.controller)
    
    def disconnect(self):
        self.levitatorLib.disconnect.argtypes = [ctypes.c_void_p]
        self.levitatorLib.disconnect(self.controller)
    
    def turn_off(self):
        self.levitatorLib.turn_off.argtypes = [ctypes.c_void_p]
        self.levitatorLib.turn_off(self.controller)

    def levitate(self, phases, amplitudes=None, relative_amplitude=1, permute=True):
        '''
        Send a single phase map to the levitator - This is the reccomended function to use as will deal with dtype conversions etc\\
        `phases`: `Torch.Tensor` of phases, expects a batched dimension in dim 0 and takes and sends ONLY THE FIRST HOLOGRAM. If phases is complex then ` phases = torch.angle(phases)` will be run, else phases left as is\\
        `amplitudes`: Optional `Torch.Tensor` of amplitudes\\
        `relative_amplitude`: Single value [0,1] to set amplitude to. Default 1\\
        `permute`: Convert between acoustools transducer order and OpenMPD. Default True.
        '''
        
        if permute:
            phases = phases[:,self.IDX]

        if torch.is_complex(phases):
            phases = torch.angle(phases)
        
        phases = phases[0].squeeze().cpu().detach().tolist()

        phases = (ctypes.c_float * (256*self.board_number))(*phases)

        if amplitudes is not None:
            amplitudes = (ctypes.c_float * (256*self.board_number))(*amplitudes)

        relative_amplitude = ctypes.c_float(1.0)

        self.set_phase_amplitude(phases, amplitudes, relative_amplitude)
        self.send_message()



if __name__ == '__main__':
    lev = LevitatorController()

    p = create_points(1,1,x=0,y=0,z=0)
    x = wgs_wrapper(p)
    x = add_lev_sig(x)

    lev.levitate(x)
    input()
    lev.disconnect()

