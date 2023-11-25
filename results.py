import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import sys
import os
import shutil

import json

from quaternion import *

kappa = 0.2785
eps_o = 2e-3
k_eo = 0.001
k_phie = 2.5
DELTA_O = np.sqrt(3*kappa*eps_o/k_eo/k_phie)

class Result():

    def __init__(self, name):
        self.__name = name
        if os.path.exists(f'results/{self.__name}'): raise RuntimeError(f'Results "{self.__name}" already exists')
        os.mkdir(f'results/{self.__name}')

    def _read_data(self):
        self.__time_array = []
        self.__o_array = []
        self.__od_array = []
        self.__p_array = []
        self.__v_array = []
        self.__e_o_array = []
        with open('states.log', 'r') as fh:
            for line in fh.readlines():
                d = json.loads(line.replace("'",'"'))
                # max = 273120#267120
                # if d['time'] > max:
                #     break
                self.__time_array.append(d['time']/1000.0)
                o = Quaternion([d['stateEstimate.qw'],d['stateEstimate.qx'],d['stateEstimate.qy'],d['stateEstimate.qz']])
                od = Quaternion([-d['target_qw'],-d['target_qx'],d['target_qy'],d['target_qz']])
                e_o = 2*(o*od.inv()).log()
                try:
                    p = PureQuaternion([d['px'],d['py'],d['pz']])
                    self.__p_array.append(p.imag)
                    v = PureQuaternion([d['vx'],d['vy'],d['vz']])
                    self.__v_array.append(v.imag)
                except:
                    pass
                self.__o_array.append(o.data)
                self.__od_array.append(od.data)
                self.__e_o_array.append(e_o.imag)
        self.__time_array = np.array(self.__time_array)

    def _move_data(self):
        for file in ['states.log', 'console.log']:
            shutil.move(f"{file}", f"results/{self.__name}/{file}")


    def save(self):
        self._read_data()
        plt.figure()
        plt.plot(self.__time_array, self.__o_array, label=['o_w', 'o_x', 'o_y', 'o_z'])
        plt.plot(self.__time_array, self.__od_array, '--', label=['od_w', 'od_x', 'od_y', 'od_z'])
        plt.legend()
        plt.grid('on')
        plt.savefig(f'results/{self.__name}/orient.jpg')
        plt.close()

        plt.figure()
        plt.plot(self.__time_array, self.__e_o_array, '-.', linewidth=1, markersize=12, label=['e_x', 'e_y', 'e_z'])
        plt.plot(self.__time_array, [np.linalg.norm(e_o) for e_o in self.__e_o_array], color='k', linewidth=2, markersize=12, label='|e|')
        plt.hlines(y=DELTA_O,xmin=self.__time_array[0],xmax=self.__time_array[-1], color='r', linestyles='--', linewidth=0.5, label='delta_o')
        plt.legend()
        plt.grid('on')
        plt.savefig(f'results/{self.__name}/errors.jpg')
        plt.close()

        if self.__p_array:
            plt.figure()
            plt.plot(self.__time_array, self.__p_array, label=['p_x', 'p_y', 'p_z'])
            plt.legend()
            plt.grid('on')  
            plt.savefig(f'results/{self.__name}/position.jpg')
            plt.close()

            plt.figure()
            plt.plot(self.__time_array, self.__v_array, label=['v_x', 'v_y', 'v_z'])
            plt.legend()
            plt.grid('on')  
            plt.savefig(f'results/{self.__name}/velocity.jpg')
            plt.close()
        self._move_data()



# plt.show()
if __name__ == '__main__':
    name = sys.argv[1]
    r = Result(name)
    r.save()
