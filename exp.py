import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.positioning.motion_commander import MotionCommander

from math import pi,sin,cos

# Custom lib for quaternion algebra
from quaternion import *


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
formatter = logging.Formatter('%(message)s') # logging.Formatter('%(asctime)s - %(levelname)s | %(message)s')
console_handler = logging.FileHandler('console.log')        
console_handler.setFormatter(formatter)
console_logger = logging.getLogger('console')
console_logger.setLevel(logging.INFO)
console_logger.addHandler(console_handler)
state_handler = logging.FileHandler('states.log')        
state_handler.setFormatter(formatter)
state_logger = logging.getLogger('state')
state_logger.setLevel(logging.INFO)
state_logger.addHandler(state_handler)


G = 9.81
m = 0.03
K_HAT = PureQuaternion([0,0,1])


class Experiment():
    def __init__(self, crazyflie_id: int, channel : int = 100, dt : float = 0.01, Tmax : int = 300, p0 : list = [0.0,0.0,0.0], v0: list = [0.0,0.0,0.0]):
        # Initialize the low-level drivers
        cflib.crtp.init_drivers()
        # URI to the Crazyflie to connect to
        self._uri = uri_helper.uri_from_env(
            default='radio://0/%d/2M/E7E7E7E7%02d' % (channel, crazyflie_id)
        ) 
        # Experiment constants
        self._dt = dt
        self._Tmax = Tmax
        # Init variables for in-the-loop simulation part
        self.__p = PureQuaternion(p0)
        self.__v = PureQuaternion(v0) 
        self.__prev_t = None  
        self.__prev_F = None   

    def run(self):
        with SyncCrazyflie(self._uri, cf=Crazyflie(rw_cache='./cache')) as self.scf:
            print("Connected to crazyflie. Starting experiment...")
            # log console
            self.scf.cf.console.receivedChar.add_callback(self._log_console)
            # Add logger with callback
            lg_stab = LogConfig(name='Stabilizer', period_in_ms=int(self._dt*1000))
            lg_stab.add_variable('stateEstimate.qw', 'float')
            lg_stab.add_variable('stateEstimate.qx', 'float')
            lg_stab.add_variable('stateEstimate.qy', 'float')
            lg_stab.add_variable('stateEstimate.qz', 'float')
            # lg_stab.add_variable('sensfusion6.qw', 'float')
            # lg_stab.add_variable('sensfusion6.qx', 'float')
            # lg_stab.add_variable('sensfusion6.qy', 'float')
            # lg_stab.add_variable('sensfusion6.qz', 'float')
            self.scf.cf.log.add_config(lg_stab)
            # # Test propperlers
            # self._test_proppelers() 
            print('Flying...')
            try:
                # Set parameters
                self._set_params()
                lg_stab.data_received_cb.add_callback(self._callback)
                lg_stab.start()
                # Lock until experiment is finished
                time.sleep(self._Tmax)
            finally:
                self.scf.cf.commander.send_stop_setpoint()
                self.scf.cf.commander.send_notify_setpoint_stop()
                print('Done')
                lg_stab.stop()
                print("Finished experiment. Disconnecting...")

    def _test_proppelers(self):
        # Test properllers
        print('Testing properlers...')
        self.scf.cf.param.set_value('health.startPropTest', 0)
        self.scf.cf.param.set_value('health.startPropTest', 1)
        time.sleep(8)
        self.scf.cf.param.set_value('health.startPropTest', 0)
        print('Done testing properlers')

    def _set_params(self):
        print("Setting parameters")
        self.scf.cf.param.set_value('stabilizer.estimator', 3)
        self.scf.cf.param.set_value('stabilizer.controller', 5)
        # time.sleep(1)
        self.scf.cf.param.set_value('ctrlAtt.robust_controller', 1.0)
        self.scf.cf.param.set_value('ctrlAtt.att_thrust', 4.0)
        self.scf.cf.param.set_value('ctrlAtt.saturate_xi3_hat', 0.001)
        self.scf.cf.param.set_value('ctrlAtt.saturate_xi4_hat', 0.001)
        # self.scf.cf.param.set_value('ctrlAtt.xi_r', 0.0005)
        self.scf.cf.param.set_value('ctrlAtt.k_varphi_e', 2.5)
        self.scf.cf.param.set_value('ctrlAtt.epsilon_o', 2e-3)
        self.scf.cf.param.set_value('ctrlAtt.k_e_w', 0.001)
        self.scf.cf.param.set_value('ctrlAtt.k_e_o', 0.001)
        self.scf.cf.param.set_value('ctrlAtt.rho_3', 2.5e-5)
        self.scf.cf.param.set_value('ctrlAtt.rho_4', 2.5e-5)
        time.sleep(2)
        print("Done")

    def _callback(self, timestamp: int, data: dict, logconf: LogConfig):
        self._t = timestamp/1000.0
        data['time'] = timestamp
        if self.__prev_t is None:
            self.__prev_t = self._t
        dt = self._t - self.__prev_t
        self.__prev_t = self._t
        data['dt'] = dt
        o = Quaternion([data['stateEstimate.qw'],data['stateEstimate.qx'],data['stateEstimate.qy'],data['stateEstimate.qz']])
        od,u_T = self._controller(o, self._t, dt)
        self._in_the_loop_integrate(o,u_T,self._t, dt)
        data['target_qx'] = od.imag[0]
        data['target_qy'] = od.imag[1]
        data['target_qz'] = od.imag[2]
        data['target_qw'] = od.real
        data['px'] = self.__p.imag[0]
        data['py'] = self.__p.imag[1]
        data['pz'] = self.__p.imag[2]
        data['vx'] = self.__v.imag[0]
        data['vy'] = self.__v.imag[1]
        data['vz'] = self.__v.imag[2]
        data['u_T'] = u_T
        state_logger.info(f'{data}')
        self.scf.cf.commander.send_velocity_world_setpoint(-data['target_qx'],data['target_qy'],data['target_qz'],data['target_qw'])
        

    def _controller(self, o: Quaternion, t: float, dt: float):
        goal = PureQuaternion([-1,2,5])
        z_p = self.__p - goal
        F = -2*(z_p)
        if F.norm > 2:
            F = (2/F.norm)*F
        if self.__prev_F is None:
            self.__prev_F = F
        try:
            dFdt = (F - self.__prev_F)/dt
        except:
            dFdt = PureQuaternion([0,0,0])
        self.__prev_F = F
        z_v = self.__v - F
        f_L = -2*z_p - 2*z_v
        f = m*(dFdt + G*K_HAT + f_L)
        f = PureQuaternion(f.imag)
        
        # f = PureQuaternion([0.0,0.0,1.0])
        
        n_z = (1/f.norm)*K_HAT.cross(f)
        if n_z.norm == 0:
            n_z = K_HAT
        elif (n_z.norm - 1)**2 > 0.01**2:
            n_z = (1/n_z.norm)*n_z
        phi_z = np.arccos(K_HAT.dot(f)/f.norm)
        o_z = ((phi_z/2)*n_z).exp(always_quaternion=True)
        o_d = o_z
        k_B = o*K_HAT*o.conj(); k_B = PureQuaternion(k_B.imag)
        u_T = k_B.dot(f)
        # angle = t/2 #% (2*pi) - pi
        # o_d = Quaternion([cos(angle/2), sin(angle/2), 0, 0])
        o_d = Quaternion([1.0, 0.0, 0.0, 0.0])
        # u_T = 0
        # o_d = Quaternion([cos(t/10), 0.0, 0.0, sin(t/10)])*Quaternion([cos(pi/8), sin(pi/8)*cos(t/10), sin(pi/8)*sin(t/10), 0.0])
        # o_d = Quaternion([cos(pi/12), sin(pi/12)*cos(t/10), sin(pi/12)*sin(t/10), 0.0])
        return o_d,u_T
    
    def _in_the_loop_integrate(self, o: Quaternion, u_T: float, t: float, dt: float):
        o_data = o.data
        o_fixed = Quaternion([
            -o_data[0], #w
            -o_data[1], #x
            o_data[2], #y
            o_data[3] #z
        ])
        self.__p = self.__p + (dt/2)*self.__v
        k_B = o_fixed*K_HAT*o_fixed.conj(); k_B = PureQuaternion(k_B.imag)
        self.__v = self.__v + (dt/2)*(-G*K_HAT + (u_T/m)*k_B )

    def _log_console(self, text):
        console_logger.info(text)


if __name__ == '__main__':
    exp = Experiment(4)
    exp.run()
