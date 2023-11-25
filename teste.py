import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E704')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def simple_log(scf: SyncCrazyflie, logconf: LogConfig):
    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            # break


def simple_log_async(scf: SyncCrazyflie, logconf: LogConfig):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()


def log_stab_callback(timestamp, data, logconf: LogConfig):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))


def simple_param_async(scf: SyncCrazyflie, groupstr: str, namestr: str):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    cf.param.add_update_callback(group=groupstr, name=namestr,cb=param_stab_est_callback)
    
    cf.param.set_value(full_name,5)
    time.sleep(1)
    cf.param.set_value(full_name,1)
    time.sleep(1)


def param_stab_est_callback(name: str, value: str):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)


def simple_connect():
    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # simple_connect()
        # simple_log(scf, lg_stab)
        # simple_log_async(scf, lg_stab)
        group = "stabilizer"
        group = "health"
        # simple_param_async(scf, group, 'estimator')
        simple_param_async(scf, group, 'controller')

        # simple_log_async(scf, lg_stab)
