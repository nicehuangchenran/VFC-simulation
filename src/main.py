from datetime import datetime
import pandas as pd
import numpy as np
from V2VChannel import V2VChannel


class Task:
    def __init__(self):
        self.id = -1
        self.size = -1
        self.epslion = -1
        self.tau = -1  # 单位ms
        self.cycle = -1


class Transactoin:
    def __init__(self):
        self.rv = -1
        self.fv = -1
        self.task = None
        self.time = None


class Vehicle:
    def __init__(self, id, position, angle, speed, fre, account, score):
        self.id = id
        self.position = position
        self.velocity = speed
        self.direction = angle
        self.CPU_frequency = fre
        self.off_tasks = []
        self.rec_tasks = []
        self.failed_tasks = []
        self.__activated = True
        self.account = account
        self.score = score
        self.time = 0

    def finish_it(self):
        # 停车了但是也要计算任务，只是不能进行position renew和传输了
        assert self.is_running
        self.__activated = False
        self.position = [-10000, -10000]  # 不出现在任何的neighbor之中
        self.CPU_frequency = 0

    @property
    def is_running(self):
        return self.__activated

    def update_position(self, position):
        assert self.is_running
        self.position = position

    def update_velocity(self, velocity):
        assert self.is_running
        self.velocity = velocity

    def update_direction(self, direction):
        assert self.is_running
        self.direction = direction

    def update_time(self, time):
        assert self.is_running
        self.time = time
        self.time = round(self.time, 1)


class Environment:
    def __init__(self):
        self.V2V_power_dB = 23
        self.n_veh = 0
        self.vehicles = []
        self.n_rb=20
        self.cur_time = 0
        self.time_step = 1
        # 信道
        self.V2V_active_links = np.zeros((self.n_veh, self.n_veh), dtype='bool')
        self.V2V_channel=V2VChannel(self.n_veh,self.n_rb)

        self.load_sumo_trace()
        self.renew_veh_positions()



    def load_sumo_trace(self):
        trace_path = '../data/test.xlsx'
        print('读取sumo路径文件\'%s\'...' % trace_path)
        self.sumo_trace = pd.read_excel(trace_path)  # 读取sumo的路径
        print('读取完成')

    def generate_one_vehicle(self, data):
        # 从data中生成一辆车
        veh = Vehicle(data['id'], [data['x'], data['y']], data['angle'], data['speed'], data['cpu'], data['account'],
                      data['score'])
        veh.time = self.cur_time
        return veh

    def in_vehicle(self, v_id):
        # 根据v_id判断是否在列表中
        for v in self.vehicles:
            if v.id == v_id:
                return True
        return False

    def deactivate_veh(self, v_idx):
        # 1 车辆自身结束
        vi = self.vehicles[v_idx]
        vi.finish_it()  # 不移除，返回车辆的activated列表
        # 计算任务判断为失败
        vi.failed_tasks.extend(vi.rec_tasks)
        # 2 deactivate链路
        self.V2V_active_links[v_idx, :] = False
        self.V2V_active_links[:, v_idx] = False

    def renew_veh_positions(self):
        '''读取self.sumo_trace，根据time step'''
        df = self.sumo_trace
        tmp = df.loc[df['time'] == self.cur_time, :]
        cnt = 0
        for _ in tmp.index:
            line = tmp.iloc[cnt]
            row_data = line.to_dict()
            cnt += 1
            if not self.in_vehicle(row_data['id']):
                self.n_veh += 1  # 车辆数量，等于len(vehicles)
                vehicle = self.generate_one_vehicle(row_data)
                self.vehicles.append(vehicle)
            else:
                self.update_vehicle(row_data)  # 更新位置、角度和速度
        # 对于当前时间没有位置的车辆，就是消失了，需要不激活
        for (idx, vi) in enumerate(self.vehicles):
            if vi.time != self.cur_time and vi.is_running:
                self.deactivate_veh(idx)
        self.V2V_channel.update_Vnum(self.n_veh)

    def update_vehicle(self, row_data):
        veh = None
        for vi in self.vehicles:
            if vi.id == row_data['id']:
                veh = vi
                veh: Vehicle
                break
        assert veh is not None
        veh.update_direction(row_data['angle'])
        veh.update_position([row_data['x'], row_data['y']])
        veh.update_velocity(row_data['speed'])
        veh.update_time(row_data['time'])


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    env = Environment()
