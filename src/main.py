from collections import deque
from datetime import datetime


import pandas as pd
import numpy as np
# 信道
from V2VChannel import V2VChannel
from V2IChannel import V2IChannel
# 基础设备
from task import Task
from vehicle import Vehicle

class BS:
    # BS simulator
    def __init__(self, start_position, cpu):
        self.position = start_position
        self.I2I_neighbors = []
        self.I2I_destinations = [] # 接受的信息
        self.CPU_frequency = cpu # 当前idle的计算资源
        self.task_queue = []
        self.to_others_task = [] # 传输给其他人的任务（mode, task, to_dev, transmission_time）
        self.computing_res_alloc = [] # 分配的计算资源（任务，分配的资源，开始时间，剩余计算量）
        self.power_capacity = 0 # 当前剩余的电量

class Transactoin:
    def __init__(self):
        self.rv = -1
        self.fv = -1
        self.task = None
        self.time = None


class Environment:
    def __init__(self):
        self.V2V_power_dB = 23
        self.n_Veh = 0
        self.vehicles = []

        self.n_BS = 1  # 先假设环境中先有一个RSU
        self.BSs=[]
        self.BS_positions = [[0,0],[5000,4000],[10000,5000]] # 二维数组格式输入BS的位置
        self.BS_cpu = 5 # 随便设置的

        self.n_RB=20

        #系统时间相关
        self.cur_time = 0
        self.n_step = 0
        self.time_step = 1

        # 任务相关生成数据
        self.task_poisson = 1 # 每秒几个任务，Poisson分布
        self.task_cpu_min = 1 # Giga-Cycle
        self.task_cpu_max = 3
        self.task_data_min = 0.2 # MB
        self.task_data_max = 2
        self.task_ddl_min = 1 # s
        self.task_ddl_max = 5
        self.succeed_tasks = deque(maxlen=3000)  # 保留3000条数据
        self.failed_tasks = deque(maxlen=3000)
        self.failed_transmit_tasks = deque(maxlen=3000)  # 传输失败的任务

        # 每辆车最多由有几个neighbor
        self.v_neighbor_Veh = 3
        self.v_neighbor_BS = 1
        self.B_neighbor_BS = 3
        # 通信相关
        self.bandwidth = 20 # MHz
        self.V2V_power_dB = 23 # dBm 记录的都是最大功率
        self.V2I_power_dB = 26 
        self.sig2_dB = -114
        self.sig2 = 10**(self.sig2_dB/10)
        self.V2V_active_links = np.zeros((self.n_Veh, self.n_Veh), dtype='bool')
        self.V2VChannel=V2VChannel(self.n_Veh,self.n_RB)
        self.V2V_Interference = np.zeros((self.n_Veh, self.v_neighbor_Veh))

        self.V2IChannel = V2IChannel(self.n_Veh, self.n_BS, self.n_RB, self.BS_positions)
        self.V2I_Interference = np.zeros(self.n_Veh) # 默认每个车辆归属于一个基站
        self.V2I_active_links = np.zeros((self.n_Veh, self.n_BS), dtype='bool')

        #self.I2IChannel = I2IChannel(self.n_BS, self.n_RB, self.BS_positions)
        # RV车辆集合与FV车辆集合
        self.RV_queue=[]
        self.FV_queue=[]



        self.load_sumo_trace()
        self.initialize_BSs()
        self.renew_neighbor()


    #导入车辆运动信息
    def load_sumo_trace(self):
        trace_path = '../data/test.xlsx'
        print('读取sumo路径文件\'%s\'...' % trace_path)
        self.sumo_trace = pd.read_excel(trace_path)  # 读取sumo的路径
        print(type(self.sumo_trace.id[1]))
        print('读取完成')

    def initialize_BSs(self):
        for i in range(self.n_BS):
            bs_i = BS(self.BS_positions[i], self.BS_cpu)
            self.BSs.append(bs_i)

    def generate_one_vehicle(self, data):
        # 从data中生成一辆车
        veh = Vehicle(data['id'], [data['x'], data['y']], data['angle'], data['speed'], data['cpu'], data['account'],
                      data['score'])
        veh.time = self.cur_time
        self.n_Veh +=1
        self.vehicles.append(veh)
        print("生成车辆"+str(veh.id))
        return veh

    #
    def in_vehicle(self, v_id):
        # 根据v_id判断是否在列表中
        for v in self.vehicles:
            if v.id == v_id:
                return True
        return False

    #停用车辆
    def deactivate_veh(self, v_idx):
        # 1 车辆自身结束
        vi = self.vehicles[v_idx]
        vi.finish_it()  # 不移除，返回车辆的activated列表
        # 计算任务判断为失败
        vi.failed_tasks.extend(vi.rec_tasks)
        # 2 deactivate链路
        self.V2V_active_links[v_idx, :] = False
        self.V2V_active_links[:, v_idx] = False

    # 更新车辆位置
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
                self.n_Veh += 1  # 车辆数量，等于len(vehicles)
                vehicle = self.generate_one_vehicle(row_data)
                self.vehicles.append(vehicle)
            else:
                self.update_vehicle(row_data)  # 更新位置、角度和速度
        # 对于当前时间没有位置的车辆，就是消失了，需要不激活
        for (idx, vi) in enumerate(self.vehicles):
            if vi.time != self.cur_time and vi.is_running:
                self.deactivate_veh(idx)
        self.update_channel_vNum()

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

    def update_activated_veh(self):
        # 这样就能保证不同时隙内id的属性值是一样的了
        self.activated_vehicles = [vi.is_running for vi in self.vehicles]

    # 生成任务
    def generate_tasks(self):
        #根据泊松分布生成计算任务
        def between(a,b):
            assert a <= b
            return a + np.random.rand() * (b-a)
        task_num = np.random.poisson(self.task_poisson * self.time_step, self.n_Veh)
        for (idx, tnum) in enumerate(task_num):
            vi = self.vehicles[idx]
            vi:Vehicle
            if not vi.is_running: continue
            for _ in range(tnum):
                # 生成时间，cpu, data, ddl,
                importance=np.random.randint(1, 5)
                tmp_task = Task(vi.id, self.cur_time, between(self.task_cpu_min, self.task_cpu_max), between(self.task_data_min, self.task_data_max), importance,between(self.task_ddl_min, self.task_ddl_max))
                vi.generate_task(tmp_task)
                print('生成任务:车辆id'+str(tmp_task.g_veh)+'任务id'+str(tmp_task.id))
    
    # 进行任务卸载决策 ：选车
    def offload_tasks(self, v2v_offload,v2i_offload,v2v_tran,v2i_tran):
        '''车不能卸载其他人的任务，-1表示不卸载，同一时刻只能卸载或接收一个任务, active links'''
        assert len(v2v_offload)  == len(v2i_offload) == self.n_Veh
        
        #初始化车辆间可通信的信道矩阵
        self.V2V_active_links = np.zeros((self.n_Veh, self.n_Veh), dtype='bool')
        self.V2I_active_links = np.zeros((self.n_Veh, self.n_BS), dtype='bool')
       
        for i in range(self.n_Veh):
            vi = self.vehicles[i]
            # 全都是单向信道
            # 车辆vi有需要卸载的任务
            if vi.wait_to_transmit:
                mode, _, _, _ = vi.get_transmit_task_info()

                if mode == 'V2I':
                    self.V2I_active_links[i, vi.to_device] = True
                mode == 'V2V'
                if mode == 'V2V':
                    # 把所有需要卸载任务的车的所有邻居车辆的信道都打开?
                    # 打开RV的信道
                    self.V2V_active_links[i, vi.to_device] = True

        # v2v offload
        for i in range(self.n_Veh):
            vi = self.vehicles[i]
            # 判断vi车辆(RV)是否符合卸载要求
            if vi.not_offload or not vi.is_running: continue # 在启动状态
            if vi.wait_to_transmit: continue

            # 选择FV
            for j in range(self.v_neighbor_Veh):
                if v2v_offload[i,j] == -1:
                    continue

                # 继续判断：判断当前等待传输文件的时间，如果大于time step，就无法卸载
                rx = vi.get_V2V_neighbor(j)
                assert rx != vi
                if rx is None: continue

                # 参数分别为：FVid，task_idx，卸载模式，RB
                vi.offload_task(rx, v2v_offload[i,j], 'V2V',v2v_tran[i,j])
                self.V2V_active_links[i,rx] = True
                break # 只能任务卸载一次
        

        # v2i offload
        for i in range(self.n_Veh):
            vi = self.vehicles[i]
            if vi.not_offload or not vi.is_running: continue
            if vi.wait_to_transmit:
                continue
            for j in range(self.v_neighbor_BS):
                if v2i_offload[i,j] == -1:
                    continue
                rx = vi.V2I_neighbors[j]
                vi.offload_task(rx, v2i_offload[i,j], 'V2I', v2i_tran[i,j])
                self.V2I_active_links[i,rx] = True
                break


    def Compute_Rate(self, v2v_tran, v2i_tran):
        '''按照资源块划分带宽资源'''
        avg_band = self.bandwidth / self.n_RB
        self.Compute_Rate_without_Bandwidth(v2v_tran, v2i_tran)
        self.V2V_Rate = avg_band * self.V2V_Rate
        self.V2I_Rate = avg_band * self.V2I_Rate
        
    
    def Compute_Rate_without_Bandwidth(self, v2v_tran, v2i_tran):
        '''V2V, V2I, V2U, U2U, U2I的传输速率。各种干扰都要考虑, action是针对neighbor进行的二维数组，RB序号'''
        # 由于返回结果的数据太小，直接忽略U2V, I2V, I2U的信道
        # v2i_tran: (n_Veh x v_neighbor_Veh x n_RB)
        
        # 干扰是接收端收到的功率，单位mW
        X2I_Interference = np.zeros((self.n_BS, self.n_RB))
        X2V_Interference = np.zeros((self.n_Veh, self.n_RB))
        
        # 1. 计算所有的signal
       
        # --------------compute V2I signal----------------
        V2I_Signal = np.zeros((self.n_Veh, self.n_BS, self.n_RB))
        for i in range(self.n_RB):
            indexes = np.argwhere(v2i_tran == i)
            for pair in indexes:
                v2i_tx = pair[0]
                vtx = self.vehicles[v2i_tx]
                v2i_rx = pair[1] # 接收方
                bs_rx = vtx.V2I_neighbors[v2i_rx]
                if not self.V2I_active_links[v2i_tx, bs_rx]:
                    continue
                V2I_Signal[v2i_tx, bs_rx, i] = 10 ** ((self.V2I_power_dB - self.V2IChannel_with_fastfading[v2i_tx, bs_rx, i]) / 10)
        

        # --------------compute V2V signal----------------
        V2V_Signal = np.zeros((self.n_Veh, self.n_Veh, self.n_RB))
        # 应该还要考虑上一个时隙的结果对于当前传输的影响
        for i in range(self.n_RB):
            indexes = np.argwhere(v2v_tran == i)
            for pair in indexes:
                v2v_tx = pair[0]
                vtx = self.vehicles[v2v_tx]
                v2v_rx = pair[1] # 接收方
                vrx = vtx.get_V2V_neighbor(v2v_rx)
                if vrx is None or not self.V2V_active_links[v2v_tx, vrx]:
                    continue
                V2V_Signal[v2v_tx, vrx, i] = 10 ** ((self.V2V_power_dB - self.V2VChannel_with_fastfading[v2v_tx, vrx, i]) / 10)  # 毫瓦


        # 2. 分别计算每个链路对X2I, X2U, X2V的干扰，同一个RB的情况下
        
        # 2.1 X2I Interference
        for i in range(self.n_RB):
            # V2I, V2V, V2U, U2U, U2I
            for j in range(self.n_Veh):
                # 检查V2I, V2V, V2U，如果signal>0，再计算发射端到接收端的距离
                v2i_flag = 1 if np.max(V2I_Signal[j, :, i])>0 else 0
                v2v_flag = 1 if np.max(V2V_Signal[j, :, i])>0 else 0
                
                assert v2i_flag + v2v_flag <= 1
                if v2i_flag + v2v_flag >= 1:
                    for k in range(self.n_BS):
                        # 根据vj是那种链路，决定power
                        interference_power_db = self.V2I_power_dB
                        if v2i_flag != 0:
                            interference_power_db = self.V2V_power_dB if v2v_flag == 1 else self.V2U_power_dB
                        X2I_Interference[k, i] += 10 ** ((interference_power_db - self.V2IChannel_with_fastfading[j, k, i]) / 10)
            
        # 2.2 X2V Interference
        for i in range(self.n_RB):
            # V2I, V2V, V2U, U2U, U2I
            for j in range(self.n_Veh):
                # 检查V2I, V2V, V2U，如果signal>0，再计算发射端到接收端的距离
                v2i_flag = 1 if np.max(V2I_Signal[j, :, i])>0 else 0
                v2v_flag = 1 if np.max(V2V_Signal[j, :, i])>0 else 0
                
                assert v2i_flag + v2v_flag <= 1
                if v2i_flag + v2v_flag == 0: continue
                for k in range(self.n_Veh):
                    if j == k: continue # 自己发信号不造成干扰
                    interference_power_db = self.V2I_power_dB
                    if v2i_flag != 0:
                        interference_power_db = self.V2V_power_dB if v2v_flag == 1 else self.V2U_power_dB
                    X2V_Interference[k, i] += 10 ** ((interference_power_db - self.V2VChannel_with_fastfading[j, k, i]) / 10)
                    if X2V_Interference[k, i] > 2000:
                        print('什么玩意')
            
        
        # 3. 最后再计算rate
        # 3.1 V2I rate
        V2I_Interference = np.zeros((self.n_Veh, self.n_BS, self.n_RB))
        # 通过X2I的功率减去i-th veh对j-th BS的V2I的功率
        for i in range(self.n_Veh):
            for j in range(self.v_neighbor_BS):
                for k in range(self.n_RB):
                    BS_id = self.vehicles[i].V2I_neighbors[j]
                    # if not self.V2I_active_links[i, BS_id]:
                    #     V2I_Interference[i, BS_id, k] = X2I_Interference[BS_id, k]
                    #     continue
                    V2I_Interference[i, BS_id, k] = X2V_Interference[i, k]
                    #  - 10 ** ((self.V2I_power_dB - self.V2IChannel_with_fastfading[i, BS_id, k] ) / 10)
        self.V2I_Interference = V2I_Interference + self.sig2
        # SINR计算
        self.V2I_Rate = np.log2(1 + np.divide(V2I_Signal, self.V2I_Interference))
        
        # 3.2 V2V rate
        V2V_Interference = np.zeros((self.n_Veh, self.n_Veh, self.n_RB))
        # 通过X2V的功率减去i-th veh对j-th veh的V2V的功率
        for i in range(self.n_Veh):
            for j in range(self.v_neighbor_Veh):
                for k in range(self.n_RB):
                    veh_id = self.vehicles[i].get_V2V_neighbor(j)
                    if veh_id is None:
                        continue
                    if not self.V2V_active_links[i, veh_id]:
                        continue
                    V2V_Interference[i, veh_id, k] = X2V_Interference[veh_id, k] 
                    # - 10 ** ((self.V2V_power_dB - self.V2VChannel_with_fastfading[i, veh_id, k] ) / 10)
        self.V2V_Interference = V2V_Interference + self.sig2
        # SINR计算
        self.V2V_Rate = np.log2(1 + np.divide(V2V_Signal, self.V2V_Interference))
        
        
    # 进行任务的传输和任务的执行
    def Execute_Compute_and_Communicate(self):
        '''需要先把offload的决策确定下来，基于offload的决策确定active link, 进行传输和所有单元的计算'''
        # V2U, V2I, V2V任务卸载，传输时延
        for i in range(self.n_Veh):
            veh_i = self.vehicles[i]
            veh_i:Vehicle
            if len(veh_i.off_tasks) > 0:
                mode, task, device_idx, RB_no = veh_i.get_transmit_task_info()
                assert mode in ['V2V','V2I','V2U']
                tran_latency = 0
                V2X_Rate = None
                if mode == 'V2V':
                    trans_data = self.V2V_Rate[i, device_idx, RB_no] * self.time_step
                    V2X_Rate = self.V2V_Rate
                    obj_dev = self.vehicles[device_idx]
                    if not obj_dev.is_running:
                         continue
                    active_links = self.V2V_active_links
                elif mode == 'V2I':
                    trans_data = self.V2I_Rate[i, device_idx, RB_no] * self.time_step
                    V2X_Rate = self.V2I_Rate
                    obj_dev = self.BSs[device_idx]
                    active_links = self.V2I_active_links


                if not active_links[i, device_idx]:
                    # 如果没法连接（对方车辆离开了, 之前的连接对不上）
                    veh_i.finish_transmit_task()
                    self.failed_transmit_tasks.append(task)
                    continue

                if trans_data > task.data_size: # 在当前时间片内传输成功
                    tran_latency = trans_data / V2X_Rate[i, device_idx, RB_no]
                    task.data_size = 0
                    estimate_remain_latency = 0
                else:
                    tran_latency = self.time_step
                    task.data_size -= trans_data # 减去已经传播的一部分
                    estimate_remain_latency = task.data_size / (0.001 + V2X_Rate[i, device_idx, RB_no]) # 防止出现除以0错误

                task.trans_delay += tran_latency
                if estimate_remain_latency == 0:
                    # 对象新增一个任务
                    obj_dev.append_task(task) # BS也要增加任务
                    # 自身减少一个任务
                    veh_i.finish_transmit_task()
                    active_links[i, device_idx] = False # 完成传输，关闭链路
                else:
                    veh_i.update_transmit_task_delay(estimate_remain_latency)

        finished_tasks = []
        # veh 计算时延
        for i in range(self.n_Veh):
            # 假设平均分配计算资源，直接加在计算任务的时延上面
            veh_i = self.vehicles[i]
            veh_i:Vehicle
            if not veh_i.is_running: continue
            tmp = veh_i.calculate_task(self.time_step, mode='avg')
            finished_tasks.extend(tmp)
        '''
        for i in range(self.n_BS):
            bs_i = self.UAVs[i]
            tmp = bs_i.calculate_task(self.time_step, mode='avg')
            finished_tasks.extend(tmp)
        '''

        # 计算任务是完成了还是失败了
        for task in finished_tasks:
            if task.ddl >= task.service_delay:
                self.succeed_tasks.append(task)
            else:
                self.failed_tasks.append(task)
    # 更新大尺度衰落
    def update_large_fading(self, veh_positions):
        # 涉及到车辆和无人机的channel需要更新位置
        self.V2IChannel.update_positions(veh_positions)
        self.V2VChannel.update_positions(veh_positions)
        
        # 更新path loss
        self.V2IChannel.update_pathloss()
        self.V2VChannel.update_pathloss()
        
        # 计算距离差
        veh_delta_distance = self.time_step * np.asarray([c.velocity if c.is_running else 0 for c in self.vehicles])
        
        # 更新阴影
        self.V2IChannel.update_shadow(veh_delta_distance)
        self.V2VChannel.update_shadow(veh_delta_distance)
        
    # 更新小尺度衰落
    def update_small_fading(self):
        self.V2IChannel.update_fast_fading()
        self.V2VChannel.update_fast_fading()

    # 更新信道   
    def renew_channel(self):
        # ====================================================================================
        # This function updates all the channels including V2V, V2I, V2U, U2U, U2I channels
        # ====================================================================================
        veh_positions = [c.position for c in self.vehicles]
        
        self.update_large_fading(veh_positions)
        self.V2VChannel_abs = self.V2VChannel.PathLoss + self.V2VChannel.Shadow
        self.V2IChannel_abs = self.V2IChannel.PathLoss + self.V2IChannel.Shadow

    # 更新信道快速衰落 
    def renew_channels_fastfading(self):   
        # ====================================================================================
        # This function updates all the channels including V2V, V2I, V2U, U2U, U2I channels
        # ====================================================================================
        self.renew_channel() # large fadings
        self.update_small_fading()
        # 为什么要减去快速衰落?
        V2VChannel_with_fastfading = np.repeat(self.V2VChannel_abs[:, :, np.newaxis], self.n_RB, axis=2)
        self.V2VChannel_with_fastfading = V2VChannel_with_fastfading - self.V2VChannel.FastFading
        V2IChannel_with_fastfading = np.repeat(self.V2IChannel_abs[:, :, np.newaxis], self.n_RB, axis=2)
        self.V2IChannel_with_fastfading = V2IChannel_with_fastfading - self.V2IChannel.FastFading

    def update_channel_vNum(self):
        self.V2VChannel.update_Vnum(self.n_Veh)  # number of vehicles
        self.V2IChannel.update_VNum(self.n_Veh)

    # 更新邻居
    def renew_neighbor(self):   
        # ==========================================
        # update the neighbors of vehicles.
        # ===========================================
        # -----V2V neighbor-----------
        z = np.array([[complex(c.position[0],c.position[1]) for c in self.vehicles]])
        z3 = np.array([[complex(c.position[0],c.position[1]) for c in self.BSs]])
        # V2V邻居
        Distance = abs(z.T-z)
        sort_idx_v2v = np.argsort(Distance, axis = 1)
        # V2I邻居
        Distance = abs(z.T-z3)
        sort_idx_v2i = np.argsort(Distance, axis = 1)
        
        for i in range(len(self.vehicles)):
            # -----V2V neighbor-----------
            self.vehicles[i].V2V_neighbors = sort_idx_v2v[i, 1:self.v_neighbor_Veh+1]
            self.vehicles[i].V2V_destinations = self.vehicles[i].V2V_neighbors
            
            # -----V2I neighbor-----------  
            self.vehicles[i].V2I_neighbors = sort_idx_v2i[i, 0:self.v_neighbor_BS]
            self.vehicles[i].V2I_destinations = self.vehicles[i].V2I_neighbors

    def Compute_Statistics(self):
        # -- compute the statistics --
        success_num = len(self.succeed_tasks)
        fail_num = len(self.failed_tasks)
        completion_ratio = 0 if success_num == 0 else success_num / (success_num + fail_num)

        avg_latency = 0
        for task in self.succeed_tasks + self.failed_tasks:
            avg_latency += task.service_delay
        avg_latency = 0 if success_num == 0 else avg_latency / (success_num + fail_num)

        trans_completion_ration = 1 - len(self.failed_transmit_tasks) / (len(self.failed_transmit_tasks) + success_num + fail_num)
        return avg_latency, completion_ratio, trans_completion_ration
        #系统平均延时，任务完成率，传输完成率
    
    def act(self,actions):

        v2v_offload = actions[0]
        v2i_offload = actions[1]
        # RB选择
        v2v_tran = actions[2]  # n_Veh x v_neighbor_Veh: [0, n_RB)
        v2i_tran = actions[3]
        # 1 任务卸载决策, active links
        self.offload_tasks(v2v_offload,v2i_offload,v2v_tran,v2i_tran)
        # 2 信道信息计算
        self.Compute_Rate(v2v_tran, v2i_tran)
        # 3 进行传输，计算时延
        self.Execute_Compute_and_Communicate()
        # 4 统计结果
        # 5 车辆位置移动，无人机位置移动
        self.renew_veh_positions()
        self.update_activated_veh()
        
        self.renew_neighbor()
        self.renew_channels_fastfading()

        self.generate_tasks()
        #reward = self.getReward() # 奖励函数还没加
        self.n_step += 1
        self.cur_time += self.time_step
        self.cur_time = round(self.cur_time, 2)
        print("当前时间" + str(self.cur_time))
        #return reward




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    env = Environment()
    while env.n_step < 10:
        actions = [None for _ in range(12)]
        #v2v_offload
        actions[0] = -1 * np.zeros((env.n_Veh,env.v_neighbor_Veh), dtype='int')
        #v2i_offload
        actions[1] = -1 * np.ones((env.n_Veh, env.v_neighbor_BS), dtype='int')
        # V2V通信使用的资源块数
        actions[2] = np.random.randint(low=0, high=env.n_RB, size=(env.n_Veh,env.v_neighbor_Veh))
        # V2I通信使用的资源块数
        actions[3] = np.random.randint(low=0, high=env.n_RB, size=(env.n_Veh,env.v_neighbor_BS))
        # I2I通信使用的资源块数
        actions[4] = np.random.randint(low=0, high=env.n_RB, size=(env.n_BS,env.B_neighbor_BS))
        
        env.act(actions)


    print(env.Compute_Statistics())
