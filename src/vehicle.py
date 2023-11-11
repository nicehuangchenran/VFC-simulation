import numpy as np

class Vehicle:
    def __init__(self, id, position, angle, speed, fre, account, score):
        self.id = id
        self.position = position
        self.velocity = speed
        self.direction = angle
        self.CPU_frequency = fre
        self.task_queue=[]    # 生成的任务/要完成的任务列表
        self.off_tasks = []  #要卸载给别人的任务
        self.rec_tasks = []
        self.failed_tasks = []
        self.__activated = True
        self.account = account
        self.score = score
        self.time = 0
        self.V2V_neighbors = []

    def get_V2V_neighbor(self, neighbor_idx):
        if neighbor_idx < len(self.V2V_neighbors):
            return self.V2V_neighbors[neighbor_idx]
        return None

    def finish_it(self):
        # 停车了但是也要计算任务，只是不能进行position renew和传输了
        assert self.is_running
        self.__activated = False
        self.position = [-10000, -10000]  # 不出现在任何的neighbor之中
        self.CPU_frequency = 0

    # 更新车辆位置信息相关
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

    # 车辆任务相关
    @property
    def not_offload(self):
        return len(self.task_queue) == 0
    
    @property
    def wait_to_transmit(self):
        return len(self.off_tasks) > 0

    @property
    def to_device(self):
        return self.off_tasks[0][2]
    #获取需要卸载的任务信息
    def get_transmit_task_info(self):
        '''返回传输模式，任务，设备真实index'''
        task = self.off_tasks[0]
        #print(task[2],task[3])
        return task[0], task[1], task[2], task[3]

    def finish_transmit_task(self):
        self.off_tasks.pop(0)

    def update_transmit_task_delay(self, estimate_delay):
        '''预估还要传输多久的时间'''
        self.off_tasks[0][-1] = estimate_delay
        #off_tasks[0][-1]表示第一个卸载任务还有多久传输完成

    #生成任务
    def generate_task(self, task):
        assert self.is_running
        self.task_queue.append(task)

    def append_task(self, task):
        assert self.is_running
        self.task_queue.append(task)

    # 执行任务
    def calculate_task(self, time_step, mode = 'avg', decisions = None):
        '''直接使用mode进行任务计算，或者通过decisions来指定'''
        assert self.is_running
        if len(self.task_queue) == 0: 
            return []
        finished_tasks = []
        if mode == 'avg':
            avg_r = self.CPU_frequency / len(self.task_queue) #分给每个任务的cpu
            avg_res = np.ones(len(self.task_queue)) * avg_r * time_step #完成的计算量
            for (idx, task) in enumerate(self.task_queue.copy()):
                if task.cpu >= avg_res[idx]:
                    task.cpu -= avg_res[idx]
                    task.comp_delay += time_step
                #  
                else:
                    task.comp_delay += task.cpu / avg_r
                    task.cpu = 0
                    finished_tasks.append(task)
                    self.task_queue.remove(task)
        # 如果已经完成了需要返回完成的任务列表，必须完成了才去算是否超时
        return finished_tasks

    
    def offload_task(self, off_veh, task_idx, mode='V2V', RB = 0):
        '''一次只能卸载一个任务，所以可以直接remove, mode=V2V, V2I, off_veh是真实idx'''
        assert self.is_running
        assert task_idx < len(self.task_queue) and task_idx >= 0
        task = self.task_queue[task_idx]
        self.task_queue.pop(task_idx)
        self.off_tasks.append([mode, task, off_veh, RB, -1]) # -1表示还没开始传输，正数表示还需要多久的时间(s)，0表示完成传输
        task.routing.append(off_veh) # 添加路由
    
