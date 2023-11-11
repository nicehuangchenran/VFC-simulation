class Task:
    id_counter = 0
    def __init__(self,vehicle_id,start_time,cycle,size,importance,ddl):
        Task.id_counter += 1
        self.id = Task.id_counter
        self.g_veh= vehicle_id
        self.start_time=start_time
        self.cpu = cycle
        self.data_size = size    # 任务大小
        self.importance = importance   # 任务重要程度
        self.ddl = ddl  # 单位ms,任务截止时间
        
        self.routing = [self.g_veh] #任务卸载的路由:第一位是任务生成车
        self.trans_delay = 0 # mod time_step就是本时间段内的传输消耗
        self.comp_delay = 0 # 计算时延
    
    @property
    def service_delay(self):
        return self.trans_delay + self.comp_delay
    
    
