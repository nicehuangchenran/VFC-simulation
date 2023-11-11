import numpy as np
import math
class V2IChannel: 
    # Simulator of the V2I channels
    def __init__(self, n_Veh, n_BS, n_RB, BS_positions):
        '''V2I只存在于一个BS范围内的V2I channel'''
        self.h_bs = 25 # 基站高度25m
        self.h_ms = 1.5 
        self.Decorrelation_distance = 50        
        self.BS_positions = BS_positions 
        self.shadow_std = 8
        self.n_Veh = n_Veh
        self.n_BS = n_BS
        self.n_RB = n_RB
        self.Shadow = np.random.normal(0, self.shadow_std, size=(self.n_Veh, self.n_BS))
        self.update_shadow([])
    def update_VNum(self, vnum):
        self.n_Veh = vnum
    def update_positions(self, veh_positions):
        self.positions = veh_positions
        
    def update_pathloss(self):
        self.PathLoss = np.zeros(shape=(len(self.positions), len(self.BS_positions)))
        for i in range(len(self.positions)):
            for j in range(len(self.BS_positions)):
                d1 = abs(self.positions[i][0] - self.BS_positions[j][0])
                d2 = abs(self.positions[i][1] - self.BS_positions[j][1])
                distance = math.hypot(d1,d2)+0.001
                self.PathLoss[i][j] = 128.1 + 37.6*np.log10(math.sqrt(distance**2 + (self.h_bs-self.h_ms)**2)/1000) # 根据3GPP，距离的单位是km

    def update_shadow(self, delta_distance_list):
        if len(self.Shadow) != len(delta_distance_list):
            # 1 如果过去一个时间片的车辆数量发生了变化（只会增加）
            Shadow = np.random.normal(0, self.shadow_std, size=(self.n_Veh, self.n_BS))
            Shadow[0:len(self.Shadow),:] = self.Shadow[:, :]
            self.Shadow = Shadow
        if len(delta_distance_list) != 0:
            delta_distance = np.repeat(delta_distance_list[:,np.newaxis], self.n_BS, axis=1)
            self.Shadow = np.exp(-1*(delta_distance/self.Decorrelation_distance))* self.Shadow + np.sqrt(1-np.exp(-2*(delta_distance/self.Decorrelation_distance)))*np.random.normal(0,self.shadow_std, size=(self.n_Veh, self.n_BS))

    def update_fast_fading(self):
        h = 1/np.sqrt(2) * (np.random.normal(size = (self.n_Veh, self.n_BS, self.n_RB)) + 1j* np.random.normal(size = (self.n_Veh, self.n_BS, self.n_RB)))
        self.FastFading = 20 * np.log10(np.abs(h))
