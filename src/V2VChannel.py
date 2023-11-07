import numpy as np
import math
class V2VChannel:
    def __init__(self, n_Veh, n_RB):
        '''RB数量不变，车辆数量会变，参数设置来源3GPP TR36.885-A.1.4-1'''
        self.t = 0
        self.h_bs = 1.5 # 车作为BS的高度
        self.h_ms = 1.5 # 车作为MS的高度
        self.fc = 2 # carrier frequency
        self.decorrelation_distance = 10
        self.shadow_std = 3 # shadow的标准值
        self.n_Veh = n_Veh # 车辆数量
        self.n_RB = n_RB # RB数量
        self.Shadow = np.random.normal(0, self.shadow_std, size=(self.n_Veh, self.n_Veh))
        self.update_shadow([])

    def update_Vnum(self, new_n_Veh):
        self.n_Veh = new_n_Veh

    def update_positions(self, positions):
        self.positions = positions
    def update_pathloss(self):
        self.PathLoss = np.zeros(shape=(len(self.positions),len(self.positions)))
        for i in range(len(self.positions)):
            for j in range(len(self.positions)):
                if i == j: continue
                self.PathLoss[i][j] = self.get_path_loss(self.positions[i], self.positions[j])

    def update_shadow(self, delta_distance_list):
        '''输入距离变化，计算阴影变化，基于3GPP的规范'''
        if len(self.Shadow) == 0:
            self.Shadow = np.random.normal(0, self.shadow_std, size=(self.n_Veh, self.n_Veh))
        if len(self.Shadow) != len(delta_distance_list):
            # 1 如果过去一个时间片的车辆数量发生了变化（只会增加）
            Shadow = np.random.normal(0, self.shadow_std, size=(self.n_Veh, self.n_Veh))
            Shadow[0:len(self.Shadow), 0:len(self.Shadow)] = self.Shadow[:, :]
            self.Shadow = Shadow
        delta_distance = np.zeros((len(delta_distance_list), len(delta_distance_list)))
        for i in range(len(delta_distance)):
            for j in range(len(delta_distance)):
                delta_distance[i][j] = delta_distance_list[i] + delta_distance_list[j]
        if len(delta_distance_list) != 0:
            self.Shadow = np.exp(-1*(delta_distance/self.decorrelation_distance)) * self.Shadow + np.sqrt(1 - np.exp(-2*(delta_distance/self.decorrelation_distance))) * np.random.normal(0, self.shadow_std, size = (self.n_Veh, self.n_Veh))

    def update_fast_fading(self):
        '''快衰落，网上开源代码'''
        h = 1/np.sqrt(2) * (np.random.normal(size=(self.n_Veh, self.n_Veh, self.n_RB)) + 1j * np.random.normal(size=(self.n_Veh, self.n_Veh, self.n_RB)))
        self.FastFading = 20 * np.log10(np.abs(h))

    def get_path_loss(self, position_A, position_B):
        '''出自IST-4-027756 WINNER II D1.1.2 V1.2 WINNER II的LoS和NLoS模型'''
        d1 = abs(position_A[0] - position_B[0]) # 单位是km
        d2 = abs(position_A[1] - position_B[1])
        d = math.hypot(d1,d2)+0.001
        d_bp = 4 * (self.h_bs - 1) * (self.h_ms - 1) * self.fc * (10**9)/(3*10**8)
        def PL_Los(d):
            if d <= 3:
                return 22.7 * np.log10(3) + 41 + 20*np.log10(self.fc/5)
            else:
                if d < d_bp:
                    return 22.7 * np.log10(d) + 41 + 20 * np.log10(self.fc/5)
                else:
                    return 40.0 * np.log10(d) + 9.45 - 17.3 * np.log10(self.h_bs) - 17.3 * np.log10(self.h_ms) + 2.7 * np.log10(self.fc/5)
        def PL_NLos(d_a,d_b):
            n_j = max(2.8 - 0.0024*d_b, 1.84)
            return PL_Los(d_a) + 20 - 12.5*n_j + 10 * n_j * np.log10(d_b) + 3*np.log10(self.fc/5)
        if min(d1,d2) < 10: # 以7m作为LoS存在的标准
            PL = PL_Los(d)
            self.ifLOS = True
            self.shadow_std = 3
        else:
            PL = min(PL_NLos(d1,d2), PL_NLos(d2,d1))
            self.ifLOS = False
            self.shadow_std = 4                      # if Non line of sight, the std is 4
        return PL