import math
from config import ROBOT_INIT_X, ROBOT_INIT_Y, ROBOT_INIT_TH

class Two_wheeled_robot:
    def __init__(self, init_x=ROBOT_INIT_X, init_y=ROBOT_INIT_Y, init_th=ROBOT_INIT_TH):
        print(init_x, init_y, init_th)
        self.x = init_x
        self.y = init_y
        self.th = init_th
        self.u_v = 0.0
        self.u_th = 0.0

        # 時刻歴保存用
        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_th = [init_th]
        self.traj_u_v = [0.0]
        self.traj_u_th = [0.0]

    def update_state(self, u_th, u_v, dt):
        self.u_th = u_th
        self.u_v = u_v

        next_x = self.u_v * math.cos(self.th) * dt + self.x
        next_y = self.u_v * math.sin(self.th) * dt + self.y
        next_th = self.u_th * dt + self.th

        self.traj_x.append(next_x)
        self.traj_y.append(next_y)
        self.traj_th.append(next_th)

        self.x = next_x
        self.y = next_y
        self.th = next_th
        return self.x, self.y, self.th