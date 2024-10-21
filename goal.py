class Const_goal:
    def __init__(self):
        self.traj_g_x = []
        self.traj_g_y = []

    def calc_goal(self, time_step):
        if time_step <= 150:
            g_x = 10.0
            g_y = 10.0
        else:
            g_x = -10.0
            g_y = -10.0

        self.traj_g_x.append(g_x)
        self.traj_g_y.append(g_y)

        return g_x, g_y