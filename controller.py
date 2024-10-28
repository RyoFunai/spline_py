import numpy as np
import time
from robot import Two_wheeled_robot
from goal import Const_goal
from dwa import DWA
from config import DT, GOAL_THRESHOLD, MAX_ITERATIONS, LOOKAHEAD_DISTANCE
from utils import load_obstacles


class Main_controller():# Mainの制御クラス
    def __init__(self):
        self.robot = Two_wheeled_robot()
        self.goal_maker = Const_goal()
        self.controller = DWA()
        self.samplingtime = DT

    def run_to_goal(self, obstacles):
        goal_flag = False
        time_step = 0

        while not goal_flag and time_step < MAX_ITERATIONS:
            start_time = time.time()
            # Input calculation
            paths, opt_path = self.controller.calc_input(self.robot, obstacles)
            
            # ロボットの状態を更新
            self.robot.update_state(opt_path.u_th, opt_path.u_v, self.samplingtime)
            

            g_x, g_y = self.controller.course.get_next_target_point(self.robot.x, self.robot.y, self.robot.th, LOOKAHEAD_DISTANCE)
            
            print(g_x, g_y)
            dis_to_goal = np.sqrt((g_x-self.robot.x)**2 + (g_y-self.robot.y)**2)
            if dis_to_goal < GOAL_THRESHOLD:
                goal_flag = True

            time_step += 1
            end_time = time.time()
            print("time:", end_time - start_time)
        return self.robot.traj_x, self.robot.traj_y, self.robot.traj_th, \
                self.controller.traj_paths, self.controller.traj_opt, self.controller.traj_g_x, self.controller.traj_g_y, self.controller.course
