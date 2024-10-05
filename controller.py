import numpy as np
from robot import Two_wheeled_robot
from goal import Const_goal
from dwa import DWA
from obstacle import Obstacle
from config import DT, GOAL_THRESHOLD, NUM_OBSTACLES, ROBOT_INIT_X, ROBOT_INIT_Y


class Main_controller():# Mainの制御クラス
    def __init__(self):
        self.robot = Two_wheeled_robot()
        self.goal_maker = Const_goal()
        self.controller = DWA()

        # Generate random obstacles away from the initial position
        self.obstacles = []
        initial_safe_radius = 5.0  # Safe radius around the initial position
        for _ in range(NUM_OBSTACLES):
            while True:
                x = np.random.uniform(-10, 10)
                y = np.random.uniform(-10, 10)
                size = np.random.uniform(0.1, 0.5)
                
                # Check if the obstacle is far enough from the initial position
                if np.sqrt((x - ROBOT_INIT_X)**2 + (y - ROBOT_INIT_Y)**2) > initial_safe_radius:
                    self.obstacles.append(Obstacle(x, y, size))
                    break

        self.samplingtime = DT

    def run_to_goal(self):
        goal_flag = False
        time_step = 0

        while not goal_flag:
        # for i in range(250):
            g_x, g_y = self.goal_maker.calc_goal(time_step)

            # 入力決定
            paths, opt_path = self.controller.calc_input(g_x, g_y, self.robot, self.obstacles)

            u_th = opt_path.u_th
            u_v = opt_path.u_v

            # 入力で状態更新
            self.robot.update_state(u_th, u_v, self.samplingtime)

            # goal判定
            dis_to_goal = np.sqrt((g_x-self.robot.x)*(g_x-self.robot.x) + (g_y-self.robot.y)*(g_y-self.robot.y))
            if dis_to_goal < GOAL_THRESHOLD:
                goal_flag = True

            time_step += 1

        return self.robot.traj_x, self.robot.traj_y, self.robot.traj_th, \
                self.goal_maker.traj_g_x, self.goal_maker.traj_g_y, self.controller.traj_paths, self.controller.traj_opt, self.obstacles
