import numpy as np
import time
from robot import Two_wheeled_robot
from goal import Const_goal
from dwa import DWA
from config import DT, GOAL_THRESHOLD, LOOKAHEAD_DISTANCE
from animation import Animation_robot
from obstacle import Obstacle

class Main_controller():  # Main control class
    def __init__(self):
        self.robot = Two_wheeled_robot()
        self.goal_maker = Const_goal()
        self.controller = DWA()
        self.samplingtime = DT
        self.animation = Animation_robot()
        self.obstacles = []
        self._load_course_obstacles()

    def _load_course_obstacles(self):
        # Load course boundaries as obstacles
        for lane in [self.controller.course.left_lane, self.controller.course.right_lane, self.controller.course.center_lane]:
            for point in lane:
                x, y = point[:2]
                self.obstacles.append(Obstacle(x, y, 0.25))  # Size can be adjusted as needed
        print(f"Loaded {len(self.obstacles)} course obstacles.")

    def run_to_goal(self):
        goal_flag = False
        time_step = 0

        self.animation.initialize_animation(self.controller.course)

        while not goal_flag:
            start_time = time.time()

            # Input calculation with obstacles
            paths, opt_path = self.controller.calc_input(self.robot, self.obstacles)

            # Update robot state
            self.robot.update_state(opt_path.u_th, opt_path.u_v, self.samplingtime)

            # Get next target point
            g_x, g_y = self.controller.course.get_next_target_point(
                self.robot.x, self.robot.y, self.robot.th, LOOKAHEAD_DISTANCE
            )

            print(f"Next target point: ({g_x}, {g_y})")
            dis_to_goal = np.sqrt((g_x - self.robot.x)**2 + (g_y - self.robot.y)**2)
            if dis_to_goal < GOAL_THRESHOLD:
                goal_flag = True

            # Update animation frame
            self.animation.update_frame(self.robot, paths, opt_path, g_x, g_y, time_step)

            end_time = time.time()
            cycle_time = end_time - start_time
            print(f"Cycle {time_step} processing time: {cycle_time:.4f} seconds")

            time_step += 1

        return (
            self.robot.traj_x,
            self.robot.traj_y,
            self.robot.traj_th,
            self.controller.traj_paths,
            self.controller.traj_opt,
            self.controller.traj_g_x,
            self.controller.traj_g_y,
            self.controller.course
        )
