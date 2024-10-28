import math
import numpy as np
from utils import min_max_normalize, angle_range_corrector
from obstacle import Obstacle
from config import *
from course import Course
import sys
from config import LOOKAHEAD_DISTANCE
import time

class Simulator_DWA_robot:
    def __init__(self):
        self.max_accelation = MAX_ACCEL
        self.max_ang_accelation = MAX_DYAWRATE
        self.lim_max_velo = MAX_SPEED
        self.lim_min_velo = MIN_SPEED
        self.lim_max_ang_velo = MAX_YAWRATE
        self.lim_min_ang_velo = MIN_YAWRATE

    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step):
        next_xs = []
        next_ys = []
        next_ths = []

        for _ in range(pre_step):
            temp_x = velo * math.cos(th) * dt + x
            temp_y = velo * math.sin(th) * dt + y
            temp_th = ang_velo * dt + th

            next_xs.append(temp_x)
            next_ys.append(temp_y)
            next_ths.append(temp_th)

            x = temp_x
            y = temp_y
            th = temp_th

        return next_xs, next_ys, next_ths

class Path:
    def __init__(self, u_th, u_v):
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th


class DWA():
    def __init__(self):
        self.simu_robot = Simulator_DWA_robot()
        self.pre_time = PREDICT_TIME
        self.pre_step = int(self.pre_time / DT)
        self.delta_velo = V_RESOLUTION
        self.delta_ang_velo = YAWRATE_RESOLUTION
        self.samplingtime = DT
        self.weight_angle = WEIGHT_ANGLE
        self.weight_velo = WEIGHT_VELOCITY
        self.weight_obs = WEIGHT_OBSTACLE
        self.weight_distance = WEIGHT_DISTANCE
        try:
            self.course = Course(LEFT_LANE_BOUND_FILE, RIGHT_LANE_BOUND_FILE, CENTER_LANE_LINE_FILE)
        except FileNotFoundError as e:
            print(f"ラー: {e}")
            print("CSVファイルのパスが正しいか確認してください。")
            sys.exit(1)

        # すべてのPathを保存
        self.traj_paths = []
        self.traj_opt = []

    def calc_input(self, robot, obstacles):
        print("obstacles = ", obstacles)
        paths = self._generate_paths(robot)
        target_point = self._get_target_point(robot)
        opt_path = self._evaluate_paths(paths, target_point, robot, obstacles)
        self._update_trajectory(opt_path, target_point)
        return paths, opt_path

    def _generate_paths(self, robot):
        paths = self._make_path(robot)
        print(f"Number of paths generated: {len(paths)}")
        return paths

    def _get_target_point(self, robot):
        g_x, g_y = self.course.get_next_target_point(robot.x, robot.y, robot.th, LOOKAHEAD_DISTANCE)
        print(f"Next target point: ({g_x}, {g_y})")
        return g_x, g_y

    def _update_trajectory(self, opt_path, target_point):
        self.traj_opt.append(opt_path)
        if not hasattr(self, 'traj_g_x'):
            self.traj_g_x = []
            self.traj_g_y = []
        self.traj_g_x.append(target_point[0])
        self.traj_g_y.append(target_point[1])

    def _make_path(self, state): 
        # 角度と速度の範囲算出
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)

        # 全てのpathのリスト
        paths = []

        # 角速度と速度の組み合わせを全探索
        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):

                path = Path(ang_velo, velo)

                next_x, next_y, next_th \
                    = self.simu_robot.predict_state(ang_velo, velo, state.x, state.y, state.th, self.samplingtime, self.pre_step)

                path.x = next_x
                path.y = next_y
                path.th = next_th

                # 作ったpathを追加
                paths.append(path)

        # 時刻歴Pathを保存
        self.traj_paths.append(paths)

        return paths

    def _calc_range_velos(self, state): # 角速度と角度の範囲決定①
        # 角速度
        range_ang_velo = self.samplingtime * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo
        # 最小値
        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo
        # 最大値
        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        # 速度
        range_velo = self.samplingtime * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo
        # 最小値
        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo
        # 最大値
        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo

    def _evaluate_paths(self, paths, target_point, state, obstacles):
        valid_paths = []
        for path in paths:
            scores = self._calculate_path_scores(path, target_point, obstacles)
            # print(f"Path scores: {scores}")  # スコアの確認
            if all(np.isfinite(score) for score in scores):
                valid_paths.append((path, scores))

        if not valid_paths:
            raise ValueError("No valid paths found. All paths are either out of bounds or colliding with obstacles.")

        print(f"Number of valid paths: {len(valid_paths)}")
        normalized_scores = self._normalize_scores(valid_paths, target_point)
        opt_path = self._select_optimal_path(valid_paths, normalized_scores)

        return opt_path

    def _calculate_path_scores(self, path, target_point, obstacles):
        angle_score = self._heading_angle(path, *target_point)
        velo_score = self._heading_velo(path)
        obs_score = self._obstacle(path, obstacles)
        return angle_score, velo_score, obs_score

    def _normalize_scores(self, valid_paths, target_point):
        angle_scores, velo_scores, obs_scores = zip(*[scores for _, scores in valid_paths])
        
        normalized_scores = {
            'angle': min_max_normalize(angle_scores),
            'velo': min_max_normalize(velo_scores),
            'obs': min_max_normalize(obs_scores),
            'distance': self._calculate_distance_scores(valid_paths, target_point)
        }
        
        return normalized_scores

    def _calculate_distance_scores(self, valid_paths, target_point):
        distances = [-math.hypot(target_point[0] - path.x[-1], target_point[1] - path.y[-1]) for path, _ in valid_paths]
        return min_max_normalize(distances)

    def _select_optimal_path(self, valid_paths, normalized_scores):
        best_score = -float('inf')
        opt_path = None

        for i, (path, _) in enumerate(valid_paths):
            score = (self.weight_angle * normalized_scores['angle'][i] +
                     self.weight_velo * normalized_scores['velo'][i] +
                     self.weight_obs * normalized_scores['obs'][i] +
                     self.weight_distance * normalized_scores['distance'][i])
            
            if score > best_score:
                best_score = score
                opt_path = path

        return opt_path

    def _heading_angle(self, path, g_x, g_y): # ゴールに向いているか
        # 終端の向き
        last_x = path.x[-1]
        last_y = path.y[-1]
        last_th = path.th[-1]

        # 角度計算
        angle_to_goal = math.atan2(g_y-last_y, g_x-last_x)

        # score計算
        score_angle = angle_to_goal - last_th

        # ぐるぐる防止
        score_angle = abs(angle_range_corrector(score_angle))

        # 最大と最小をひっくり返す
        score_angle = math.pi - score_angle

        # print('score_sngle = {0}' .format(score_angle))

        return score_angle

    def _heading_velo(self, path): # 速くんでいるか（直進）

        score_heading_velo = path.u_v

        return score_heading_velo

    def _calc_nearest_obs(self, state, obstacles):
        area_dis_to_obs = 1 # パラメー（何メートル考慮するか，本当は制動距離）
        nearest_obs = [] # あるエリアに入ってる障害物

        for obs in obstacles:
            temp_dis_to_obs = math.sqrt((state.x - obs.x) ** 2 + (state.y - obs.y) ** 2)

            if temp_dis_to_obs < area_dis_to_obs :
                nearest_obs.append(obs)

        return nearest_obs


    def _obstacle(self, path, obstacles):
        min_distance = float('inf')
        
        for x, y in zip(path.x, path.y):
            # Check existing obstacles
            for obs in obstacles:
                distance = math.hypot(x - obs.x, y - obs.y)
                if distance < obs.size:
                    return -float('inf')
                min_distance = min(min_distance, distance)
            
            # Check course boundary
            boundary_distance = self.course.distance_to_course_boundary(x, y)
            min_distance = min(min_distance, boundary_distance)
        
        return min_distance












