import math
import numpy as np
from utils import min_max_normalize, angle_range_corrector
from obstacle import Obstacle
from config import *

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

        # すべてのPathを保存
        self.traj_paths = []
        self.traj_opt = []

    def calc_input(self, g_x, g_y, state, obstacles): # stateはロボットクラスでくる
        # Path作成
        paths = self._make_path(state)
        # Path評価
        opt_path = self._eval_path(paths, g_x, g_y, state, obstacles)

        self.traj_opt.append(opt_path)

        return paths, opt_path

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

    def _eval_path(self, paths, g_x, g_y, state, obstacles):
        # 一番近い障害物判定
        nearest_obs = self._calc_nearest_obs(state, obstacles)

        score_heading_angles = []
        score_heading_velos = []
        score_obstacles = []
        valid_paths = []

        # 全てのpathで評価を検索
        for path in paths:
            # (1) heading_angle
            angle_score = self._heading_angle(path, g_x, g_y)
            # (2) heading_velo
            velo_score = self._heading_velo(path)
            # (3) obstacle
            obs_score = self._obstacle(path, nearest_obs)

            # パスが有効かチェック（スコアにinfやnanが含まれていないか）
            if np.isfinite(angle_score) and np.isfinite(velo_score) and np.isfinite(obs_score):
                score_heading_angles.append(angle_score)
                score_heading_velos.append(velo_score)
                score_obstacles.append(obs_score)
                valid_paths.append(path)
            else:
                # 無効なパスはスキップ
                continue

        # 有効なパスが存在しない場合
        if not valid_paths:
            raise ValueError("有効なPathが存在しません。全てのPathが障害物と衝突しています。")

        # 正規化
        score_heading_angles = min_max_normalize(score_heading_angles)
        score_heading_velos = min_max_normalize(score_heading_velos)
        score_obstacles = min_max_normalize(score_obstacles)

        score = -float('inf')  # スコアの初期値を負の無限大に設定
        opt_path = None        # opt_path を初期化

        # 最適なpathを探索
        for k in range(len(valid_paths)):
            temp_score = (self.weight_angle * score_heading_angles[k] + 
                          self.weight_velo * score_heading_velos[k] + 
                          self.weight_obs * score_obstacles[k])

            if temp_score > score:
                opt_path = valid_paths[k]
                score = temp_score

        if opt_path is None:
            # フォールバック: 最初のpathを選択
            if valid_paths:
                opt_path = valid_paths[0]
            else:
                raise ValueError("Pathが生成されていません。")

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

    def _heading_velo(self, path): # 速く進んでいるか（直進）

        score_heading_velo = path.u_v

        return score_heading_velo

    def _calc_nearest_obs(self, state, obstacles):
        area_dis_to_obs = 5 # パラメー（何メートル考慮するか，本当は制動距離）
        nearest_obs = [] # あるエリアに入ってる障害物

        for obs in obstacles:
            temp_dis_to_obs = math.sqrt((state.x - obs.x) ** 2 + (state.y - obs.y) ** 2)

            if temp_dis_to_obs < area_dis_to_obs :
                nearest_obs.append(obs)

        return nearest_obs

    def _obstacle(self, path, nearest_obs):
        # 障害物回避（エリアに入ったらその線は使わない）/ (障害物ともっとも近い距離距離)))
        score_obstacle = 2
        temp_dis_to_obs = 0.0

        for i in range(len(path.x)):
            for obs in nearest_obs: 
                temp_dis_to_obs = math.sqrt((path.x[i] - obs.x) * (path.x[i] - obs.x) + (path.y[i] - obs.y) *  (path.y[i] - obs.y))

                if temp_dis_to_obs < score_obstacle:
                    score_obstacle = temp_dis_to_obs # 一番近いところ

                # そもそも中に入ってる判定
                if temp_dis_to_obs < obs.size:
                    score_obstacle = -float('inf')
                    break

            else:
                continue

            break

        return score_obstacle