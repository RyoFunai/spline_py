import math
import numpy as np

def min_max_normalize(data):

    data = np.array(data, dtype=np.float64)

    # データにinfやnanが含まれていないか確認
    if not np.all(np.isfinite(data)):
        raise ValueError("データに無限大またはNaNが含まれています。")

    max_data = np.max(data)
    min_data = np.min(data)

    if max_data - min_data == 0:
        normalized_data = np.zeros_like(data)
    else:
        normalized_data = (data - min_data) / (max_data - min_data)

    return normalized_data
# 角度補正用
def angle_range_corrector(angle):

    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi

    return angle

# 円を書く
def write_circle(center_x, center_y, angle, circle_size=0.2):#人の大きさは半径15cm
    # 初期化
    circle_x = [] #位置を表す円のx
    circle_y = [] #位置を表す円のy

    steps = 100 #円を書く分解能はこの程度で大丈夫
    for i in range(steps):
        circle_x.append(center_x + circle_size*math.cos(i*2*math.pi/steps))
        circle_y.append(center_y + circle_size*math.sin(i*2*math.pi/steps))

    circle_line_x = [center_x, center_x + math.cos(angle) * circle_size]
    circle_line_y = [center_y, center_y + math.sin(angle) * circle_size]

    return circle_x, circle_y, circle_line_x, circle_line_y