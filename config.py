import math

# Robot parameters
ROBOT_INIT_X = -10.0
ROBOT_INIT_Y = -10.0
ROBOT_INIT_TH = 0.0

# DWA parameters
MAX_SPEED = 1.6
MIN_SPEED = 0.0
MAX_YAWRATE = math.pi
MIN_YAWRATE = -math.pi
MAX_ACCEL = 1.0
MAX_DYAWRATE = 100.0 * math.pi / 180.0
V_RESOLUTION = 0.02
YAWRATE_RESOLUTION = 0.02

# Simulator parameters
DT = 0.1  # Main_controllerとdwaで共通のsamplingtime
PREDICT_TIME = 3.0
ROBOT_RADIUS = 0.2

# Cost parameters
WEIGHT_ANGLE = 0.04
WEIGHT_VELOCITY = 0.2
WEIGHT_OBSTACLE = 0.1
WEIGHT_DISTANCE = 1.0  # ゴールへの距離の重み
# Goal parameters
GOAL_THRESHOLD = 0.5

# Animation parameters
ANIMATION_INTERVAL = 100

# Obstacle parameters
NUM_OBSTACLES = 50