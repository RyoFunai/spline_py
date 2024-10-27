import math

# Robot parameters
ROBOT_INIT_X = 5.0
ROBOT_INIT_Y = -5.0
ROBOT_INIT_TH = 2.7

# DWA parameters
MAX_SPEED = 5.0
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
GOAL_THRESHOLD = 0.01
GOAL_INIT_X = 50.0
GOAL_INIT_Y = 50.0

# Animation parameters
ANIMATION_INTERVAL = 100

# Obstacle parameters
NUM_OBSTACLES = 50

# Course parameters
LEFT_LANE_BOUND_FILE = 'csv_files/left_lane_bound.csv'
RIGHT_LANE_BOUND_FILE = 'csv_files/right_lane_bound.csv'
CENTER_LANE_LINE_FILE = 'csv_files/center_lane_line.csv'

# Main parameters
MAX_ITERATIONS = 10

X_MIN, X_MAX = -30, 60
Y_MIN, Y_MAX = -20, 70

# DWA parameters
LOOKAHEAD_DISTANCE = 10.0