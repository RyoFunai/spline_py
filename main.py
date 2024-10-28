from controller import Main_controller
from animation import Animation_robot
from utils import load_obstacles

def main():
    controller = Main_controller()
    
    # 障害物を読み込む
    obstacles = load_obstacles()
    traj_x, traj_y, traj_th, traj_paths, traj_opt, traj_g_x, traj_g_y, course = controller.run_to_goal(obstacles)

    animation = Animation_robot()
    animation.fig_set()
    animation.maximize_window()
    animation.func_anim_plot(
        traj_x, traj_y, traj_th, traj_paths,
        traj_g_x, traj_g_y, traj_opt, obstacles, course
    )

if __name__ == '__main__':
    main()
