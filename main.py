from controller import Main_controller
from animation import Animation_robot

def main():
    controller = Main_controller()
    traj_x, traj_y, traj_th, traj_paths, traj_opt, traj_g_x, traj_g_y, course = controller.run_to_goal()

    animation = Animation_robot()
    animation.fig_set()
    animation.maximize_window()
    animation.func_anim_plot(traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y, traj_opt, [], course)

if __name__ == '__main__':
    main()
