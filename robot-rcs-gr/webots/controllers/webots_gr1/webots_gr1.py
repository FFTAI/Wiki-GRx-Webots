from robot_rcs_gr.robot.gr1.fi_webots_gr1 import WebotsGR1

print("WebotsGR1 controller is ready to work.")

sim_dt = 1  # ms

webots_robot = WebotsGR1(sim_dt=sim_dt)
webots_robot.prepare()
webots_robot.enable()

webots_robot.before_control_loop()

while (webots_robot.webots_robot_intf.step(sim_dt) != -1):
    webots_robot.control_loop_update_robot_state()
    webots_robot.control_loop_algorithm()
    webots_robot.control_loop_output()
