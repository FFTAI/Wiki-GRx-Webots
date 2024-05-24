from robot_rcs_gr.robot.gr1t2_simple.fi_webots_gr1t2_simple import WebotsGR1T2Simple

print("WebotsGR1T1Simple controller is ready to work.")

sim_dt = 1  # ms

webots_robot = WebotsGR1T2Simple(sim_dt=sim_dt)
webots_robot.prepare()
webots_robot.enable()

webots_robot.before_control_loop()

while (webots_robot.webots_robot_intf.step(sim_dt) != -1):
    webots_robot.control_loop_update_robot_state()
    webots_robot.control_loop_algorithm()
    webots_robot.control_loop_output()
