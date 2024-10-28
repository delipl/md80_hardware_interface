import os

# List of motor IDs to set up
motor_ids = [10, 11, 12, 20, 21, 22, 30, 31, 32, 40, 41, 42]
config_path = "quadruped_robot/AK80-9.cfg"


os.system("mkdir -p /home/${USER}/.config/mdtool/mdtool_motors/quadruped_robot")
os.system("cp ../config/AK80-9.cfg /home/${USER}/.config/mdtool/mdtool_motors/quadruped_robot")

# Iterate over motor IDs and run the command for each one
for motor_id in motor_ids:
    command = f"mdtool setup motor {motor_id} {config_path}"
    os.system(command)  # Execute the command in the shell
    print(f"Executed: {command}")
