import subprocess
joint_names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "grip"]

def angle_write(angle_orders):
    msg = "{"
    i = 0
    for order in angle_orders:
        if order != None:
            if i>0:
                msg += ", "
            msg += f"{joint_names[i]}: {float(order)}"
        i += 1
    msg += "}"
    # print(msg)

    try:
        command = ["ros2", "topic", "pub", "/angle_control", "angle_control_interfaces/msg/AngleControl", msg, "-1"]
        result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(result.stdout.decode('utf-8'))
    except subprocess.CalledProcessError as e:
        print(f"Error occurred: {e.stderr.decode('utf-8')}")

def main():
    angle_write([-20, -20, -20, -20, -20, -20, -20, None])

if __name__ == "__main__":
    main()