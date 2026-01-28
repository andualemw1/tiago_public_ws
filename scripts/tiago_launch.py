#!/usr/bin/env python3

# python3 tiago_launch.py

#!/usr/bin/env python3

import subprocess
import os
import signal

def main():
    # Full launch command as a string
    tiago_cmd = (
        "ros2 launch tiago_gazebo tiago_gazebo.launch.py "
        "world_name:=empty "
        "is_public_sim:=True "
        "arm_type:=tiago-arm "
        "end_effector:=pal-hey5"
    )

    print("[LAUNCH] Starting TIAGo simulation in Gazebo...")
    # Use shell=True so ROS2 launch works
    tiago_proc = subprocess.Popen(tiago_cmd, shell=True, preexec_fn=os.setsid)

    print("[INFO] TIAGo launched. Press CTRL+C to stop.")
    try:
        tiago_proc.wait()
    except KeyboardInterrupt:
        print("[SHUTDOWN] Terminating TIAGo...")
        os.killpg(os.getpgid(tiago_proc.pid), signal.SIGTERM)

if __name__ == "__main__":
    main()
