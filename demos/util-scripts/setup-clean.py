import subprocess
import signal
import sys
from time import sleep


def run_command(command):
    """Function to run a shell command."""
    subprocess.run(command, shell=True)

def kill_tmux_session(session_name):
    """Function to kill a tmux session."""
    subprocess.run(['tmux', 'kill-session', '-t', session_name], stderr=subprocess.DEVNULL)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Killing tmux session...')
    kill_tmux_session(session_name)
    sys.exit(0)

# Define the tmux session name
session_name = 'ros_session'

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# Start a new gnome-terminal and run the tmux commands within it
run_command(f'gnome-terminal -- bash -c "tmux new-session -d -s {session_name}; '
            f'tmux split-window -h -t {session_name}; '
            f'tmux split-window -v -t {session_name}:0.0; '
            f'tmux split-window -v -t {session_name}:0.1; '
            f'tmux split-window -v -t {session_name}:0.2; '
            f'tmux split-window -v -t {session_name}:0.3; '
            f'tmux send-keys -t {session_name}:0.0 \'hsr;roslaunch suturo_bringup robocup_bringup.launch\' C-m; '
            f'tmux send-keys -t {session_name}:0.1 \'hsr;roslaunch giskardpy giskardpy_hsr_real_vel.launch\' C-m; '
            f'tmux send-keys -t {session_name}:0.2 \'hsr;robokudo_venv; rosrun robokudo main.py _ae=clean_table _ros_pkg=milestone1\' C-m; '
            f'tmux send-keys -t {session_name}:0.3 \'sshpass -p hsrhmi ssh -t hsr-hmi@hsrb.local\' C-m; '
            f'tmux send-keys -t {session_name}:0.3 \'wait && ./launch_display.sh\' C-m; '

    

            f'tmux attach-session -t {session_name}"')

# Keep the script running to handle signals
while True:
    signal.pause()
