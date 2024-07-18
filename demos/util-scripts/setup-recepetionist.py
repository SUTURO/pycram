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
            f'tmux split-window -v -t {session_name}:0.4; '
            f'tmux split-window -v -t {session_name}:0.5; '
            f'tmux split-window -v -t {session_name}:0.6; '
            f'tmux split-window -v -t {session_name}:0.7; '
            # Terminal1 0: Rviz map launch 
            # roslaunch suturo_bringup pre_robocup_door_envi.launch
            f'tmux send-keys -t {session_name}:0.0 \'roslaunch suturo_bringup robocup_bringup.launch\' C-m; '
            
            # Terminal1 1: Giskard launch
            f'tmux send-keys -t {session_name}:0.1 \'roslaunch giskardpy giskardpy_hsr_real_vel.launch\' C-m; '
            
            # Terminal1 2: Perception launch
            f'tmux send-keys -t {session_name}:0.2 \'robokudo_venv; rosrun robokudo main.py _ae=receptionist _ros_pkg=milestone1\' C-m; '
           
            # Terminal1 3: HSR Display launch
            f'tmux send-keys -t {session_name}:0.3 \'sshpass -p hsrhmi ssh -t hsr-hmi@192.168.0.102\' C-m; '
            f'tmux send-keys -t {session_name}:0.3 \'wait && ./launch_display.sh\' C-m; '
            
            # Terminal 4: Rasa start and Audio topic check
            f'tmux send-keys -t {session_name}:0.4 \'virtual_nlp\' C-m; '
            f'tmux send-keys -t {session_name}:0.4 \'cd /home/suturo/suturo23_24/nlp_ws/src/Rasa/rasa_new/suturo_rasa \' C-m; '
            f'tmux send-keys -t {session_name}:0.4 \'rasa run --enable-api &\' C-m; '
            f'tmux send-keys -t {session_name}:0.4 \'rostopic hz /audio/audio\' C-m; '
            
            # Terminal1 5: NLP Bell-Detection script start
            f'tmux send-keys -t {session_name}:0.5 \'rosrun sound_detection nlp_sound_detect.py 5 \' C-m; '
            
            # Terminal1 6: NLP HRI-Script start
            f'tmux send-keys -t {session_name}:0.6 \'nlp_venv; cd /home/suturo/suturo23_24/nlp_ws/src/suturo_nlp/activate_language_processing/scripts ; python3 nlp_receptionist.py -hsr\' C-m; '
            
            # Terminal1 7: HSR Audio topic  start
            f'tmux send-keys -t {session_name}:0.7 \'sshpass -p password ssh -t administrator@192.168.0.102\' C-m; '
            f'tmux send-keys -t {session_name}:0.7 \'wait && roslaunch audio_capture capture_wave.launch\' C-m; '

            f'tmux attach-session -t {session_name}"')

# Keep the script running to handle signals
while True:
    signal.pause()
