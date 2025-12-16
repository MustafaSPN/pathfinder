#!/bin/bash

# --- AYARLAR ---
SESSION="rover_mission"
CONTAINER="ros2_humble"
DELAY=5  # Bekleme süresi
# ---------------

# Eski session varsa temizle
tmux kill-session -t $SESSION 2>/dev/null

echo "Rover Sistemleri Başlatılıyor..."

# ---------------------------------------------------------
# 1. PENCERE (SOL ÜST): Micro-ROS
# ---------------------------------------------------------
# Session oluştur, ilk pencere zaten aktif
tmux new-session -d -s $SESSION
tmux rename-window -t $SESSION:0 'Rover Mission'

# Aktif pencereye (Sol Üst) yaz
tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
tmux send-keys "cd ~/microros_ws/ && source install/setup.bash" C-m
tmux send-keys "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 115200" C-m


# ---------------------------------------------------------
# 2. PENCERE (SAĞ ÜST): Rover Bringup
# ---------------------------------------------------------
# Ekranı YATAY böl (Sağ taraf oluşur ve odak oraya geçer)
tmux split-window -h 

# Artık aktif pencere Sağ Üst
tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
tmux send-keys "cd ~/ros2_ws/ && source install/setup.bash" C-m
tmux send-keys "ros2 launch rover_bringup rover.launch.py" C-m


# ---------------------------------------------------------
# 3. PENCERE (SOL ALT): Navigation
# ---------------------------------------------------------
# Sol Üstteki pencereyi seç (Pane 0 her zaman ilk açılandır)
tmux select-pane -t 0
# DİKEY böl (Altına yeni pencere açılır ve odak oraya geçer)
tmux split-window -v

# Artık aktif pencere Sol Alt
tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
tmux send-keys "cd ~/ros2_ws/ && source install/setup.bash" C-m
tmux send-keys "ros2 launch rover_bringup navigation.launch.py" C-m


# ---------------------------------------------------------
# 4. PENCERE (SAĞ ALT): Web Backend
# ---------------------------------------------------------
# Sağ Üstteki pencereyi bulmak için önce Sol Üst'e git, sonra Sağa git
tmux select-pane -t 0
tmux select-pane -R  # Sağa git (Sağ Üsttesin)
# DİKEY böl (Altına yeni pencere açılır ve odak oraya geçer)
tmux split-window -v

# Artık aktif pencere Sağ Alt
tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
tmux send-keys "cd ~/ros2_ws/ && source install/setup.bash" C-m
tmux send-keys "cd apps/pathfinder_webapp/backend/" C-m
tmux send-keys "python3 -m uvicorn app:app --host 0.0.0.0 --port 8000" C-m


# ---------------------------------------------------------
# BİTİRİŞ
# ---------------------------------------------------------
# Görünümü eşitle
tmux select-layout tiled
tmux set -g mouse on

# İmleci Sol Üst'e getir ve bağlan
tmux select-pane -t 0
tmux attach-session -t $SESSION