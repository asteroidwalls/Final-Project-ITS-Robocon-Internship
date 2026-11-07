# Simulasi TR — ABU Robocon 2027 "The Pursuit of Mustika Nusantara"

Kemampuan saat ini:

- Kontrol manual lewat keyboard (node buatan sendiri, bukan `teleop_twist_keyboard`) dengan
  mode local dan global.
- Kontrol manual lewat WebUI Foxglove.
- Gerak semi-otonom: robot menerima target posisi lalu bergerak sendiri dengan kontrol PID.

## Instalasi dan build

```bash
mkdir -p ~/simulasi-2027_ws/src
cd ~/simulasi-2027_ws/src
git clone https://github.com/asteroidwalls/Final-Project-ITS-Robocon-Internship itsr_simulasi-2027

cd ~/simulasi-2027_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Menjalankan simulasi

```bash
# Webots + driver + robot_state_publisher + RViz
ros2 launch robocon_2027_webots bringup.launch.py

# bringup + foxglove_bridge sekaligus (file launch/bringup_foxglove.launch.xml)
ros2 launch robocon_2027_webots bringup_foxglove.launch.xml
```

Tunggu sampai robot mendarat di arena sebelum mengirim perintah gerak.

## Kontrol keyboard

Di terminal **lain** (sudah di-`source`):

```bash
ros2 run robocon_2027_webots tr_keyboard_teleop
```

| Tombol | Fungsi |
|---|---|
| `w` / `s` | maju / mundur (+X / −X) |
| `a` / `d` | geser kiri / kanan (+Y / −Y) |
| `q` / `e` | putar kiri / kanan |
| spasi | stop semua |
| `c` | stop rotasi saja |
| `m` | ganti mode local ↔ global (robot berhenti saat ganti mode) |
| `r` / `f` | kecepatan linear naik / turun |
| `t` / `g` | kecepatan angular naik / turun |
| `Ctrl-C` | keluar |

Tombol bersifat *latched*: tekan sekali, robot terus bergerak sampai spasi. Tiap sumbu
independen, sehingga `w` lalu `d` menghasilkan gerak diagonal.

### Mode kontrol

- **LOCAL**: semua tombol relatif ke arah hadap badan robot.
- **GLOBAL**: `w/s/a/d` searah sumbu +X/−X/+Y/−Y frame `odom`, walau robot sedang berputar.
  Node membaca yaw dari `/odom`; sebelum `/odom` pertama diterima, robot tidak bergerak di mode ini.

### Parameter

```bash
ros2 run robocon_2027_webots tr_keyboard_teleop --ros-args -p start_mode:=global
```

| Parameter | Bawaan | Keterangan |
|---|---|---|
| `start_mode` | `local` | `local` atau `global` |
| `linear_speed` | `0.5` | m/s |
| `angular_speed` | `1.0` | rad/s |
| `max_linear_speed` | `1.5` | batas atas saat menaikkan kecepatan |
| `max_angular_speed` | `3.0` | |
| `linear_accel` | `2.0` | m/s², pelunakan perubahan kecepatan |
| `angular_accel` | `6.0` | rad/s² |
| `publish_rate` | `20.0` | Hz; driver menghentikan robot jika 0.5 detik tanpa perintah |
| `cmd_vel_topic`, `odom_topic` | `/cmd_vel`, `/odom` | |

## WebUI Foxglove

1. Jalankan simulasi, lalu bridge di terminal terpisah:

   ```bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml
   ```

2. Buka Foxglove Studio (https://app.foxglove.dev atau aplikasi desktop), pilih
   Open connection → Foxglove WebSocket, isi `ws://localhost:8765`.
3. Tambahkan panel (menu ⋮ pada panel → Split right/down): 3D, Teleop,
   Raw Messages, Plot.
4. Atur panel Teleop:

   | Pengaturan | Teleop 1 (maju & putar) | Teleop 2 (geser, opsional) |
   |---|---|---|
   | Topic | `/cmd_vel` | `/cmd_vel` |
   | Publish rate | `20` | `20` |
   | Stop on release | aktif | aktif |
   | Up / Down | linear x = +0.5 / −0.5 | — |
   | Left / Right | angular z = +1.0 / −1.0 | linear y = +0.5 / −0.5 |
   | Stop | linear x = 0 | linear y = 0 |

## Gerak semi-otonom (target posisi + PID)

Node `tr_goto_pid` (C++, paket `robocon_2027_control`) menerima target posisi dan arah, lalu
menggerakkan robot sendiri ke sana.

```bash
colcon build --packages-select robocon_2027_control
source install/setup.bash
ros2 run robocon_2027_control tr_goto_pid
```

### Memberi target

```bash
# ke (1.0, 0.5) di frame odom, arah 0
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"

# maju 1 m dari posisi sekarang (relatif ke robot)
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}"

# batalkan target
ros2 topic pub --once /goto/cancel std_msgs/msg/Empty "{}"

# pantau status
ros2 topic echo /goto/status
```

## Dokumentasi penggunaan AI

https://drive.google.com/drive/folders/1jITJXPdvMdRcSqp4OBKg6_rN2p4Wf0Fe?usp=sharing