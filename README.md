# mujoco_pendulum

ROS 2 Humble uzerinde MuJoCo + ros2_control entegrasyonu.

Bu repo iki kullanim akisini destekler:
- `pendulum`: tek eksen test ortami (hizli kontrol denemesi)
- `orion5`: 6 eksen robot (gravity/hold tuning)

## Neler Var?

- `src/mujoco_system.cpp`: MuJoCo `hardware_interface::SystemInterface` bridge
- `src/computed_torque_node.cpp`: tek eksen computed torque node
- `src/gravity_comp_relay_node.cpp`: 6 eksen gravity/hold relay node
- `src/effort_test_node.cpp`: sabit/step torque test node
- `mujoco/pendulum.xml`: tek eksen MuJoCo modeli
- `mujoco/orion5.xml`: 6 eksen MuJoCo modeli
- `config/controllers.yaml`: pendulum controller ayarlari
- `config/orion5_controllers.yaml`: orion5 controller ayarlari
- `config/tuning/*.yaml`: soft/medium/stiff tuning presetleri
- `launch/sim.launch.py`: pendulum simulasyon
- `launch/orion5_mujoco.launch.py`: orion5 simulasyon + ros2_control
- `launch/orion5_gravity_comp.launch.py`: orion5 + gravity_comp_relay_node

## Gereksinimler

### 1) Sistem

- Ubuntu 22.04
- ROS 2 Humble

### 2) ROS paketleri

```bash
sudo apt update
sudo apt install -y \
  ros-humble-mujoco-vendor \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-forward-command-controller \
  ros-humble-hardware-interface \
  ros-humble-pluginlib \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro
```

### 3) MuJoCo vendor kontrolu

`mujoco_vendor` binary paketi yuklendikten sonra su dosya bulunmali:

Bu pakette CMake dogrudan su dosyalari linkliyor:
- `/opt/ros/humble/opt/mujoco_vendor/include`
- `/opt/ros/humble/opt/mujoco_vendor/lib/libmujoco.so`

Kontrol:

```bash
ls /opt/ros/humble/opt/mujoco_vendor/lib/libmujoco.so
```

## Derleme

```bash
cd ~/orion_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_pendulum
source install/setup.bash
```

## Hizli Baslangic

### A) Orion5 (onerilen)

```bash
ros2 launch mujoco_pendulum orion5_gravity_comp.launch.py
```

Beklenen:
- `joint_state_broadcaster` aktif
- `effort_controller` aktif
- `gravity_comp_relay_node` aktif

Kontrol:

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /effort_controller/commands
```

### B) Pendulum (tek eksen)

```bash
ros2 launch mujoco_pendulum sim.launch.py
```

Manuel tork testi:

```bash
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [10.0]}" -r 100
```

## Parametre ve Tuning

`orion5_gravity_comp.launch.py` varsayilan olarak `home_hold_medium.yaml` yukler.

Calisirken parametre degistirme:

```bash
ros2 param set /gravity_comp_relay_node q_ref "[0.0,0.0,-1.57,0.0,-1.57,0.0]"
ros2 param set /gravity_comp_relay_node kp "[0.0,0.0,18.0,0.0,0.0,0.0]"
ros2 param set /gravity_comp_relay_node kd "[0.0,0.0,2.5,0.0,0.0,0.0]"
ros2 param set /gravity_comp_relay_node gain_vector "[1.0,1.0,1.05,1.0,1.0,1.0]"
```

## SIk Karsilasilan Sorunlar (Bizim Yasadiklarimiz)

1. Eksen cok kucuk acida kilitleniyor (ornek ~0.055 rad)
- Genelde joint `range` ve aci birimi karisiyor.
- MuJoCo tarafinda `angle="radian"` kullaniyoruz.
- Derece bekleyip radyan girilirse beklenmeyen kilitlenme olur.

2. Komut var ama hareket yok
- Topic tipi `std_msgs/msg/Float64MultiArray` olmali.
- Ayni topic'e birden fazla publisher yaziyor olabilir:
  - `ros2 topic info /effort_controller/commands -v`

3. KD artinca robot saliniyor, torklar +/- max'a vuruyor
- D-terimi fazla agresif olabilir.
- `kd` dusur, `d_term_limit` ve `qd_lpf_alpha` parametrelerini kullan.
- `max_torque` saturasyona girerse salinim artar.

4. `ros2 run` import veya paket bulunamadi hatalari
- Dogru workspace'i source ettiginden emin ol:
  - `source /opt/ros/humble/setup.bash`
  - `source ~/orion_humble_ws/install/setup.bash`

5. RViz acilmiyor ama sistem calisiyor
- Headless/uzak ortamda RViz acilmayabilir.
- Controller ve topiclerden sistemi dogrula.

## Model Notu

`mujoco/pendulum.xml` icinde:
- `inertiafromgeom="false"` => dinamikler geometriye gore otomatik hesaplanmaz.
- `<inertial>` ile verilen `mass/inertia` degerleri dogrudan kullanilir.

## Gelistirme Notu

- `orion5_humble_description` ile URDF zinciri ve isimlerin uyumlu olmasi kritik.
- URDF degisirse `mujoco/orion5.xml` tarafini da esle.
