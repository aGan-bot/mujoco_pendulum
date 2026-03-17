# mujoco_pendulum

ROS 2 Humble üzerinde çalışan, MuJoCo tabanlı tek eklemli sarkaç (`hinge`) ve `ros2_control` entegrasyon paketi.

Bu paket ile şunları yapabilirsiniz:
- MuJoCo sistemini `hardware_interface` plugin olarak çalıştırmak
- `forward_command_controller` ile effort (tork) komutu vermek
- `computed_torque_node` ile hedef açı takibi yapmak
- `effort_test_node` ile sabit/step tork testleri yapmak

## İçerik
- `src/mujoco_system.cpp`: MuJoCo + `SystemInterface` köprüsü
- `src/computed_torque_node.cpp`: computed torque kontrolcü node'u
- `src/effort_test_node.cpp`: debug/test için sabit veya step tork yayıcı
- `mujoco/pendulum.xml`: MuJoCo modeli
- `urdf/pendulum.urdf`: ROS tarafı robot tanımı
- `config/controllers.yaml`: `controller_manager` ayarları
- `launch/sim.launch.py`: temel simülasyon başlatma
- `launch/torque_hold_test.launch.py`: sim + effort test node

## Gereksinimler
- Ubuntu + ROS 2 Humble
- MuJoCo (`mujoco_vendor` üzerinden)
- `ros2_control`, `controller_manager`, `joint_state_broadcaster`, `forward_command_controller`

## Derleme
Workspace kökünden çalıştırın:

```bash
cd ~/humble_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_pendulum --symlink-install
source ~/humble_ws/install/setup.bash
```

## Çalıştırma
### 1) Temel simülasyon

```bash
ros2 launch mujoco_pendulum sim.launch.py
```

Not:
- `sim.launch.py` içinde göreli yollar `src/mujoco_pendulum/...` şeklinde kullanıldığı için komutu `~/humble_ws` kökünden çalıştırmanız önerilir.
- Headless ortamda RViz açılmayabilir; kontrol zinciri yine çalışır.

### 2) Controller durumunu kontrol

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Beklenen:
- `joint_state_broadcaster` aktif
- `effort_controller` aktif

### 3) Manuel tork komutu

```bash
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [10.0]}" -r 100
```

Durum gözlemi:

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

## Computed Torque Kullanımı

```bash
ros2 run mujoco_pendulum computed_torque_node
```

Önemli varsayılan parametreler:
- `kp=34.0`
- `kd=12.0`
- `ki=0.35`
- `i_clamp=2.0`
- `mass=1.7`
- `com_length=0.706`
- `max_torque=20.0`
- `control_rate_hz=500.0`

Canlı parametre değişimi örnekleri:

```bash
ros2 param set /computed_torque_node q_ref 1.5
ros2 param set /computed_torque_node disturbance_tau 2.0
ros2 param set /computed_torque_node kp 30.0
ros2 param set /computed_torque_node kd 10.0
ros2 param set /computed_torque_node ki 0.5
```

## Torque Test Aracı
Simülasyon ile birlikte test node'u açar:

```bash
ros2 launch mujoco_pendulum torque_hold_test.launch.py
```

Bu launch içinde `effort_test_node` varsayılan olarak sabit tork yayınlar ve `joint_states` bilgisini loglar.

## Model Notları (`mujoco/pendulum.xml`)
- `<compiler angle="radian" ...>` kullanılır, yani eklem `range` değerleri radyandır.
- Eklem limiti şu an `[-pi, pi]` olarak tanımlıdır.
- Aktüatör limiti `ctrlrange="-20 20"`.
- Dinamikler explicit inertial ile verilir (`inertiafromgeom="false"`).

## Sık Karşılaşılan Problemler
1. `q` çok küçük bir açıda kilitleniyor:
- MuJoCo açı birimi/range kontrol edin (`angle="radian"`).
- `range` değerinin derece gibi yorumlanmadığından emin olun.

2. Komut gidiyor ama hareket yok:
- `/effort_controller/commands` tipi `Float64MultiArray` olmalı.
- Aynı topic'e birden fazla publisher yazıyor olabilir, kontrol edin:
  - `ros2 topic info /effort_controller/commands -v`

3. Takip çok yavaş veya son kısım sürünüyor:
- Önce `mass` ve `com_length` model uyumunu düzeltin.
- Sonra `kp/kd` ayarlarını yapın.
- En sonda küçük `ki` ile kalıcı hatayı kapatın.

## Geliştirme Önerisi
- `sim.launch.py` içindeki path kullanımı `get_package_share_directory` ile paket-share tabanlı hale getirilebilir.
- Parametrelerin YAML dosyasından yüklenmesi bakım kolaylığı sağlar.
