# mujoco_pendulum

ROS 2 Humble uzerinde MuJoCo + ros2_control entegrasyonu.

Bu repo iki kullanim akisini destekler:
- `pendulum`: tek eksen test ortami (hizli kontrol denemesi)
- `orion5`: 6 eksen robot (gravity/hold tuning)

## Neler Var?

- `src/mujoco_system.cpp`: MuJoCo `hardware_interface::SystemInterface` bridge
- `src/computed_torque_node.cpp`: tek eksen computed torque node
- `src/gravity_comp_relay_node.cpp`: 6 eksen gravity/hold relay node
- `src/pinocchio_gravity_node.cpp`: Pinocchio ile `g(q)` hesaplayip MuJoCo bias ile karsilastirma node'u
- `src/pinocchio_ff_hold_node.cpp`: Pinocchio `g(q)` + PD ile kapali-cevrim hold node'u
- `src/effort_test_node.cpp`: sabit/step torque test node
- `mujoco/pendulum.xml`: tek eksen MuJoCo modeli
- `mujoco/orion5.xml`: 6 eksen MuJoCo modeli
- `config/controllers.yaml`: pendulum controller ayarlari
- `config/orion5_controllers.yaml`: orion5 controller ayarlari
- `config/tuning/*.yaml`: soft/medium/stiff tuning presetleri
- `launch/sim.launch.py`: pendulum simulasyon
- `launch/orion5_mujoco.launch.py`: orion5 simulasyon + ros2_control
- `launch/orion5_gravity_comp.launch.py`: orion5 + gravity_comp_relay_node
- `launch/orion5_pinocchio_gravity_check.launch.py`: orion5 + Pinocchio gravity feedforward dogrulama
- `launch/orion5_pinocchio_ff_hold.launch.py`: orion5 + Pinocchio feedforward closed-loop hold

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
ros2 param set /gravity_comp_relay_node q_ref "[0.0,0.0,-1.57,0.0,1.57,0.0]"
ros2 param set /gravity_comp_relay_node kp "[0.0,0.0,18.0,0.0,0.0,0.0]"
ros2 param set /gravity_comp_relay_node kd "[0.0,0.0,2.5,0.0,0.0,0.0]"
ros2 param set /gravity_comp_relay_node gain_vector "[1.0,1.0,1.05,1.0,1.0,1.0]"
```

## Pinocchio Gravity Feedforward Dogrulama

Ilk hedef icin yalnizca gravity feedforward'u dogrulamak icin:

```bash
ros2 launch mujoco_pendulum orion5_pinocchio_gravity_check.launch.py
```

Bu launch ile:
- MuJoCo simulasyon (`orion5_mujoco`) acilir
- `pinocchio_gravity_node` calisir

Node yayinlari:
- `/pinocchio/gravity_torque`: Pinocchio `g(q)`
- `/pinocchio/gravity_error`: `/mujoco/bias_torque - g(q)`
- `/pinocchio/bias_torque`: Pinocchio `rnea(q,qd,0)` (bias benzeri)
- `/pinocchio/bias_error`: `/mujoco/bias_torque - pinocchio_bias_torque`
- `/pinocchio/rne_gravity_error`: `/mujoco/rne_gravity_torque - g(q)`

MuJoCo debug yayinlari:
- `/mujoco/bias_torque`
- `/mujoco/rne_gravity_torque` (MuJoCo RNE ile `qvel=0` gravity benzeri tork)

Hizli kontrol:

```bash
ros2 topic echo /pinocchio/gravity_torque
ros2 topic echo /pinocchio/gravity_error
ros2 topic echo /pinocchio/bias_torque
ros2 topic echo /pinocchio/bias_error
ros2 topic echo /pinocchio/rne_gravity_error
```

Beklenti:
- Robot durgunken hata kucuk kalmali.
- Hata cok buyukse eklem eksen yonu, inertia veya frame uyumsuzlugu kontrol edilmelidir.
- Robot hareketliyken `/pinocchio/bias_error` genelde `/pinocchio/gravity_error`'dan daha anlamli bir karsilastirma verir.
- Kök neden analizi icin en guclu sinyal:
  - `/pinocchio/rne_gravity_error`
  - Bu sinyal kucukse, farkin buyuk kismi `qfrc_bias` tanimindan gelir ve model geometri/inertia tarafi dogrudur.

### Kritik Not: URDF `rpy` ve MJCF `euler` sirasi

Orion5 modelinde kalici gravity farkinin ana nedeni, URDF `rpy` konvansiyonu ile MuJoCo `euler`
konvansiyonunun farkli yorumlanmasi oldu.

Bu projede fix:
- `mujoco/orion5.xml` icinde `compiler` satirina `eulerseq=\"XYZ\"` eklendi.

Dogrulama sonucu:
- Durgun durumda `/pinocchio/rne_gravity_error` degerleri sayisal yuvarlama seviyesine indi
  (tipik olarak `1e-6` ve alti, bazi orneklerde `1e-15` mertebesi).

Pratik cikarim:
- URDF'ten MJCF'e geciste `eulerseq` acik yazilmali.
- Mümkünse uzun vadede `euler` yerine `quat` kullanimi tercih edilmeli.

## Pinocchio Feedforward Closed-Loop (g(q)+PD)

Kapali-cevrim feedforward hold testi:

```bash
ros2 launch mujoco_pendulum orion5_pinocchio_ff_hold.launch.py
```

Varsayilan kontrol formu:
- `tau = g(q) + Kp*(q_ref - q) - Kd*qd`
- `qd` terimi LPF ile filtrelenir (`qd_lpf_alpha`)
- tork komutu eklem-bazli slew-rate limit ile yumusatilir (`torque_rate_limit_vector`)
- ilk acilista referans mevcut poza alinabilir (`hold_current_on_start=true`)
- `q_ref` adim komutu, eklem-bazli hiz/ivme limitli referans ureteci ile yumusatilir:
  - `joint_max_speed_deg`
  - `joint_max_accel_deg`
  - `enable_ref_generator`
- Time-sync acikken tum eklemler hareketi ayni anda bitirecek sekilde ortak sureye esitlenir:
  - `time_sync_enabled`

Varsayilan config:
- `config/tuning/pinocchio_ff_hold_medium.yaml`

Canli izleme:

```bash
ros2 topic echo /pinocchio_ff/cmd_torque
ros2 topic echo /pinocchio_ff/pos_error
ros2 topic echo /joint_states
```

### Test Hazirligi ve Kriterler

1. Home hold:
- hedef: `q_ref = [0, 0, -1.5708, 0, 1.5708, 0]`
- beklenti: hizlar kisa surede sifira iner, buyuk salinim olmaz.

2. Poz adim testi (manuel):
- `ros2 param set /pinocchio_ff_hold_node q_ref \"[...]\"`
- 2-3 farkli pozda tekrar et.
- Not: `q_ref` artik hedef degerdir; node bunu limitli trajeye cevirir.
- Not: `time_sync_enabled=true` iken limitler korunarak ortak sure (synchronized finish) kullanilir.

3. Basari kriteri:
- `|pos_error|` kalici olarak kucuk kalmali.
- `cmd_torque` rated limitlere dayanip kalmamalidir (surekli saturasyon olmamali).
- robotun drift davranisi olmamali.

4. J6 salinim gorulurse:
- once `kd[5]` degerini kucult (ornek `0.02 - 0.08` araligi),
- sonra `kp[5]` degerini kucult (ornek `0.1 - 0.4`),
- gerekiyorsa `torque_rate_limit_vector[5]` degerini dusur.

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
