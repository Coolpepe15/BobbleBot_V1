# BobbleBot en Raspberry Pi 4 (ROS 2 Jazzy / Ubuntu 24.04)

Esta carpeta es un **paquete nuevo**, hermano de [`bobble_controllers`](../bobble_controllers),
que porta el controlador de balance (PID en cascada) del proyecto original —
pensado para ROS 1 + Gazebo — a **ROS 2 Jazzy** corriendo sobre hardware real
en una Raspberry Pi 4:

- Driver puente H **L298N** (2 canales, uno por rueda)
- IMU **MPU6050** (acelerómetro + giroscopio, I2C)
- Motorreductores **JGY-370**
- Batería **LiPo 11.1V 2200mAh 3S**

No es una copia 1:1 del código de `bobble_controllers` (ese es C++ sobre
`ros_control`/Gazebo, pensado para un IMU BNO055 simulado con una orientación
de montaje fija). Es un **puerto en Python** de la misma lógica de control
(cascada Velocidad → Inclinación deseada → Esfuerzo de motor, más el lazo de
giro), adaptado para hardware real y para que la orientación del IMU se
pueda **calibrar sin tocar código** (ver más abajo). Todo el detalle de qué
se portó igual y qué se adaptó está comentado en el código fuente de cada
archivo.

## Estructura

```
bobble_pi_ros2/
├── bobble_interfaces/          # Paquete de mensajes ROS 2 (ament_cmake)
│   └── msg/BobbleBotStatus.msg, ControlCommands.msg
└── bobble_pi/                  # Nodos Python (ament_python)
    ├── bobble_pi/
    │   ├── mpu6050.py              # Driver I2C del MPU6050
    │   ├── tilt_estimator.py       # Filtro complementario de inclinación
    │   ├── motor_driver.py         # L298N + encoders JGY-370 (gpiozero)
    │   ├── difference_filter.py    # Filtros digitales (puerto de Filter.cpp)
    │   ├── pid_control.py          # PID en cascada (puerto de PidControl.cpp)
    │   ├── imu_node.py             # Publica sensor_msgs/Imu
    │   ├── balance_controller_node.py  # Nodo principal de control
    │   ├── keyboard_control_node.py
    │   └── joystick_control_node.py
    ├── config/balance_control.yaml
    └── launch/bringup.launch.py
```

## 1. Requisitos de hardware y cableado

> ⚠️ **Seguridad eléctrica primero.** Una LiPo 3S en corto o mal conectada
> puede incendiarse. Usa siempre un cargador balanceador para cargarla,
> nunca la descargues por debajo de ~3.3V por celda, guárdala en una bolsa
> LiPo-safe, y coloca un fusible/interruptor accesible entre la batería y el
> resto del circuito para poder cortar la energía de inmediato.

### Alimentación

- La LiPo 11.1V alimenta el **L298N** (terminal de motores) y, a través de
  su regulador a bordo (o mejor, un **buck converter 5V/3A dedicado**),
  puede alimentar la Raspberry Pi. **No** uses el 5V de salida del L298N
  para la Pi si vas a exigir corriente a los motores: las caídas de tensión
  al arrancar los motores pueden resetear la Pi a mitad de un balanceo.
  Lo más robusto es un buck converter aparte, dedicado solo a la Pi.
- Une **todos los GND**: LiPo/L298N, buck converter, Pi y MPU6050 deben
  compartir una referencia de tierra común.
- El MPU6050 se alimenta a **3.3V** (normalmente tiene regulador a bordo
  que acepta 3.3-5V, pero sus líneas I2C SDA/SCL son 3.3V lógicos —
  confirma el datasheet de tu módulo específico antes de conectarlo
  directo a la Pi).

### MPU6050 (I2C)

| MPU6050 | Raspberry Pi 4 (BCM) | Pin físico |
|---|---|---|
| VCC | 3V3 | pin 1 |
| GND | GND | pin 6 |
| SDA | GPIO2 (SDA1) | pin 3 |
| SCL | GPIO3 (SCL1) | pin 5 |
| AD0 | GND (dirección 0x68) | — |

### L298N (2 motores)

Los pines por defecto en `config/balance_control.yaml` son (BCM):

| Función | Rueda izquierda | Rueda derecha |
|---|---|---|
| IN1 | GPIO5 | GPIO20 |
| IN2 | GPIO6 | GPIO21 |
| ENA/ENB (PWM) | GPIO12 | GPIO13 |

Conecta el terminal de alimentación de motores del L298N (12V/motor power)
a la LiPo, GND del L298N al GND común, y sus salidas OUT1/OUT2 y OUT3/OUT4 a
cada motor JGY-370. **Ajusta estos números de pin a tu cableado real** antes
de correr nada — son BCM (numeración de GPIO), no los números físicos del
header de 40 pines.

### Encoders del JGY-370 (opcional pero recomendado)

Si tus JGY-370 traen encoder Hall, puedes cablear sus canales A/B a GPIOs
libres y habilitar `use_wheel_odometry: true` en el YAML (ver sección de
calibración). Si el encoder de tu módulo saca lógica de 5V, usa un
divisor de voltaje o level-shifter antes de meterlo a un GPIO de la Pi —
los GPIO de la Pi **no toleran 5V**.

Si no vas a cablear encoders todavía, deja `use_wheel_odometry: false`
(el valor por defecto): el robot igual puede pararse y moverse, usando una
inclinación de referencia calculada directamente del comando de velocidad
en lugar de realimentación real de velocidad de rueda. Es una
simplificación deliberada — sin ella no hay forma de portar el lazo de
velocidad del proyecto original sin inventar datos de encoder que no
tenemos.

## 2. Sistema operativo y ROS 2 Jazzy

1. Graba **Ubuntu Server 24.04 LTS (arm64)** en la SD/SSD con Raspberry Pi
   Imager, habilitando SSH y tu usuario desde el propio Imager.
2. Habilita I2C:

   ```sh
   sudo raspi-config    # si está disponible, o edita /boot/firmware/config.txt
   # añade/descomenta: dtparam=i2c_arm=on
   sudo reboot
   sudo apt install -y i2c-tools
   i2cdetect -y 1        # deberías ver "68" en la tabla si el MPU6050 responde
   ```

3. Instala ROS 2 Jazzy siguiendo la guía oficial para Ubuntu 24.04:
   <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
   (paquete `ros-jazzy-ros-base` es suficiente, no necesitas el desktop
   completo en la Pi).

4. Instala dependencias del sistema y de Python:

   ```sh
   sudo apt install -y python3-colcon-common-extensions python3-pip \
       ros-jazzy-joy
   pip install --break-system-packages smbus2 gpiozero lgpio
   ```

   `lgpio` es el backend recomendado de `gpiozero` en Ubuntu 24.04 sobre
   Raspberry Pi 4 (el backend clásico `RPi.GPIO` no funciona bien fuera de
   Raspberry Pi OS). gpiozero lo detecta automáticamente si está instalado.

## 3. Compilar el workspace

Copia (o clona) esta carpeta dentro del `src` de un workspace de colcon en
la Pi:

```sh
mkdir -p ~/bobble_ws/src
cp -r bobble_pi_ros2/bobble_interfaces bobble_pi_ros2/bobble_pi ~/bobble_ws/src/
cd ~/bobble_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Calibración (obligatoria antes de intentar balancear)

### 4.1 Sesgo del giroscopio

Automático: `imu_node` promedia el giroscopio al arrancar (parámetro
`gyro_calibration_samples`, 400 muestras por defecto). Mantén el robot
**quieto y nivelado** durante ese arranque.

### 4.2 Ejes del IMU (crítico)

El proyecto original asumía una orientación fija de montaje del IMU que no
podemos replicar a ciegas para tu chasis físico. En vez de adivinar, calibra
así con el robot **apoyado, sin correr el controlador de balance**:

1. Lanza solo el IMU: `ros2 run bobble_pi imu_node`
2. En otra terminal: `ros2 topic echo /imu/data_raw`
3. Con el robot parado verticalmente (como si balanceara), inclínalo
   suavemente hacia adelante (la dirección en la que "cae" si no
   balanceara). Observa qué eje de `linear_acceleration` cambia de forma
   más clara — ese es tu `accel_fall_axis`. El eje que marca ~9.8 m/s² con
   el robot vertical es tu `accel_vertical_axis` (normalmente `z`).
4. Gira el robot en el mismo sentido (inclinación adelante/atrás) y mira
   qué eje de `angular_velocity` responde — ese es `gyro_tilt_axis`.
5. Gira el robot sobre su eje vertical (como girando en el lugar) y anota
   qué eje de `angular_velocity` responde — ese es `gyro_yaw_axis`.
6. Ajusta los `*_sign` (1.0 o -1.0) hasta que: inclinar hacia adelante dé
   un **`tilt` que aumenta** en `/bobble/bb_controller_status`, y que el
   signo de `tilt_dot` coincida con el signo de la inclinación creciente
   (mismo signo = sin invertir).

Actualiza esos 8 valores en `config/balance_control.yaml`
(`accel_fall_axis/sign`, `accel_vertical_axis/sign`, `gyro_tilt_axis/sign`,
`gyro_yaw_axis/sign`).

### 4.3 Offset de inclinación

Una vez que el robot balancee razonablemente pero tienda a irse hacia un
lado, ajusta `tilt_offset_deg` en pequeños incrementos (ej. 0.5°) para
compensar un IMU que no quedó perfectamente nivelado en el chasis.

## 5. Probar y correr

**Antes de balancear**, prueba los motores con las ruedas en el aire:

```sh
ros2 launch bobble_pi bringup.launch.py
# En otra terminal:
ros2 topic pub --once /bobble/bb_cmd bobble_interfaces/msg/ControlCommands "{diagnostic_cmd: true}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

En modo `DIAGNOSTIC` el nodo manda `linear.x`/`angular.z` directo como
esfuerzo de motor (sin pasar por el control de balance), así puedes
confirmar que cada rueda gira en el sentido correcto antes de arriesgarte a
balancear. Corrige `left_motor_invert`/`right_motor_invert` si giran al
revés. Vuelve a modo IDLE con:

```sh
ros2 topic pub --once /bobble/bb_cmd bobble_interfaces/msg/ControlCommands "{idle_cmd: true}"
```

Para balancear de verdad, con teclado:

```sh
ros2 launch bobble_pi bringup.launch.py
# En otra terminal, con foco en esa ventana:
ros2 run bobble_pi keyboard_control_node
# barra espaciadora para activar (pasa a STARTUP -> BALANCE)
```

O con joystick (requiere `ros2 run joy joy_node` corriendo):

```sh
ros2 run bobble_pi joystick_control_node
```

## 6. Ajuste de ganancias (tuning)

Los parámetros en `config/balance_control.yaml` tienen el mismo
significado que en `bobble_controllers/config/bobble_sim_balance_control.yaml`
del proyecto original: `tilt_control_kp/kd` es el lazo interno que
realmente sostiene el balance (empieza por aquí), `velocity_control_*`
ajusta qué tan agresivo persigue la velocidad deseada (solo aplica si
`use_wheel_odometry: true`), y `turning_control_*` el giro. Sube ganancias
gradualmente y prueba siempre primero con las ruedas apoyadas sobre una
superficie blanda o sujetando el robot con la mano.

## 7. Limitaciones conocidas

- **Frecuencia de lazo**: por defecto 100Hz en Python vía `rclpy`, frente a
  los 500Hz en C++ con `ros_control` del proyecto original. Es una
  frecuencia realista para CPython en una Pi 4, pero si notas jitter o el
  robot oscila de forma errática, es la primera sospechosa (baja
  `control_loop_frequency_hz` o considera portar el nodo crítico a C++ más
  adelante).
- **Sin magnetómetro**: el MPU6050 no trae magnetómetro, así que `heading`
  en `BobbleBotStatus` es solo telemetría que deriva con el tiempo — no se
  usa para controlar (el control de giro usa velocidad angular, no rumbo
  absoluto, igual que el proyecto original).
- **Encoder de un solo canal**: si tu JGY-370 solo tiene una salida de
  pulsos (no cuadratura A/B), la dirección de giro se infiere del último
  comando de esfuerzo enviado al motor, no se mide — puede fallar si el
  motor sigue girando por inercia tras cortar el esfuerzo.
- **`pulses_per_revolution`** en el YAML es un valor de ejemplo — mídelo
  para tu motor/reductora específicos antes de confiar en la odometría.

## 8. Solución de problemas

- `i2cdetect -y 1` no muestra `68`: revisa cableado SDA/SCL y que I2C esté
  habilitado (`dtparam=i2c_arm=on`).
- `RuntimeError: smbus2 is not installed` / errores de `gpiozero` sobre pin
  factory: confirma que instalaste `smbus2`, `gpiozero` y `lgpio` con pip
  (paso 2.4) y que corres como un usuario en el grupo `gpio`/`i2c`
  (`sudo usermod -aG gpio,i2c $USER`, luego cierra sesión y vuelve a entrar).
- El robot se va de bruces apenas entra a `BALANCE`: revisa primero la
  calibración de ejes (sección 4.2) — es la causa más común, y con los
  ejes/seños equivocados el controlador literalmente empuja en la
  dirección incorrecta.
