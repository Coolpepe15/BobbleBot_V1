"""Interactive MPU6050 axis calibration helper.

The single most common reason a BobbleBot faceplants the instant it enters
BALANCE is a wrong axis/sign mapping in balance_control.yaml: the controller
then pushes the wheels in the direction the robot is already falling. The
README describes calibrating those 8 values by eye from `ros2 topic echo`;
this tool does the same thing by measurement and prints the YAML block to
paste.

Run it with the balance controller NOT running (it talks to the MPU6050
over I2C directly, no ROS needed):

    ros2 run bobble_pi imu_calibrate

It walks through three poses and then shows a live tilt readout so you can
confirm the result before trusting it.
"""

import math
import sys
import time

from bobble_pi.mpu6050 import Mpu6050
from bobble_pi.tilt_estimator import ComplementaryFilter, accel_tilt_angle, select_axis

AXES = ('x', 'y', 'z')
GRAVITY = 9.80665


def _prompt(text):
    try:
        input(f'\n>>> {text} y pulsa ENTER...')
    except (EOFError, KeyboardInterrupt):
        print('\nCancelado.')
        sys.exit(1)


def _average(imu, seconds, label):
    """Average accel + gyro over a window, and also return the peak-magnitude
    gyro sample so a short motion isn't washed out by the still time around it."""
    print(f'    Midiendo {label} ({seconds:.0f} s)...')
    accel_sum = [0.0, 0.0, 0.0]
    gyro_sum = [0.0, 0.0, 0.0]
    gyro_peak = [0.0, 0.0, 0.0]
    count = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        accel, gyro = imu.read()
        for i in range(3):
            accel_sum[i] += accel[i]
            gyro_sum[i] += gyro[i]
            if abs(gyro[i]) > abs(gyro_peak[i]):
                gyro_peak[i] = gyro[i]
        count += 1
        time.sleep(0.005)
    accel_avg = tuple(v / count for v in accel_sum)
    gyro_avg = tuple(v / count for v in gyro_sum)
    return accel_avg, gyro_avg, tuple(gyro_peak)


def _dominant(vector, exclude=None):
    """Index of the largest-magnitude component, optionally skipping one axis."""
    best, best_mag = None, -1.0
    for i in range(3):
        if exclude is not None and i == exclude:
            continue
        if abs(vector[i]) > best_mag:
            best, best_mag = i, abs(vector[i])
    return best, vector[best]


def calibrate(imu):
    print(__doc__.split('Run it with')[0])
    print('=' * 62)
    print('CALIBRACION DE EJES DEL MPU6050')
    print('=' * 62)
    print('Ten el robot en la mano. NO conectes la bateria de los motores;')
    print('esta rutina no mueve nada, solo lee la IMU.')

    # --- 1. Upright: vertical axis + gyro bias -------------------------
    _prompt('Pon el robot VERTICAL (como cuando balancea) y sujetalo QUIETO')
    print('    Calibrando el sesgo del giroscopio (no lo muevas)...')
    imu.calibrate_gyro(samples=400)
    upright_accel, _, _ = _average(imu, 2.0, 'la pose vertical')

    magnitude = math.sqrt(sum(v * v for v in upright_accel))
    print(f'    accel vertical = {_fmt(upright_accel)}  |a| = {magnitude:.2f} m/s^2')
    if abs(magnitude - GRAVITY) > 1.5:
        print(f'    !! |a| deberia ser ~{GRAVITY:.1f}. Revisa el cableado del MPU6050.')

    v_idx, v_val = _dominant(upright_accel)
    vertical_axis = AXES[v_idx]
    vertical_sign = 1.0 if v_val > 0 else -1.0
    others = [i for i in range(3) if i != v_idx]
    cross = math.sqrt(sum(upright_accel[i] ** 2 for i in others))
    print(f'    -> eje vertical: {vertical_axis} (signo {vertical_sign:+.0f})')
    if cross > 2.0:
        tilted = math.degrees(math.atan2(cross, abs(v_val)))
        print(f'    !! El robot esta ~{tilted:.0f}° fuera de la vertical. '
              'Enderezalo y vuelve a correr esto.')

    # --- 2. Lean forward: fall axis + gyro tilt axis -------------------
    _prompt('Inclina el robot HACIA ADELANTE unos 30° despacio, en 4 segundos,\n'
            '    y dejalo inclinado. Empieza cuando')
    fwd_accel, _, fwd_peak = _average(imu, 4.0, 'la inclinacion adelante')

    delta = tuple(fwd_accel[i] - upright_accel[i] for i in range(3))
    f_idx, f_val = _dominant(delta, exclude=v_idx)
    fall_axis = AXES[f_idx]
    # tilt must GROW when leaning forward, and tilt = atan2(fall, vertical),
    # so the signed fall component has to move positive during that motion.
    # Keying off the delta (not the absolute reading) keeps this right even
    # when the axis has a nonzero offset while upright.
    fall_sign = 1.0 if f_val > 0 else -1.0
    print(f'    delta accel = {_fmt(delta)}')
    print(f'    -> eje de caida: {fall_axis} (signo {fall_sign:+.0f})')
    if abs(f_val) < 1.0:
        print('    !! El cambio es muy pequeño. Inclina mas (30-45°) y repite.')

    g_idx, _ = _dominant(fwd_peak)
    gyro_tilt_axis = AXES[g_idx]
    gyro_tilt_sign = 1.0 if fwd_peak[g_idx] > 0 else -1.0
    print(f'    gyro pico  = {_fmt(fwd_peak)}')
    print(f'    -> eje de giro de inclinacion: {gyro_tilt_axis} (signo {gyro_tilt_sign:+.0f})')
    if abs(fwd_peak[g_idx]) < 0.15:
        print('    !! Apenas se detecto rotacion. Muevelo mas rapido y repite.')
    if g_idx == v_idx:
        print('    !! El eje del giroscopio coincide con el vertical: sospechoso.')

    # --- 3. Yaw left: yaw axis ----------------------------------------
    _prompt('Vuelve el robot a VERTICAL. Luego giralo sobre si mismo hacia la\n'
            '    IZQUIERDA (antihorario visto desde arriba) durante 4 segundos.\n'
            '    Empieza cuando')
    _, _, yaw_peak = _average(imu, 4.0, 'el giro en el sitio')

    y_idx, _ = _dominant(yaw_peak)
    gyro_yaw_axis = AXES[y_idx]
    # ROS convention: +angular.z = turn left (counter-clockwise).
    gyro_yaw_sign = 1.0 if yaw_peak[y_idx] > 0 else -1.0
    print(f'    gyro pico  = {_fmt(yaw_peak)}')
    print(f'    -> eje de yaw: {gyro_yaw_axis} (signo {gyro_yaw_sign:+.0f})')
    if abs(yaw_peak[y_idx]) < 0.15:
        print('    !! Apenas se detecto rotacion. Giralo mas rapido y repite.')

    return {
        'accel_fall_axis': fall_axis,
        'accel_fall_sign': fall_sign,
        'accel_vertical_axis': vertical_axis,
        'accel_vertical_sign': vertical_sign,
        'gyro_tilt_axis': gyro_tilt_axis,
        'gyro_tilt_sign': gyro_tilt_sign,
        'gyro_yaw_axis': gyro_yaw_axis,
        'gyro_yaw_sign': gyro_yaw_sign,
    }


def _fmt(vec):
    return 'x=%7.3f y=%7.3f z=%7.3f' % vec


def print_yaml(result):
    print()
    print('=' * 62)
    print('PEGA ESTO EN config/balance_control.yaml (bajo balance_controller_node)')
    print('=' * 62)
    for key in ('accel_fall_axis', 'accel_vertical_axis'):
        print(f'    {key}: "{result[key]}"')
        sign_key = key.replace('_axis', '_sign')
        print(f'    {sign_key}: {result[sign_key]:.1f}')
    for key in ('gyro_tilt_axis', 'gyro_yaw_axis'):
        print(f'    {key}: "{result[key]}"')
        sign_key = key.replace('_axis', '_sign')
        print(f'    {sign_key}: {result[sign_key]:.1f}')
    print('    tilt_offset_deg: 0.0    # ajustalo despues, ver abajo')
    print('=' * 62)


def monitor(imu, result):
    """Live tilt readout using the freshly computed mapping, so the numbers
    can be sanity-checked before the robot is ever allowed to drive."""
    print('\nVerificacion en vivo. Deberias ver:')
    print('  * robot vertical      -> tilt cerca de 0')
    print('  * inclinado adelante  -> tilt POSITIVO y creciendo')
    print('  * cayendo adelante    -> tilt_dot POSITIVO (mismo signo que el tilt)')
    print('  * girando a la izq.   -> yaw_rate POSITIVO')
    print('\nAnota el tilt medio con el robot en su punto de equilibrio real:')
    print('ese valor es tu tilt_offset_deg. Ctrl-C para salir.\n')

    tilt_filter = ComplementaryFilter(0.98)
    last = time.monotonic()
    try:
        while True:
            accel, gyro = imu.read()
            now = time.monotonic()
            dt, last = now - last, now

            fall = select_axis(accel, result['accel_fall_axis'], result['accel_fall_sign'])
            vertical = select_axis(accel, result['accel_vertical_axis'],
                                   result['accel_vertical_sign'])
            tilt = tilt_filter.update(accel_tilt_angle(fall, vertical),
                                      select_axis(gyro, result['gyro_tilt_axis'],
                                                  result['gyro_tilt_sign']), dt)
            tilt_dot = select_axis(gyro, result['gyro_tilt_axis'], result['gyro_tilt_sign'])
            yaw_rate = select_axis(gyro, result['gyro_yaw_axis'], result['gyro_yaw_sign'])

            sys.stdout.write(
                f'\r  tilt = {math.degrees(tilt):+7.2f}°   '
                f'tilt_dot = {math.degrees(tilt_dot):+8.2f}°/s   '
                f'yaw_rate = {math.degrees(yaw_rate):+8.2f}°/s    ')
            sys.stdout.flush()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print('\n')


def main(args=None):
    imu = Mpu6050(bus_num=1, address=0x68)
    result = calibrate(imu)
    print_yaml(result)
    monitor(imu, result)


if __name__ == '__main__':
    main()
