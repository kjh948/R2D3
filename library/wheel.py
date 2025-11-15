#!/usr/bin/env python3
"""
Open-loop wheel controller with acceleration limiting and simple CLI for R2D3
- Converts linear & angular velocity commands (m/s, rad/s) into left/right motor PWM
- Uses Emakefun_MotorHAT API (open-loop, no encoder feedback)

Features added:
- Acceleration limiting (max linear & angular accelerations)
- Background control loop that ramps to target velocity at specified dt
- Interactive CLI to send target velocities or run demo sequences
"""

import math
import time
import threading
import argparse
from Emakefun_MotorHAT import Emakefun_MotorHAT

class WheelController:
    def __init__(self, motorhat_addr=0x60, wheel_radius=0.03, wheel_base=0.25,
                 max_wheel_rpm=120, max_lin_acc=0.5, max_ang_acc=1.0, control_dt=0.1):
        """Create wheel controller.

        - wheel_radius: meters
        - wheel_base: distance between wheels (track width) in meters
        - max_wheel_rpm: used to map desired wheel angular velocity to motor speed (0-255)
        - max_lin_acc: max linear acceleration (m/s^2)
        - max_ang_acc: max angular acceleration (rad/s^2)
        - control_dt: control loop interval (s)
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_wheel_rpm = max_wheel_rpm
        self.max_lin_acc = max_lin_acc
        self.max_ang_acc = max_ang_acc
        self.dt = control_dt

        self.mh = Emakefun_MotorHAT(addr=motorhat_addr)
        # Assuming motor 1 = left, motor 2 = right (adjust if different wiring)
        self.left_motor = self.mh.getMotor(1)
        self.right_motor = self.mh.getMotor(2)

        # state for ramping
        self._lock = threading.Lock()
        self.cur_linear = 0.0
        self.cur_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0

        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def _omega_to_speed(self, omega):
        """Convert wheel angular velocity (rad/s) to motor speed 0-255 (open-loop)."""
        rpm = abs(omega) * 60.0 / (2.0 * math.pi)
        speed = int(min(255, (rpm / self.max_wheel_rpm) * 255.0))
        return speed

    def _apply_velocity(self, linear, angular):
        """Apply a velocity command immediately (internal)."""
        v_r = linear + (angular * self.wheel_base / 2.0)
        v_l = linear - (angular * self.wheel_base / 2.0)

        omega_r = v_r / self.wheel_radius
        omega_l = v_l / self.wheel_radius

        speed_r = self._omega_to_speed(omega_r)
        speed_l = self._omega_to_speed(omega_l)

        # right motor
        if v_r > 0:
            self.right_motor.run(self.mh.FORWARD)
            self.right_motor.setSpeed(speed_r)
        elif v_r < 0:
            self.right_motor.run(self.mh.BACKWARD)
            self.right_motor.setSpeed(speed_r)
        else:
            self.right_motor.run(self.mh.RELEASE)

        # left motor
        if v_l > 0:
            self.left_motor.run(self.mh.BACKWARD)
            self.left_motor.setSpeed(speed_l)
        elif v_l < 0:
            self.left_motor.run(self.mh.FORWARD)
            self.left_motor.setSpeed(speed_l)
        else:
            self.left_motor.run(self.mh.RELEASE)

    def set_target_velocity(self, linear, angular):
        """Set target velocities (m/s, rad/s). Ramping happens in background loop."""
        with self._lock:
            self.target_linear = float(linear)
            self.target_angular = float(angular)

    def set_velocity_blocking(self, linear, angular, timeout=None):
        """Blocking set: set target then wait until current state reaches it (or timeout)."""
        self.set_target_velocity(linear, angular)
        start = time.time()
        while True:
            with self._lock:
                lin_close = abs(self.cur_linear - linear) < 1e-3
                ang_close = abs(self.cur_angular - angular) < 1e-3
            if lin_close and ang_close:
                return True
            if timeout is not None and (time.time() - start) >= timeout:
                return False
            time.sleep(self.dt)

    def _control_loop(self):
        """Background control loop that ramps current velocities toward target with acceleration limits."""
        while self._running:
            with self._lock:
                tgt_lin = self.target_linear
                tgt_ang = self.target_angular
                cur_lin = self.cur_linear
                cur_ang = self.cur_angular

            # compute allowed change
            max_dlin = self.max_lin_acc * self.dt
            max_dang = self.max_ang_acc * self.dt

            dlin = tgt_lin - cur_lin
            dang = tgt_ang - cur_ang

            # clamp
            if abs(dlin) > max_dlin:
                dlin = math.copysign(max_dlin, dlin)
            if abs(dang) > max_dang:
                dang = math.copysign(max_dang, dang)

            cur_lin += dlin
            cur_ang += dang

            with self._lock:
                self.cur_linear = cur_lin
                self.cur_angular = cur_ang

            # apply to motors
            try:
                self._apply_velocity(cur_lin, cur_ang)
            except Exception as e:
                # don't crash loop; log and continue
                print('Error applying velocity:', e)

            time.sleep(self.dt)

    def stop(self):
        """Stop motors and terminate background loop."""
        # first request zero target and wait a bit for ramp
        self.set_target_velocity(0.0, 0.0)
        time.sleep(max(0.0, 2*self.dt))
        self._running = False
        self._thread.join(timeout=1.0)
        # release motors
        try:
            self.left_motor.run(self.mh.RELEASE)
            self.right_motor.run(self.mh.RELEASE)
        except Exception:
            pass


def _parse_args():
    p = argparse.ArgumentParser(description='Wheel controller CLI (open-loop, ramping)')
    p.add_argument('--addr', type=lambda x: int(x,0), default=0x60, help='MotorHAT I2C address')
    p.add_argument('--wheel-radius', type=float, default=0.03, help='Wheel radius (m)')
    p.add_argument('--wheel-base', type=float, default=0.12, help='Wheel base / track width (m)')
    p.add_argument('--max-rpm', type=float, default=120, help='Max wheel RPM for mapping to 0-255')
    p.add_argument('--max-lin-acc', type=float, default=0.5, help='Max linear accel (m/s^2)')
    p.add_argument('--max-ang-acc', type=float, default=1.0, help='Max angular accel (rad/s^2)')
    p.add_argument('--dt', type=float, default=0.1, help='Control loop dt (s)')
    sub = p.add_subparsers(dest='cmd')

    # one-shot set
    one = sub.add_parser('set', help='Set target linear and angular velocity (one-shot)')
    one.add_argument('linear', type=float, help='Linear velocity (m/s)')
    one.add_argument('angular', type=float, help='Angular velocity (rad/s)')

    # interactive
    sub.add_parser('interactive', help='Interactive prompt to set velocities')

    # demo
    sub.add_parser('demo', help='Run a demo sequence')

    return p.parse_args()


def _interactive_loop(wc):
    print('Interactive mode: enter "linear angular" (m/s rad/s), or "q" to quit')
    try:
        while True:
            line = input('> ').strip()
            if not line:
                continue
            if line.lower().startswith('q') or line.lower().startswith('quit'):
                break
            parts = line.split()
            if len(parts) >= 2:
                try:
                    lin = float(parts[0])
                    ang = float(parts[1])
                    wc.set_target_velocity(lin, ang)
                    print(f'set target linear={lin:.3f} m/s angular={ang:.3f} rad/s')
                except ValueError:
                    print('invalid numbers')
            else:
                print('need two numbers: linear angular')
    except (EOFError, KeyboardInterrupt):
        pass


def _run_demo(wc):
    print('Demo: forward, rotate, reverse, stop')
    try:
        wc.set_target_velocity(0.1, 0.0)
        time.sleep(3)
        wc.set_target_velocity(0.0, 0.6)
        time.sleep(3)
        wc.set_target_velocity(-0.08, 0.0)
        time.sleep(3)
        wc.set_target_velocity(0.0, 0.0)
        time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    args = _parse_args()
    wc = WheelController(motorhat_addr=args.addr, wheel_radius=args.wheel_radius,
                         wheel_base=args.wheel_base, max_wheel_rpm=args.max_rpm,
                         max_lin_acc=args.max_lin_acc, max_ang_acc=args.max_ang_acc,
                         control_dt=args.dt)
    try:
        if args.cmd == 'set':
            wc.set_target_velocity(args.linear, args.angular)
            print('Applied target; press Ctrl-C to stop or run interactive/demo for more control')
            while True:
                time.sleep(1)
        elif args.cmd == 'interactive':
            _interactive_loop(wc)
        else:
            # demo or no command: run demo
            _run_demo(wc)
    except KeyboardInterrupt:
        pass
    finally:
        print('Stopping controller...')
        wc.stop()
