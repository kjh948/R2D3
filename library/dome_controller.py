#!/usr/bin/python
# Dome position control using encoder A/B and zero-hole (index) sensor.
# Uses Emakefun_MotorHAT for motor control (dome motor on mh.getMotor(3))
# Encoder pins and zero pin follow README.md:
#   encoderA: BCM26
#   encoderB: BCM19
#   zero hole: BCM13
# Run on Raspberry Pi (must run with appropriate permissions).

from Emakefun_MotorHAT import Emakefun_MotorHAT
import RPi.GPIO as GPIO
import time
import threading
import atexit

class DomeController:
    def __init__(
        self,
        addr=0x60,
        motor_port=3,
        encoder_a_pin=26,
        encoder_b_pin=19,
        zero_pin=13,
        counts_per_rev=34,
        kp=0.8,
        min_speed=30,
        max_speed=220,
        tolerance_degrees=1.0,
    ):
        self.mh = Emakefun_MotorHAT(addr=addr)
        atexit.register(self.disable_all_motors)

        # motor
        self.dome_motor = self.mh.getMotor(motor_port)
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.kp = kp

        # encoder / pins
        self.encoder_a_pin = encoder_a_pin
        self.encoder_b_pin = encoder_b_pin
        self.zero_pin = zero_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder_b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.zero_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # counts and scaling
        self.counts_per_rev = counts_per_rev
        self.counts_per_degree = counts_per_rev / 360.0
        self.tolerance_counts = 1#max(1, int(self.counts_per_degree * tolerance_degrees))

        # state
        self._lock = threading.Lock()
        self.encoder_count = 0   # absolute count (set at homing)
        self.homed = False
        self._running = True

        # attach encoder callback (quadrature: use A edges)
        GPIO.add_event_detect(self.encoder_a_pin, GPIO.RISING, callback=self._enc_callback, bouncetime=1)
        # optional: detect zero index asynchronously
        GPIO.add_event_detect(self.zero_pin, GPIO.BOTH, callback=self._zero_callback, bouncetime=1)

    def _enc_callback(self, channel):
        try:
            a = GPIO.input(self.encoder_a_pin)
            b = GPIO.input(self.encoder_b_pin)
            # quadrature decoding: when A changed, direction determined by B
            # depending on wiring, you may need to invert logic
            delta = 1 if b == 0 else -1
            with self._lock:
                self.encoder_count += delta
                print("update:", self.encoder_count)
        except Exception:
            pass

    def _zero_callback(self, channel):
        # when zero sensor is asserted (assume active low), set homed reference
        try:
            state = GPIO.input(self.zero_pin)
            # assume zero hole pulls pin LOW when aligned; adjust if opposite
            if state == 0:
                print("ZERO detected interrupt")
                with self._lock:
                    self.encoder_count = 0
                    self.homed = True
        except Exception:
            print("ZERO callback exception")
            pass

    def disable_all_motors(self):
        try:
            for i in range(1, 5):
                self.mh.getMotor(i).run(Emakefun_MotorHAT.RELEASE)
        except Exception:
            pass
        try:
            GPIO.remove_event_detect(self.encoder_a_pin)
            GPIO.remove_event_detect(self.zero_pin)
            GPIO.cleanup([self.encoder_a_pin, self.encoder_b_pin, self.zero_pin])
        except Exception:
            pass
        self._running = False

    def home(self, search_direction='ccw', speed=10, timeout=20.0):
        """
        Rotate slowly until zero sensor triggers. After detection, encoder_count is set to 0.
        search_direction: 'ccw' or 'cw' (maps to MotorHAT.BACKWARD / FORWARD)
        """
        dir_cmd = Emakefun_MotorHAT.BACKWARD if search_direction == 'ccw' else Emakefun_MotorHAT.FORWARD
        self.dome_motor.setSpeed(speed)
        self.dome_motor.run(dir_cmd)

        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                if self.homed == True:
                    break
            #time.sleep(0.01)

        self.dome_motor.run(Emakefun_MotorHAT.RELEASE)
        return self.homed

    def current_angle(self):
        with self._lock:
            return (self.encoder_count / self.counts_per_degree)

    def move_to_angle(self, target_angle, timeout=30.0):
        """
        Closed-loop movement using encoder counts.
        target_angle: absolute angle in degrees (0..360 typically)
        This implementation assumes homed reference (home() called).
        """
        if not self.homed:
            raise RuntimeError("DomeController: not homed. Call home() before position moves.")

        target_count = int(round(target_angle * self.counts_per_degree))

        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                current = self.encoder_count
            error = target_count - current
            if abs(error) <= self.tolerance_counts:
                break
            #print("Current:", current, "Target:", target_count, "Error:", error)
            # proportional speed
            speed = int(min(self.max_speed, max(self.min_speed, abs(error) * self.kp)))
            self.dome_motor.setSpeed(speed)
            if error < 0:
                self.dome_motor.run(Emakefun_MotorHAT.FORWARD)
            else:
                self.dome_motor.run(Emakefun_MotorHAT.BACKWARD)

            # short wait then re-evaluate
            time.sleep(0.02)

        self.dome_motor.run(Emakefun_MotorHAT.RELEASE)
        return abs(target_count - self.encoder_count) <= self.tolerance_counts

    def move_by_delta(self, delta_degrees, timeout=30.0):
        """Move relative degrees from current position."""
        with self._lock:
            start_count = self.encoder_count
        target_count = start_count + int(round(delta_degrees * self.counts_per_degree))
        return self._move_to_count(target_count, timeout)

    def _move_to_count(self, target_count, timeout=30.0):
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                current = self.encoder_count
            error = target_count - current
            print("Current:", current, "Target:", target_count, "Error:", error)
            if abs(error) <= self.tolerance_counts:
                break
            speed = int(min(self.max_speed, max(self.min_speed, abs(error) * self.kp)))
            self.dome_motor.setSpeed(speed)
            if error < 0:
                self.dome_motor.run(Emakefun_MotorHAT.FORWARD)
            else:
                self.dome_motor.run(Emakefun_MotorHAT.BACKWARD)
            #time.sleep(0.02)
        self.dome_motor.run(Emakefun_MotorHAT.RELEASE)
        return abs(target_count - self.encoder_count) <= self.tolerance_counts

    def set_positions(self, positions_dict):
        """positions: dict mapping name->angle (degrees)"""
        self.positions = positions_dict

    def move_to_named_position(self, name, timeout=30.0):
        if not hasattr(self, 'positions') or name not in self.positions:
            raise KeyError("Unknown position: " + str(name))
        return self.move_to_angle(self.positions[name], timeout=timeout)

if __name__ == "__main__":
    # example usage
    dc = DomeController(counts_per_rev=4096, kp=0.1, min_speed=30, max_speed=150)
    # positions can be filled based on README or project convention
    dc.set_positions({
        "home": 0,
        "open": 5,
        "closed": 0,
    })

    print("Homing...")
    if dc.home(search_direction='ccw', speed=50, timeout=1.0):
        print("Homing successful.")
        pass
    else:
        print("Homing failed, retrying clockwise...")
        dc.home(search_direction='cw', speed=80, timeout=1.0)
    print("Homed. current angle:", dc.current_angle())
    print("Move to open (90 deg)...")
    #ok = dc.move_to_named_position("open")
    dc._move_to_count(3)
    dc._move_to_count(-3)
    dc._move_to_count(2)
    dc._move_to_count(-2)

    print("angle:", dc.current_angle())