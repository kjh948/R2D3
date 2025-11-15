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

        self.tolerance_counts = 1
        # state
        self._lock = threading.Lock()
        self.encoder_count = 0   # absolute count (set at homing)
        self.homed = True
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
            if self.homed == True:
                break
            #time.sleep(0.01)
        if self.homed == False:
            dir_cmd = Emakefun_MotorHAT.FORWARD if search_direction == 'ccw' else Emakefun_MotorHAT.BACKWARD
            speed *= 1.5
            self.dome_motor.setSpeed(speed)
            self.dome_motor.run(dir_cmd)
            while time.time() - start < timeout:
                if self.homed == True:
                    break
        self.dome_motor.run(Emakefun_MotorHAT.RELEASE)
        return self.homed

    def current_count(self):
        with self._lock:
            return (self.encoder_count )

    def move_to_count(self, target_count, timeout=30.0):
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

        self.dome_motor.run(Emakefun_MotorHAT.RELEASE)
        return abs(target_count - self.encoder_count) <= self.tolerance_counts


if __name__ == "__main__":
    # example usage
    dc = DomeController(counts_per_rev=4096, kp=0.1, min_speed=30, max_speed=150)

    print("Homing...")
    dc.home(search_direction='ccw', speed=50, timeout=5.0)
    # if :
    #     print("Homing successful.")
    #     pass
    # else:
    #     print("Homing failed, retrying clockwise...")
    #     dc.home(search_direction='cw', speed=80, timeout=1.0)
    print("Homed. current angle:", dc.current_count())

    dc.move_to_count(5)
    dc.move_to_count(3)
    dc.move_to_count(0)
    dc.move_to_count(-3)
    dc.move_to_count(2)
    dc.move_to_count(0)

    print("count:", dc.current_count())