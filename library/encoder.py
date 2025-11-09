import RPi.GPIO as GPIO
import time

# Encoder 1 pin definitions
ENCODER1_A_PIN = 26  # Right Encoder Yellow wire to BCM pin 26
ENCODER1_B_PIN = 19  # Right Encoder Green Wire  to BCM pin 19

# Encoder 2 pin definitions
ENCODER2_A_PIN = 13  # Left Encoder Yellow wire to BCM pin  13
ENCODER2_B_PIN = 6   # Left Encoder Green Wire  to BCM pin 6

# Initialize position counters
position1 = 0
position2 = 0

# Set up the GPIO library
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER1_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER1_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER2_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER2_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder state tracking
a1_last_state = GPIO.input(ENCODER1_A_PIN)
a2_last_state = GPIO.input(ENCODER2_A_PIN)

# Callback function to update the position for Encoder 1
def update_position1(channel):
    global position1, a1_last_state
    a1_state = GPIO.input(ENCODER1_A_PIN)
    b1_state = GPIO.input(ENCODER1_B_PIN)
    
    # Check direction for Encoder 1
    if a1_state != a1_last_state:
        if b1_state != a1_state:
            position1 += 1  # Clockwise
        else:
            position1 -= 1  # Counter-clockwise
        print("Encoder 1 Position:",position1)

# Callback function to update the position for Encoder 2
def update_position2(channel):
    global position2, a2_last_state
    a2_state = GPIO.input(ENCODER2_A_PIN)
    b2_state = GPIO.input(ENCODER2_B_PIN)
    
    # Check direction for Encoder 2
    if a2_state != a2_last_state:
        if b2_state != a2_state:
            position2 += 1  # Clockwise
        else:
            position2 -= 1  # Counter-clockwise
        print("Encoder 2 Position: ",position2)

# Attach event listeners to the encoder A pins
GPIO.add_event_detect(ENCODER1_A_PIN, GPIO.BOTH, callback=update_position1)
#GPIO.add_event_detect(ENCODER2_A_PIN, GPIO.BOTH, callback=update_position2)

try:
    print("Reading encoder positions. Press Ctrl+C to exit.")
    while True:
        time.sleep(0.1)  # Delay for readability
except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    GPIO.cleanup()
