import RPi.GPIO as GPIO
import time

# Define the GPIO pins connected to the encoder
ENCODER_A_PIN = 26  # Example pin for encoder A
ENCODER_B_PIN = 19  # Example pin for encoder B

# Initialize GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM numbering
GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Enable internal pull-up resistors
GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global variables for encoder state
encoder_pos = 0
last_encoder_A_state = GPIO.input(ENCODER_A_PIN)

# Interrupt handler function
def encoder_callback(channel):
    global encoder_pos
    global last_encoder_A_state

    current_encoder_A_state = GPIO.input(ENCODER_A_PIN)
    current_encoder_B_state = GPIO.input(ENCODER_B_PIN)

    if current_encoder_A_state != last_encoder_A_state: # Check for a change on A
        if current_encoder_A_state == GPIO.LOW: # Falling edge on A
            if current_encoder_B_state == GPIO.HIGH:
                encoder_pos += 1  # Clockwise rotation
            else:
                encoder_pos -= 1  # Counter-clockwise rotation
        # No action on rising edge of A (can be used for more robust debouncing)

    last_encoder_A_state = current_encoder_A_state # Update last state

# Attach the interrupt handler to both encoder pins
# Trigger on both rising and falling edges for more precise detection
GPIO.add_event_detect(ENCODER_A_PIN, GPIO.BOTH, callback=encoder_callback)#, bouncetime=1) 
# bouncetime helps with debouncing; adjust as needed

try:
    while True:
        print("Encoder Position:", encoder_pos)
        time.sleep(0.1) # Print position periodically
except KeyboardInterrupt:
    GPIO.cleanup() # Clean up GPIO on exit