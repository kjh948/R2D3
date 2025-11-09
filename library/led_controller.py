#!/usr/bin/python
"""
LED Controller for R2D3 using WS2812 LEDs
- 2 WS2812 LEDs connected to GPIO6
- Uses rpi_ws281x library for control
"""

import time
import atexit
from rpi_ws281x import PixelStrip, Color

# LED strip configuration:
LED_COUNT = 2        # Number of LED pixels.
LED_PIN = 6         # GPIO pin connected to the pixels (must support PWM!)
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10        # DMA channel to use for generating signal
LED_BRIGHTNESS = 255 # Set to 0 for darkest and 255 for brightest
LED_CHANNEL = 0     # PWM channel 0
LED_INVERT = False  # True to invert the signal (when using NPN transistor level shift)

class LEDController:
    def __init__(self):
        """Initialize LED controller with WS2812 configuration"""
        self.strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()
        self._pattern_running = False
        atexit.register(self.cleanup)

    def cleanup(self):
        """Turn off all LEDs on exit"""
        self.clear()
        self._pattern_running = False

    def clear(self):
        """Turn off all LEDs"""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(0, 0, 0))
        self.strip.show()

    def set_color(self, index, r, g, b):
        """Set specific LED to RGB color"""
        if 0 <= index < LED_COUNT:
            self.strip.setPixelColor(index, Color(r, g, b))
            self.strip.show()

    def set_all_color(self, r, g, b):
        """Set all LEDs to the same RGB color"""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, Color(r, g, b))
        self.strip.show()

    def blink(self, r, g, b, duration=0.5, times=1):
        """Blink LEDs in specified color"""
        for _ in range(times):
            self.set_all_color(r, g, b)
            time.sleep(duration)
            self.clear()
            if times > 1:  # Don't sleep after last blink
                time.sleep(duration)

    def alternate_blink(self, r1, g1, b1, r2, g2, b2, duration=0.5, times=1):
        """Alternate between two colors on the two LEDs"""
        for _ in range(times):
            # First pattern
            self.set_color(0, r1, g1, b1)
            self.set_color(1, r2, g2, b2)
            time.sleep(duration)
            # Second pattern
            self.set_color(0, r2, g2, b2)
            self.set_color(1, r1, g1, b1)
            time.sleep(duration)

    def pulse(self, r, g, b, steps=50, duration=0.05):
        """Pulse LEDs with smooth brightness transitions"""
        for i in range(steps):
            brightness = i / steps
            scaled_r = int(r * brightness)
            scaled_g = int(g * brightness)
            scaled_b = int(b * brightness)
            self.set_all_color(scaled_r, scaled_g, scaled_b)
            time.sleep(duration)
        for i in range(steps, -1, -1):
            brightness = i / steps
            scaled_r = int(r * brightness)
            scaled_g = int(g * brightness)
            scaled_b = int(b * brightness)
            self.set_all_color(scaled_r, scaled_g, scaled_b)
            time.sleep(duration)

    # Predefined patterns
    def pattern_warning(self, duration=2.0):
        """Warning pattern - Red blink"""
        self.blink(255, 0, 0, duration=0.2, times=int(duration/0.4))

    def pattern_success(self, duration=2.0):
        """Success pattern - Green pulse"""
        steps = int(duration * 10)  # 10 steps per second
        self.pulse(0, 255, 0, steps=steps, duration=duration/steps/2)

    def pattern_thinking(self, duration=None):
        """Thinking pattern - Blue alternating"""
        count = 0
        self._pattern_running = True
        while self._pattern_running and (duration is None or count < duration):
            self.alternate_blink(0, 0, 255, 0, 0, 128, duration=0.5, times=1)
            if duration is not None:
                count += 1

    def stop_pattern(self):
        """Stop any running pattern"""
        self._pattern_running = False
        self.clear()

def create_color(r, g, b):
    """Helper function to create 24-bit color value"""
    return Color(r, g, b)

# Common colors
RED = create_color(255, 0, 0)
GREEN = create_color(0, 255, 0)
BLUE = create_color(0, 0, 255)
WHITE = create_color(255, 255, 255)
OFF = create_color(0, 0, 0)

if __name__ == "__main__":
    # Example usage
    led = LEDController()
    
    try:
        print("Testing LED patterns...")
        
        # Basic color test
        print("Testing solid colors...")
        for color in [RED, GREEN, BLUE, WHITE]:
            led.set_all_color(color)
            time.sleep(1)
        
        # Test patterns
        print("Warning pattern...")
        led.pattern_warning(duration=2.0)
        
        print("Success pattern...")
        led.pattern_success(duration=2.0)
        
        print("Thinking pattern (3 seconds)...")
        led.pattern_thinking(duration=3)
        
        # Clean up
        led.clear()
        print("Test complete")
        
    except KeyboardInterrupt:
        print("Stopping...")
        led.cleanup()
