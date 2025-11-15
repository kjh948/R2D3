from library.led_controller import LEDController
import threading

class LEDWrapper:
    def __init__(self):
        self.led = LEDController()
        self._thinking_thread = None

    def set_all_color(self, r, g, b):
        self.led.set_all_color(r,g,b)

    def pattern_warning(self, duration=2.0):
        self.led.pattern_warning(duration=duration)

    def pattern_success(self, duration=2.0):
        self.led.pattern_success(duration=duration)

    def pattern_thinking(self, duration=None):
        # run thinking pattern in a thread so it doesn't block the caller
        def _run():
            self.led.pattern_thinking(duration=duration)
        if self._thinking_thread and self._thinking_thread.is_alive():
            return
        self._thinking_thread = threading.Thread(target=_run, daemon=True)
        self._thinking_thread.start()

    def stop_pattern(self):
        self.led.stop_pattern()
