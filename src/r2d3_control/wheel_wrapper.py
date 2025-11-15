from library.wheel import WheelController

class WheelWrapper:
    def __init__(self):
        self.wc = WheelController()
    def set_target_velocity(self, lin, ang):
        self.wc.set_target_velocity(lin, ang)
