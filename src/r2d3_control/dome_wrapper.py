from library.dome_controller import DomeController

class DomeWrapper:
    def __init__(self):
        self.dc = DomeController()
    def home(self):
        return self.dc.home()
    def move_to_count(self, c):
        return self.dc.move_to_count(c)
