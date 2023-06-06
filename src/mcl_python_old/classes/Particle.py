class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1
    
    def reset_weight(self):
        self.weight = 1

    def set_weight(self, weight):
        self.weight = weight

    def print_pos(self):
        print(self.x, self.y, self.theta)

