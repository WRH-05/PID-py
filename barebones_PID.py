class PID:
    def __init__(self, kp, ki, kd):
        self.k = (kp, ki, kd)
        self.integral = self.prev_err = 0

    def step(self, target, current):
        err = target - current
        self.integral += err
        output = (self.k[0]*err) + (self.k[1]*self.integral) + (self.k[2]*(err - self.prev_err))
        self.prev_err = err
        return output
    

# 1. Initialize (Proportional, Integral, Derivative constants)
controller = PID(1.0, 0.1, 0.05) 

# 2. Run in your loop
target_pos = 100
current_pos = 0

while True:
    # ... read sensor to get current_pos ...
    
    correction = controller.step(target_pos, current_pos)
    
    # ... apply 'correction' to motor/actuator ...
    
    # Update mock position for demonstration
    current_pos += correction * 0.1 
    print(f"Current: {current_pos:.2f}, Output: {correction:.2f}")
    
    if abs(target_pos - current_pos) < 0.1: break