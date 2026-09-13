#position -> outputs desired veloicity
#velocity -> outputs desired acceleration, translates to desired roll/pitch angles (x/y) and thrust (z)
#attitude (roll, pitch, yaw) -> outputs desired angular rates
#angular rate -> outputs torque commands -> motor mixing

#the inner loop, angular rate, runs fastest while the outer loop (position) is slowest. 
#the attitude feeds the attitude loop
#the angular velocity feeds the rate rool
#the position/velcoity estimate feeds outer loops

class PID: 
    def __init__(self, kp, ki, kd, output_limits, integral_limits=None, d_filter_alpha=None):

    def reset(self): 
        #clear integral + the last measurement

    def udpate(self): 
        #return clamped output