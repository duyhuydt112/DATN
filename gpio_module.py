""" 
gpiozero_extended.py contains classes that are not implemented in GPIO Zero
(https://gpiozero.readthedocs.io/en/stable/) or that could use a different
implementation which is more suitable for automation projects.

Author: Tran Tan Phat
    ver 0.0.1
    05/04/2025
"""

from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder
import time

class Motor:
    """
    The class represent a DC motor controlled with H-bridge driver.
    An encoder can be used to measure the angular position of output

    Type of driver was used:
        - Single PWM with dual enable for forward and backward rotation control (L298N in this case)
    
    Args:
        enable1(int | str): GPIO pin connect to enable1 of driver
        enable2(int | str): GPIO pin connect to enable2 of driver
        pwm1(int | str): GPIO pin connect to PWM1 of driver
        encoder1(int or str): GPIO pin connect to the encoder phase A
        encoder2(int or str): GPIO pin connect to the encoder phase B
        encoderppr(int): The number of pulses per revolution (ppr) of encoder - Default value = 300
    """

    def __init__(self, enable1=None, enable2=None, pwm1=None, encoder1=None, encoder2=None, encoderppr=300):
        """
        Class Constructor
        """
        if enable1 and enable2:
            if not pwm1:
                raise Exception(" 'pwm1' pin is undefined. \n")
            self._dualpwm = False
            self._enable1 = DigitalOutputDevice(enable1)
            self._enable2 = DigitalOutputDevice(enable2)
            self._pwm1 = PWMOutputDevice(pwm1)
        else:
            raise Exception('Pin configuration is incorrect.')
        
        if encoder1 and encoder2:
            self._encoder = RotaryEncoder(encoder1, encoder2, max_steps=0)         #max_steps: maximum value of encoder before reset to zero
            self._ppr = encoderppr
        else:
            self._encoder = None

        self._value = 0         #motor output
        self._angle = 0         #origin angle value 
    
    def __del__(self):
        """
        Class Destructor
        """
        if not self._dualpwm:
            self._enable1.close()
            self._enable2.close()
            self._pwm1.close()

        if self._encoder:
            time.sleep(1)       # Added this to avoid segmentation fault :(
            self._encoder.close()

    @property
    def value(self):
        """
        Contain motor output('read_only')
        Values can be between ``-1`` (full speed backward) and ``1`` (full
        speed forward), with ``0`` being stopped.
        """
        return self._value
    
    @value.setter
    def value(self, _):
        print('"value" is a read only attribute.')

    def get_angle(self):
        """
        Get the traveled angle of encoder
        """
        if self._encoder:
            angle = 360 / self._ppr * self._encoder.steps - self._angle
        else:
            angle = None
        return angle
    
    def reset_angle(self):
        """
        Reset the origin angle of encoder 
        """
        if self._encoder:
            self._angle = 360 / self._ppr * self._encoder.steps

    def set_output(self, output, brake=False):
        """
        Set motor duty cycle
        Args:
            output(float): the PWM duty cycle value (range: -1 -> 1)
            brake(bool): use when duty cycle is zero (-> True)
        """
        
        # Limiting output
        if output > 1:
            output = 1
        elif output < -1:
            output = -1

        # Forward rotation
        if output > 0:
            self._enable1.on()
            self._enable2.off()
            self._pwm1.value = output
        # Backward rotation
        elif output < 0:
            self._enable1.off()
            self._enable2.on()
            self._pwm1.value = -output
        # Stop motor
        elif output == 0:
            if brake:
                self._enable1.off()
                self._enable2.off()
                self._pwm1.value = 0
            else:
                self._enable1.on()
                self._enable2.on()
                self._pwm1.value = 0
        # Updating output value property
        self._value = output


class PID:
    """
    The class to represent a discrete PID controller.

    Args: 
        Ts(float): The sampling period of the execution loop.
        kp(float): The PID proportional gain.
        ki(float): The PID integral gain.
        kd(float): The PID derivative gain.
        umax(float): The upper bound of the controller output saturation. (default = 1)
        umin(float): The lower bound of the controller output saturation. (default = -1)
        tau(float): The derivative term low-pass filter response time (s). (default = 0)
    """

    def __init__(self, Ts, kp, ki, kd, umax=1, umin=-1, tau=0):
        """
        Class constructor.
        """
        self._Ts = Ts  # Sampling period (s)
        self._kp = kp  # Proportional gain
        self._ki = ki  # Integral gain
        self._kd = kd  # Derivative gain
        self._umax = umax  # Upper output saturation limit
        self._umin = umin  # Lower output saturation limit
        self._tau = tau  # Derivative term filter time constant (s)
        #
        self._eprev = [0, 0]  # Previous errors e[n-1], e[n-2]
        self._uprev = 0  # Previous controller output u[n-1]
        self._udfiltprev = 0  # Previous filtered value

    def control(self, xsp, x, uff=0):
        """
        Calculate PID controller output.

        Args:
            xsp(float): The set point value at the time step.
            x(float): The actual value at the time step.
            uff(float): The feed-forward value at the time step. (default = 0)
        """
        # Calculating error
        e = xsp - x
        # Calculating proportional term
        up = self._kp * (e - self._eprev[0])
        # Calculating integral term (with anti-windup)
        ui = self._ki*self._Ts * e
        if (self._uprev+uff >= self._umax) or (self._uprev+uff <= self._umin):
            ui = 0
        # Calculating derivative term
        ud = self._kd/self._Ts * (e - 2*self._eprev[0] + self._eprev[1])
        # Filtering derivative term
        udfilt = (
            self._tau/(self._tau+self._Ts)*self._udfiltprev +
            self._Ts/(self._tau+self._Ts)*ud
        )
        # Calculating PID controller output
        u = self._uprev + up + ui + udfilt + uff
        # Updating previous time step errors
        self._eprev[1] = self._eprev[0]
        self._eprev[0] = e
        # Updating previous time step output value
        self._uprev = u - uff
        # Updating previous time step derivative term filtered value
        self._udfiltprev = udfilt
        # Limiting output (just to be safe)
        if u < self._umin:
            u = self._umin
        elif u > self._umax:
            u = self._umax
        # Returning controller output at current time step
        return u
