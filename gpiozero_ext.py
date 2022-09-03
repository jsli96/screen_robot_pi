import time
import pigpio
from gpiozero import DigitalOutputDevice, PWMOutputDevice


class Motor:
    def __init__(
            self, pi=None, enable1=None, enable2=None, pwm1=None, pwm2=None,
            encoder1=None, encoder2=None, encoder_ppr=1666):
        if pi:
            self.pi = pi
        if pwm1 and pwm2:
            self._drv8833 = True
            self._pwm1 = PWMOutputDevice(pwm1)
            self._pwm2 = PWMOutputDevice(pwm2)
        else:
            raise Exception("Pin configuration is incorrect!")
        if encoder1 and encoder2 and pi:
            # self._encoder = RotaryEncoder(encoder1, encoder2, max_steps=0)
            self._encoder = Decoder(pi, encoder1, encoder2)
            self._ppr = encoder_ppr
        else:
            self._encoder = None
        self._value = 0
        self._angle0 = 0

    def __del__(self):
        if self._drv8833:
            self._pwm1.close()
            self._pwm2.close()
        if self._encoder:
            time.sleep(1)
            self._encoder.close()

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, _):
        print('"value" is a read only attribute.')

    def get_steps(self):
        if self._encoder:
            steps = self._encoder.steps_out()
        else:
            steps = None
        return steps

    def get_angle(self):
        if self._encoder:
            angle = 360 / self._ppr * self._encoder.steps_out() - self._angle0
        else:
            angle = None
        return angle

    def reset_angle(self):
        if self._encoder:
            self._angle0 = 360 / self._ppr * self._encoder.steps

    def set_output(self, output, brake=False):
        if output > 1:
            output = 1
        if output < -1:
            output = -1
        if output > 0:  # Forward
            if self._drv8833:
                self._pwm1.value = output
                self._pwm2.value = 0
        elif output < 0:  # Reverse
            if self._drv8833:
                self._pwm1.value = 0
                self._pwm2.value = -output
        else:
            if brake:
                if self._drv8833:
                    self._pwm1.value = 0
                    self._pwm2.value = 0
            else:
                if self._drv8833:
                    self._pwm1.value = 0
                    self._pwm2.value = 0
        self._value = output


class Decoder:
    def __init__(self, pi, gpioA, gpioB):
        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.levA = 0
        self.levB = 0
        self.steps = 0
        self.lastGpio = None

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.steps += -1
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.steps += 1

    def steps_out(self):
        return self.steps

    def close(self):
        self.cbA.cancel()
        self.cbB.cancel()


class PID:
    def __init__(self, Ts, kp, ki, kd, u_max=0.15, u_min=-0.15, tau=0):
        #
        self._Ts = Ts  # Sampling period (s)
        self._kp = kp  # Proportional gain
        self._ki = ki  # Integral gain
        self._kd = kd  # Derivative gain
        self._u_max = u_max  # Upper output saturation limit
        self._u_min = u_min  # Lower output saturation limit
        self._tau = tau  # Derivative term filter time constant (s)
        #
        self._ePrev = [0, 0]  # Previous errors e[n-1], e[n-2]
        self._uPrev = 0  # Previous controller output u[n-1]
        self._udfiltprev = 0  # Previous derivative term filtered value

    def control(self, ysp, y, uff=0):
        #
        # Calculating error e[n]
        e = ysp - y
        # Calculating proportional term
        up = self._kp * (e - self._ePrev[0])
        # Calculating integral term (with anti-windup)
        ui = self._ki * self._Ts * e
        if (self._uPrev + uff >= self._u_max) or (self._uPrev + uff <= self._u_min):
            ui = 0
        # Calculating derivative term
        ud = self._kd / self._Ts * (e - 2 * self._ePrev[0] + self._ePrev[1])
        # Filtering derivative term
        udfilt = (
                self._tau / (self._tau + self._Ts) * self._udfiltprev +
                self._Ts / (self._tau + self._Ts) * ud
        )
        # Calculating PID controller output u[n]
        u = self._uPrev + up + ui + udfilt + uff
        # Updating previous time step errors e[n-1], e[n-2]
        self._ePrev[1] = self._ePrev[0]
        self._ePrev[0] = e
        # Updating previous time step output value u[n-1]
        self._uPrev = u - uff
        # Updating previous time step derivative term filtered value
        self._udfiltprev = udfilt
        # Limiting output (just to be safe)
        if u < self._u_min:
            u = self._u_min
        elif u > self._u_max:
            u = self._u_max
        # Returning controller output at current time step
        return u
