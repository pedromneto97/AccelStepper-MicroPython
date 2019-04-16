from math import sqrt, fabs

from machine import Pin
from utime import ticks_us, sleep_ms


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


DIRECTION_CCW = 0,  # Counter-Clockwise
DIRECTION_CW = 1  # Clockwise

FUNCTION = 0  # Use the functional interface, implementing your own driver functions (internal use only)
DRIVER = 1  # Stepper Driver, 2 driver pins required
FULL2WIRE = 2  # 2 wire stepper, 2 motor pins required
FULL3WIRE = 3  # 3 wire stepper, such as HDD spindle, 3 motor pins required
FULL4WIRE = 4  # 4 wire full stepper, 4 motor pins required
HALF3WIRE = 6  # 3 wire half stepper, such as HDD spindle, 3 motor pins required
HALF4WIRE = 8  # 4 wire half stepper, 4 motor pins required


class AccelStepper:
    def __init__(self, *args):
        self._currentPos = 0
        self._targetPos = 0
        self._speed = 0.0
        self._maxSpeed = 1.0
        self._acceleration = 0.0
        self._sqrt_twoa = 1.0
        self._stepInterval = 0
        self._minPulseWidth = 1
        self._enablePin = 0xff
        self._lastStepTime = 0
        self._pin = [0, 0, 0, 0]
        self._enableInverted = False
        self._n = 0
        self._c0 = 0.0
        self._cn = 0.0
        self._cmin = 1.0
        self._direction = DIRECTION_CCW
        self._pinInverted = [0, 0, 0, 0]

        if len(args) == 6:
            self._interface = args[0]
            self._pin[0] = args[1]
            self._pin[1] = args[2]
            self._pin[2] = args[3]
            self._pin[3] = args[4]
            if args[5]:
                self.enable_outputs()
        elif len(args) == 2:
            self._interface = 0
            self._forward = args[0]
            self._backward = args[1]

        self.set_acceleration(1)

    def move_to(self, absolute: int) -> None:
        if self._targetPos != absolute:
            self._targetPos = absolute
            self.compute_new_speed()

    def move(self, relative: int) -> None:
        self.move_to(self._currentPos + relative)

    def run_speed(self) -> bool:
        if not self._stepInterval:
            return False
        time = ticks_us()
        if (time - self._lastStepTime) >= self._stepInterval:
            if self._direction == DIRECTION_CW:
                self._currentPos += 1
            else:
                self._currentPos -= 1
            self.step(self._currentPos)
            self._lastStepTime = time
            return True
        else:
            return False

    def distance_to_go(self) -> int:
        return self._targetPos - self._currentPos

    def target_position(self) -> int:
        return self._targetPos

    def current_position(self) -> int:
        return self._currentPos

    def set_current_position(self, position: int) -> None:
        self._targetPos = self._currentPos = position
        self._n = 0
        self._stepInterval = 0
        self._speed = 0.0

    def compute_new_speed(self) -> None:
        distance_to = self.distance_to_go()
        steps_to_stop = int((self._speed * self._speed) / (2.0 * self._acceleration))
        if distance_to == 0 and steps_to_stop <= 1:
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            return
        if distance_to > 0:
            if self._n > 0:
                if (steps_to_stop >= distance_to) or self._direction == DIRECTION_CCW:
                    self._n = -steps_to_stop
            elif self._n < 0:
                if (steps_to_stop < distance_to) and self._direction == DIRECTION_CW:
                    self._n = -self._n
        elif distance_to < 0:
            if self._n > 0:
                if (steps_to_stop >= -distance_to) or self._direction == DIRECTION_CW:
                    self._n = -steps_to_stop
            elif self._n < 0:
                if (steps_to_stop < -distance_to) and self._direction == DIRECTION_CCW:
                    self._n = -self._n
        if self._n == 0:
            self._cn = self._c0
            self._direction = DIRECTION_CW if distance_to > 0 else DIRECTION_CCW
        else:
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1))
            self._cn = max(self._cn, self._cmin)
        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction == DIRECTION_CCW:
            self._speed = -self._speed

    def run(self) -> bool:
        if self.run_speed():
            self.compute_new_speed()
        return self._speed != 0.0 or self.distance_to_go() != 0

    def set_max_speed(self, speed: float) -> None:
        if speed < 0.0:
            speed = -speed
        if self._maxSpeed != speed:
            self._maxSpeed = speed
            self._cmin = 1000000.0 / speed
            if self._n > 0:
                self._n = int((self._speed * self._speed) / (2.0 * self._acceleration))
                self.compute_new_speed()

    def max_speed(self):
        return self._maxSpeed

    def set_acceleration(self, acceleration: float) -> None:
        if acceleration == 0.0:
            return
        if acceleration < 0.0:
            acceleration = -acceleration
        if self._acceleration != acceleration:
            self._n = self._n * (self._acceleration / acceleration)
            self._c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0
            self._acceleration = acceleration
            self.compute_new_speed()

    def set_speed(self, speed: float) -> None:
        if speed == self._speed:
            return
        speed = constrain(speed, -self._maxSpeed, self._maxSpeed)
        if speed == 0.0:
            self._stepInterval = 0
        else:
            self._stepInterval = fabs(1000000.0 / speed)
            self._direction = DIRECTION_CW if speed > 0.0 else DIRECTION_CCW
        self._speed = speed

    def speed(self) -> float:
        return self._speed

    def step(self, step: int) -> None:
        if self._interface is FUNCTION:
            self.step0(step)
        elif self._interface is DRIVER:
            self.step1(step)
        elif self._interface is FULL2WIRE:
            self.step2(step)
        elif self._interface is FULL3WIRE:
            self.step3(step)
        elif self._interface is FULL4WIRE:
            self.step4(step)
        elif self._interface is HALF3WIRE:
            self.step6(step)
        elif self._interface is HALF4WIRE:
            self.step8(step)

    def set_output_pins(self, mask: int) -> None:
        num_pins = 2
        if self._interface is FULL4WIRE or self._interface is HALF4WIRE:
            num_pins = 4
        elif self._interface is FULL3WIRE or self._interface is HALF3WIRE:
            num_pins = 3
        for i in range(num_pins):
            self._pin[i].value(True ^ self._pinInverted[i] if mask & (1 << i) else False ^ self._pinInverted[i])

    def step0(self, step: int) -> None:
        if self._speed > 0:
            self._forward()
        else:
            self._backward()

    def step1(self, step: int) -> None:
        self.set_output_pins(0b10 if self._direction else 0b00)
        self.set_output_pins(0b11 if self._direction else 0b01)
        sleep_ms(self._minPulseWidth)
        self.set_output_pins(0b10 if self._direction else 0b00)

    def step2(self, step: int) -> None:
        aux = step & 0x3
        if aux == 0:
            self.set_output_pins(0b10)
        elif aux == 1:
            self.set_output_pins(0b11)
        elif aux == 2:
            self.set_output_pins(0b01)
        elif aux == 3:
            self.set_output_pins(0b00)

    def step3(self, step: int) -> None:
        aux = step % 3
        if aux == 0:
            self.set_output_pins(0b100)
        elif aux == 1:
            self.set_output_pins(0b001)
        elif aux == 2:
            self.set_output_pins(0b010)

    def step4(self, step: int) -> None:
        aux = step & 0x3
        if aux == 0:
            self.set_output_pins(0b0101)
        elif aux == 1:
            self.set_output_pins(0b0110)
        elif aux == 2:
            self.set_output_pins(0b1010)
        elif aux == 3:
            self.set_output_pins(0b1001)

    def step6(self, step: int) -> None:
        aux = step % 6
        if aux == 0:
            self.set_output_pins(0b100)
        elif aux == 1:
            self.set_output_pins(0b101)
        elif aux == 2:
            self.set_output_pins(0b001)
        elif aux == 3:
            self.set_output_pins(0b011)
        elif aux == 4:
            self.set_output_pins(0b010)
        elif aux == 5:
            self.set_output_pins(0b110)

    def step8(self, step: int) -> None:
        aux = step & 0x7
        if aux == 0:
            self.set_output_pins(0b0001)
        elif aux == 1:
            self.set_output_pins(0b0101)
        elif aux == 2:
            self.set_output_pins(0b0100)
        elif aux == 3:
            self.set_output_pins(0b0110)
        elif aux == 4:
            self.set_output_pins(0b0010)
        elif aux == 5:
            self.set_output_pins(0b1010)
        elif aux == 6:
            self.set_output_pins(0b1000)
        elif aux == 7:
            self.set_output_pins(0b1001)

    def disable_outputs(self) -> None:
        if not self._interface:
            return
        self.set_output_pins(0)
        if self._enablePin != 0xff:
            self._enablePin = Pin(self._enablePin, Pin.OUT)
            self._enablePin.value(False ^ self._enableInverted)

    def enable_outputs(self) -> None:
        if not self._interface:
            return
        self._pin[0] = Pin(self._pin[0], Pin.OUT)
        self._pin[1] = Pin(self._pin[1], Pin.OUT)
        if self._interface is FULL4WIRE or self._interface is HALF4WIRE:
            self._pin[2] = Pin(self._pin[2], Pin.OUT)
            self._pin[3] = Pin(self._pin[3], Pin.OUT)
        elif self._interface is FULL3WIRE or self._interface is HALF3WIRE:
            self._pin[2] = Pin(self._pin[2], Pin.OUT)
        if self._enablePin != 0xff:
            self._enablePin = Pin(self._enablePin, Pin.OUT)
            self._enablePin.value(True ^ self._enableInverted)

    def set_min_pulse_width(self, min_width: int) -> None:
        self._minPulseWidth = min_width

    def set_enable_pin(self, enable_pin: int) -> None:
        self._enablePin = enable_pin
        if self._enablePin != 0xff:
            self._enablePin = Pin(self._enablePin, Pin.OUT)
            self._enablePin.value(True ^ self._enableInverted)

    def set_pins_inverted(self, *args) -> None:
        if len(args) == 3:
            self.set_2_pins(args[0], args[1], args[2])
        elif len(args) == 5:
            self.set_4_pins(args[0], args[1], args[2], args[3], args[4])

    def set_2_pins(self, direction_invert: bool, step_invert: bool, enable_invert: bool) -> None:
        self._pinInverted[0] = step_invert
        self._pinInverted[1] = direction_invert
        self._enableInverted = enable_invert

    def set_4_pins(self, pin_1_invert: bool, pin_2_invert: bool, pin_3_invert: bool, pin_4_invert: bool,
                   enable_invert: bool) -> None:
        self._pinInverted[0] = pin_1_invert
        self._pinInverted[1] = pin_2_invert
        self._pinInverted[2] = pin_3_invert
        self._pinInverted[3] = pin_4_invert
        self._enableInverted = enable_invert

    def run_to_position(self) -> None:
        while self.run():
            pass

    def run_speed_to_position(self) -> bool:
        if self._targetPos == self._currentPos:
            return False
        if self._targetPos > self._currentPos:
            self._direction = DIRECTION_CW
        else:
            self._direction = DIRECTION_CCW
        return self.run_speed()

    def run_to_new_position(self, position: int) -> None:
        self.move_to(position)
        self.run_to_position()

    def stop(self) -> None:
        if self._speed != 0.0:
            steps_to_stop = int((self._speed * self._speed) / (2.0 * self._acceleration)) + 1
            if self._speed > 0:
                self.move(steps_to_stop)
            else:
                self.move(-steps_to_stop)

    def is_running(self) -> bool:
        return not (self._speed == 0 and self._targetPos == self._currentPos)
