from utime import ticks_ms, sleep_ms
from math import sqrt
from machine import Pin


class AccelStepper:
    def __init__(self, forward, backward):
        self._pins = 0
        self._currentPos = 0
        self._targetPos = 0
        self._speed = 0.0
        self._maxSpeed = 1.0
        self._acceleration = 1.0
        self._stepInterval = 0
        self._lastStepTime = 0
        self._pin1 = 0
        self._pin2 = 0
        self._pin3 = 0
        self._pin4 = 0
        self._forward = forward
        self._backward = backward

    def move_to(self, absolute):
        self._targetPos = absolute
        self.computeNewSpeed()

    def move(self, relative):
        self.moveTo(self._currentPos + relative)

    def runSpeed(self) -> bool:
        time = ticks_ms()
        if time > (self._lastStepTime + self._stepInterval):
            if self._speed > 0:
                self._currentPos += 1
            elif self._speed < 0:
                self._currentPos -= 1
            self.step()

    def distanceToGo(self) -> int:
        return self._targetPos - self._currentPos

    def targetPosition(self) -> int:
        return self._targetPos

    def currentPosition(self) -> int:
        return self._currentPos

    def setCurrentPosition(self, position):
        self._currentPos = position

    def computeNewSpeed(self):
        self.setSpeed(self.desiredSpeed())

    def desiredSpeed(self) -> float:
        distance_to_go = self.distanceToGo()
        required_speed = 0.0
        if distance_to_go == 0:
            return 0.0
        elif distance_to_go > 0:
            required_speed = sqrt(2.0 * distance_to_go * self._acceleration)
        else:
            required_speed = -sqrt(2.0 * distance_to_go * self._acceleration)

        if required_speed > self._speed:
            if self._speed == 0:
                required_speed = sqrt(2.0 * self._acceleration)
            else:
                required_speed = self._speed + abs(self._acceleration / self._speed)
            if required_speed > self._maxSpeed:
                required_speed = self._maxSpeed
        elif required_speed < self._speed:
            if self._speed == 0:
                required_speed = -sqrt(2.0 * self._acceleration)
            else:
                required_speed = self._speed - abs(self._acceleration / self._speed)
            if required_speed < -self._maxSpeed:
                required_speed = -self._maxSpeed
        return required_speed

    def run(self) -> bool:
        if self._targetPos == self._currentPos:
            return False
        if self.runSpeed():
            self.computeNewSpeed()
        return True

    def setMaxSpeed(self, speed: float):
        self._maxSpeed = speed
        self.computeNewSpeed()

    def setAcceleration(self, acceleration: float):
        self._acceleration = acceleration
        self.computeNewSpeed()

    def setSpeed(self, speed: float):
        self._speed = speed
        self._stepInterval = abs(1000.0 / self._speed)

    def speed(self) -> float:
        return self._speed

    def step(self, step):
        if step == 0:
            self.step0()
        elif step == 1:
            self.step1()
        elif step == 2:
            self.step2()
        else:
            self.step4()

    def step0(self):
        if self._speed > 0:
            self._forward()
        else:
            self._backward()

    def step1(self, step):
        self._pin2.value(self.speed() > 0)
        self._pin1.value(True)
        sleep_ms(1)
        self._pin1.value(False)

    def step2(self, step):
        if step == 0:
            self._pin1.value(False)
            self._pin2.value(True)
        elif step == 1:
            self._pin1.value(True)
            self._pin2.value(True)
        elif step == 2:
            self._pin1.value(True)
            self._pin2.value(False)
        else:
            self._pin1.value(False)
            self._pin2.value(False)

    def step4(self, step):
        if step == 0:
            self._pin1.value(True)
            self._pin2.value(False)
            self._pin3.value(True)
            self._pin4.value(False)
        elif step == 1:
            self._pin1.value(False)
            self._pin2.value(True)
            self._pin3.value(True)
            self._pin4.value(False)
        elif step == 2:
            self._pin1.value(False)
            self._pin2.value(True)
            self._pin3.value(False)
            self._pin4.value(True)
        else:
            self._pin1.value(True)
            self._pin2.value(False)
            self._pin3.value(False)
            self._pin4.value(True)

    def disableOutputs(self):
        if not self._pins:
            return
        self.pin1.value(False)
        self._pin2.value(False)
        if self._pins == 4:
            self._pin3.value(False)
            self._pin4.value(False)

    def enableOutputs(self):
        if not self._pins:
            return
        self._pin1 = Pin(self._pin1, Pin.OUT)
        self._pin2 = Pin(self._pin2, Pin.OUT)
        if self._pins == 4:
            self._pin3 = Pin(self._pin3, Pin.OUT)
            self._pin4 = Pin(self._pin4, Pin.OUT)

    def runToPosition(self):
        while self.run():
            pass

    def runSpeedToPosition(self) -> bool:
        return self.runSpeed() if self._targetPos != self._currentPos else False

    def runToNewPosition(self, position: int):
        self.move_to(position)
        self.runToPosition()
