from docutils.nodes import note

from DianUtils.ForwardController import *
from DianUtils.YawController import *
from DianUtils.SignalGenerator import *

class TimeSpan:
    def __init__(self, begin: float = time.time(), duration_s: float = 0.7):
        self.begin = begin
        self.duration = duration_s

    def update(self, current_time: float) -> bool:
        passed = current_time > (self.begin + self.duration)
        print("time span update: begin({}) duration({}) current({}) passed({})".format(
            self.begin, self.duration, current_time, passed
            )
        )
        return passed

    def reset(self, current_time: float):
        self.begin = current_time


class Command:
    def __init__(self):
        self.description = "command description"

    def execute(self, current_time: float):
        pass

    def feedback(self):
        pass

    def is_done(self) -> bool:
        pass


class TurnRobotCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "turn robot command"
        self.yaw_controller = YawController()
        self.signal_generator = SignalGenerator()
        self.control_sender = YawControlSender()
        self.motor_receiver = YawMotorReceiver()
        self.supervisor = TimeSpan()

    def execute(self, current_time: float):
        #print("executing command: ", self.description)

        # get ratio for controlling
        ratio = self.signal_generator.calc_signal(current_time)
        self.yaw_controller.set_ratio(ratio)

        # apply control
        motor_value = self.yaw_controller.calc_current_target()
        self.control_sender.send(motor_value)
        #print("turn robot command from({}) to({}) value({})".format(
        #    self.yaw_controller.get_current(),
        #    self.yaw_controller.get_target(),
        #    motor_value)
        #)

    def feedback(self):
        #print("feedback command: ", self.description)

        # get angle from receiver
        angle = self.motor_receiver.receive()
        self.yaw_controller.set_current(angle)
        #print("turn robot command feedback current({})".format(angle))

    def is_done(self) -> bool:
        #print('checking command: ', self.description)
        done = self.yaw_controller.check_is_done()
        if not done:
            self.supervisor.reset(time.time())
            return False

        self.control_sender.send(0)
        return self.supervisor.update(time.time())

class MoveRobotCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "move robot command"
        self.forward_controller = ForwardController()
        self.signal_generator = SignalGenerator()
        self.control_sender = ForwardControlSender()
        self.motor_receiver = ForwardMotorReceiver()
        self.supervisor = TimeSpan()

    def execute(self, current_time: float):
        #print("executing command: ", self.description)

        # get ratio for controlling
        ratio = self.signal_generator.calc_signal(current_time)
        self.forward_controller.set_ratio(ratio)

        # apply control
        motor_value = self.forward_controller.calc_current_target()
        self.control_sender.send(motor_value)
        #print("move robot command from({}) to({}) value({})".format(
        #    self.forward_controller.get_current(),
        #    self.forward_controller.get_target(),
        #    motor_value)
        #)

    def feedback(self):
        #print("feedback command: ", self.description)

        # get position from receiver
        position = self.motor_receiver.receive()
        self.forward_controller.set_current(position)
        #print("move robot command feedback current({})".format(position))

    def is_done(self) -> bool:
        #print('checking command: ', self.description)
        done = self.forward_controller.check_is_done()
        if not done:
            self.supervisor.reset(time.time())
            return False

        self.control_sender.send(0)
        return self.supervisor.update(time.time())