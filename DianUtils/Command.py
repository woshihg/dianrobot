from docutils.nodes import note

from DianUtils.ForwardController import *
from DianUtils.YawController import *
from DianUtils.SignalGenerator import *
from DianUtils.ArmPositionController import *
from DianUtils.ArmAngleController import *
from DianUtils.HeightController import *
from DianUtils.GripperController import *
from DianUtils.HeadPitchController import *


class TimeSpan:
    def __init__(self, begin: float = time.time(), duration_s: float = 0.7):
        self.begin = begin
        self.duration = duration_s

    def update(self, current_time: float) -> bool:
        passed = current_time > (self.begin + self.duration)
        # print("time span update: begin({}) duration({}) current({}) passed({})".format(
        #     self.begin, self.duration, current_time, passed
        #     )
        # )
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

class ForwardRobotCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "forward robot command"
        self.control_sender = ForwardControlSender()
        self.motor_receiver = ForwardMotorReceiver()

        self.forward_controller = ForwardController()
        self.signal_generator = SignalGenerator()
        self.signal_generator.set_interval(8)

        self.supervisor = TimeSpan()
        self.supervisor.duration = 2.0

    def execute(self, current_time: float):
        #print("executing command: ", self.description)

        # get ratio for controlling
        ratio = self.signal_generator.calc_signal(current_time)
        self.forward_controller.set_ratio(ratio)

        # controller._current = result


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

class MoveArmCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "move arm command"
        self.arm_controller = ArmPositionController()
        self.signal_generator = ConstSignalGenerator(0.01)
        self.control_sender = ArmControlSender()
        self.motor_receiver = ArmMotorReceiver()


    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.arm_controller.set_ratio(self.arm_controller.get_ratio())

        target_pos = self.arm_controller.calc_current_target_pos()
        motor_values = self.arm_controller.calc_motor_angles_by_target_pos(target_pos)

        self.control_sender.send(motor_values)
        self.arm_controller.set_current_pos(target_pos)

    def feedback(self):
        # get position from receiver
        angles = self.motor_receiver.receive()
        self.arm_controller.set_motor_angles(angles)


    def is_done(self) -> bool:
        done = self.arm_controller.check_is_done()
        return done

class MoveArmsCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "move both arms command"
        self.l_arm_controller = ArmPositionController()
        self.l_arm_controller.set_is_right(False)
        self.r_arm_controller = ArmPositionController()
        self.r_arm_controller.set_is_right(True)
        self.signal_generator = ConstSignalGenerator(0.01)
        self.control_sender = ArmControlSender()
        self.motor_receiver = ArmMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.l_arm_controller.set_ratio(self.l_arm_controller.get_ratio())
        self.r_arm_controller.set_ratio(self.l_arm_controller.get_ratio())

        l_target_pos = self.l_arm_controller.calc_current_target_pos()
        r_target_pos = self.r_arm_controller.calc_current_target_pos()
        l_motor_values = self.l_arm_controller.calc_motor_angles_by_target_pos(l_target_pos)
        r_motor_values = self.r_arm_controller.calc_motor_angles_by_target_pos(r_target_pos)
        # combine two motor values
        motor_values = np.concatenate((l_motor_values, r_motor_values))
        self.control_sender.send(motor_values)
        self.l_arm_controller.set_current_pos(l_target_pos)
        self.r_arm_controller.set_current_pos(r_target_pos)

    def feedback(self):
        #print("feedback command: ", self.description)

        # get position from receiver
        angles = self.motor_receiver.receive()
        self.l_arm_controller.set_motor_angles(angles[:6])
        self.r_arm_controller.set_motor_angles(angles[6:])
        #print("move robot command feedback current({})".format(position))

    def is_done(self) -> bool:
        done = self.l_arm_controller.check_is_done() and self.r_arm_controller.check_is_done()
        return done

class RotArmCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "rotate arm command"
        self.arm_controller = ArmAngleController()
        self.signal_generator = ConstSignalGenerator(0.01)
        self.control_sender = ArmControlSender()
        self.motor_receiver = ArmMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.arm_controller.set_ratio(self.arm_controller.get_ratio())

        target_rot = self.arm_controller.calc_current_target()
        self.control_sender.send(target_rot)

    def feedback(self):
        # get position from receiver
        angles = self.motor_receiver.receive()
        self.arm_controller.set_current(angles)

    def is_done(self) -> bool:
        done = self.arm_controller.check_is_done()
        return done

class RotArmsCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "rotate both arms command"
        self.l_arm_controller = ArmAngleController()
        self.r_arm_controller = ArmAngleController()
        self.signal_generator = ConstSignalGenerator(0.01)
        self.control_sender = ArmControlSender()
        self.motor_receiver = ArmMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.l_arm_controller.set_ratio(self.l_arm_controller.get_ratio())
        self.r_arm_controller.set_ratio(self.l_arm_controller.get_ratio())

        l_target_rot = self.l_arm_controller.calc_current_target()
        r_target_rot = self.r_arm_controller.calc_current_target()
        self.control_sender.send(np.concatenate((l_target_rot, r_target_rot)))
        self.l_arm_controller.set_current(l_target_rot)
        self.r_arm_controller.set_current(r_target_rot)

    def feedback(self):
        # get position from receiver
        angles = self.motor_receiver.receive()
        # self.l_arm_controller.set_current(angles[:6])
        # self.r_arm_controller.set_current(angles[6:])

    def is_done(self) -> bool:
        done = self.l_arm_controller.check_is_done() and self.r_arm_controller.check_is_done()
        return done

class RobotHeightCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "robot height command"
        self.height_controller = HeightController()
        self.signal_generator = ConstSignalGenerator(0.03)
        self.control_sender = HeightControlSender()
        self.motor_receiver = HeightMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.height_controller.set_ratio(self.height_controller.get_ratio())

        target_height = self.height_controller.calc_current_target()
        self.control_sender.send(target_height)
        self.height_controller.set_current(target_height)

    def feedback(self):
        # get position from receiver
        height = self.motor_receiver.receive()
        # self.height_controller.set_current(target_height)

    def is_done(self) -> bool:
        done = self.height_controller.check_is_done()
        return done

class RobotGripperCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "robot gripper command"
        self.gripper_controller = GripperController()
        self.signal_generator = ConstSignalGenerator(0.0001)
        self.control_sender = GripperControlSender()
        self.motor_receiver = GripperMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.gripper_controller.set_ratio(self.gripper_controller.get_ratio())

        target_open_size = self.gripper_controller.calc_current_target()
        self.control_sender.send(target_open_size)
        self.gripper_controller.set_current(target_open_size)

    def feedback(self):
        # get position from receiver
        open_size = self.motor_receiver.receive()
        # self.gripper_controller.set_current(open_size)

    def is_done(self) -> bool:
        done = self.gripper_controller.check_is_done()
        return done

class RobotGrippersCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "robot grippers command"
        self.l_gripper_controller = GripperController()
        self.r_gripper_controller = GripperController()
        self.signal_generator = ConstSignalGenerator(0.0001)
        self.control_sender = GripperControlSender()
        self.motor_receiver = GripperMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.l_gripper_controller.set_ratio(self.l_gripper_controller.get_ratio())
        self.r_gripper_controller.set_ratio(self.r_gripper_controller.get_ratio())

        l_target_open_size = self.l_gripper_controller.calc_current_target()
        r_target_open_size = self.r_gripper_controller.calc_current_target()
        self.control_sender.send(np.array([l_target_open_size, r_target_open_size]))
        self.l_gripper_controller.set_current(l_target_open_size)
        self.r_gripper_controller.set_current(r_target_open_size)

    def feedback(self):
        # get position from receiver
        open_size = self.motor_receiver.receive()
        # self.gripper_controller.set_current(open_size)

    def is_done(self) -> bool:
        done = self.l_gripper_controller.check_is_done() and self.r_gripper_controller.check_is_done()
        return done

class RobotHeadPitchCommand(Command):
    def __init__(self):
        super().__init__()
        self.description = "robot head pitch command"
        self.head_pitch_controller = HeadPitchController()
        self.signal_generator = ConstSignalGenerator(0.1)
        self.control_sender = HeadPitchControlSender()
        self.motor_receiver = HeadPitchMotorReceiver()

    def execute(self, current_time: float):
        ratio = self.signal_generator.calc_signal(current_time)
        self.head_pitch_controller.set_ratio(self.head_pitch_controller.get_ratio())

        target_pitch = self.head_pitch_controller.calc_current_target()
        self.control_sender.send(target_pitch)
        self.head_pitch_controller.set_current(target_pitch)

    def feedback(self):
        # get position from receiver
        pitch = self.motor_receiver.receive()
        # self.head_pitch_controller.set_current(target_pitch)

    def is_done(self) -> bool:
        done = self.head_pitch_controller.check_is_done()
        return done