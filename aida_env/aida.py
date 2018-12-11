import pybullet_data
import time
import  pybullet_envs
import numpy as np
import math
import itertools
from aida_env import motor
import os

def getDataPath():
    resdir = os.path.join(os.path.dirname(__file__))
    return resdir


#print(os.getcwd())
FRONT_LEFT = 0
REAR_RIGHT = 1
FRONT_RIGHT = 2
REAR_LEFT = 3
LEGS_NUM = [FRONT_LEFT, REAR_RIGHT, FRONT_RIGHT, REAR_LEFT]

LEGS_TO_MOTOR_NAMES = [
    ['left_front_calf_hinge', 'left_front_thigh_hinge','left_front_shoulder_hinge'],
    ['right_rear_calf_hinge', 'right_rear_thigh_hinge', 'right_rear_shoulder_hinge'],
    ['right_front_calf_hinge', 'right_front_thigh_hinge','right_front_shoulder_hinge'],
    ['left_rear_calf_hinge', 'left_rear_thigh_hinge', 'left_rear_shoulder_hinge']]
LIMITS_CALFS = (math.degrees(0.2), math.degrees(2.2))
LIMITS_THIGH = (math.degrees(-0.8), math.degrees(1.0))
LIMITS_SHOULDER = (math.degrees(-0.6), math.degrees(0.2))

LIMITS_MINS = np.array([LIMITS_CALFS[0], LIMITS_THIGH[0], LIMITS_SHOULDER[1]])
LIMITS_SPAND = np.array([e - s for (s, e) in [LIMITS_CALFS, LIMITS_THIGH, LIMITS_SHOULDER]])

def leg_scale(angles_in_01):
    """ multiply inputs (between 0 and 1 to be in the angle scale)"""
    return np.array(angles_in_01) * LIMITS_SPAND * 0.5  + LIMITS_MINS + LIMITS_SPAND / 2

def command_scale(angles):
    return np.concatenate([leg_scale(angles[i:i+3]) for i in [0, 3, 6, 9]])

def nice_angle(a):
    return ((a + 180) % (360.0)) - 180.0

def kinematic_to_model(angles):
    angles[0] = 180 - angles[0]
    angles[1] -= 90
    angles = [nice_angle(a) for a in angles]
    return angles 

def model_to_kinematics(angles):
    angles[0] -180 - angles[0]
    angles[1] += 90
    angles = [nice_angle(a) for a in angles]
    return angles 


class Aida:
    def __init__(
        self,
        pybullet_client,
        time_step=0.01,
        motor_velocity_limit=np.inf,
        pd_control_enabled=False,
        accurate_motor_model_enabled=False,
        motor_kp=0.01,
        motor_kd=1.0,
        torque_control_enabled=False,
        motor_overheat_protection=False,
        on_rack=False,
        kd_for_pd_controllers=0.3):
        """Constructs Aida and reset her to the initial states.
        Args:
        pybullet_client: The instance of BulletClient to manage different
            simulations.
        time_step: The time step of the simulation.
        motor_velocity_limit: The upper limit of the motor velocity.
        pd_control_enabled: Whether to use PD control for the motors.
        accurate_motor_model_enabled: Whether to use the accurate DC motor model.
        motor_kp: proportional gain for the accurate motor model
        motor_kd: derivative gain for the acurate motor model
        torque_control_enabled: Whether to use the torque control, if set to
            False, pose control will be used.
        motor_overheat_protection: Whether to shutdown the motor that has exerted
            large torque (OVERHEAT_SHUTDOWN_TORQUE) for an extended amount of time
            (OVERHEAT_SHUTDOWN_TIME). See ApplyAction() in aida.py for more
            details.
        on_rack: Whether to place the aida on rack. This is only used to debug
            the walking gait. In this mode, the  aida's base is hanged midair so
            that its walking gait is clearer to visualize.
        kd_for_pd_controllers: kd value for the pd controllers of the motors.
        """
    
 
        self._kp = motor_kp # motor gains 
        self._kd = motor_kd
        self._max_force = 40 # motor max_torque
        self._pybullet_client = pybullet_client
        self.time_step = time_step
        self.num_motors = 12
        self.num_legs = int(self.num_motors / 3)
        self._pybullet_client = pybullet_client
        self._motor_velocity_limit = motor_velocity_limit
        self._pd_control_enabled = pd_control_enabled
        self._motor_direction = [-1 if i in [FRONT_RIGHT * 3 + 2 , REAR_RIGHT * 3 + 2] else 1 for i in range(12)]
        self._observed_motor_torques = np.zeros(self.num_motors)
        self._applied_motor_torques = np.zeros(self.num_motors)
        self._accurate_motor_model_enabled = accurate_motor_model_enabled
        self._torque_control_enabled = torque_control_enabled
        self._motor_overheat_protection = motor_overheat_protection
        self._on_rack = on_rack
        if self._accurate_motor_model_enabled:
            self._kp = motor_kp
            self._kd = motor_kd
            self._motor_model = motor.MotorModel(
                torque_control_enabled=self._torque_control_enabled,
                kp=self._kp,
                kd=self._kd)
        elif self._pd_control_enabled:
            self._kp = 8
            self._kd = kd_for_pd_controllers
        else:
            self._kp = 1.0
            self._kd = 1.0
        self.time_step = time_step
        print (self._kp, self._kd)
        self.Reset()
  

    def load_urdf(self):
        start_pose = [0,0,0.8]
        start_orientation = self._pybullet_client.getQuaternionFromEuler([0,0,0])
        self.quadruped = self._pybullet_client.loadURDF(
            getDataPath() + "/urdf/aida.urdf",
            start_pose,
            start_orientation)
        self.load_motor_links()

    def Reset(self, reload_urdf=True):
        """Reset aida to its initial states.
        Args:
        reload_urdf: Whether to reload the urdf file. If not, Reset() just place
            the  aida back to its starting position.
        """
        if reload_urdf:
            self.load_urdf()
            self.reset_all_legs()
            if self._on_rack:
                self._pybullet_client.createConstraint(
                    self.quadruped, -1, -1, -1, self._pybullet_client.JOINT_FIXED,
                    [0, 0, 0], [0, 0, 0], [0, 0, 1])
        else:
            self._pybullet_client.resetBasePositionAndOrientation(
                self.quadruped, INIT_POSITION, INIT_ORIENTATION)
            self._pybullet_client.resetBaseVelocity(self.quadruped, [0, 0, 0],
                                                    [0, 0, 0])
            self.reset_all_legs()
        self._overheat_counter = np.zeros(self.num_motors)
        self._motor_enabled_list = [True] * self.num_motors


    
    def load_motor_links(self):

        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        list_of_joints_info = [self._pybullet_client.getJointInfo(self.quadruped, i) for i in range(num_joints)]


        list_of_joints_info_dict = {item[1] : [item[0]] + list(item[1:]) for item in list_of_joints_info}
        list_of_joints_info_dict.keys()

        self.motor_joints_info = dict((k, v) for k, v  in list_of_joints_info_dict.items() if k[-5:] == "hinge" and "foot" not in k)

        self.num_motors = len(self.motor_joints_info.keys())
        #self.joint_name_to_id = dict((k, v[0]) for k, v in self.motor_joints_info.items())
        self.joint_name_to_id = {'right_front_thigh_hinge': 14, 'left_rear_shoulder_hinge': 7, 'right_rear_thigh_hinge': 20, 'left_rear_calf_hinge': 10, 'right_rear_shoulder_hinge': 19, 'left_front_thigh_hinge': 2, 'right_rear_calf_hinge': 22, 'right_front_shoulder_hinge': 13, 'left_rear_thigh_hinge': 8, 'right_front_calf_hinge': 16, 'left_front_shoulder_hinge': 1, 'left_front_calf_hinge': 4}
        
        #self._motor_id_list = [self.joint_name_to_id[c] for c in list(itertools.chain.from_iterable(LEGS_TO_MOTOR_NAMES))]
        self._motor_id_list = [4,2,1,22,20,19,16,14,13,10,8,7]
    def _SetMotorTorqueById(self, motor_id, torque):
        self._pybullet_client.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=motor_id,
            controlMode=self._pybullet_client.TORQUE_CONTROL,
            force=torque)

    def _SetDesiredMotorAngleById(self, motor_id, desired_angle):
        #print desired_angle, "kp :", self._kp,  "kd", self._kd
        self._pybullet_client.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=motor_id,
            controlMode=self._pybullet_client.POSITION_CONTROL,
            targetPosition=math.radians(desired_angle),
            positionGain=self._kp,
            velocityGain=self._kd,
            force=self._max_force)
    
    def get_default_angles(self, leg_id=FRONT_LEFT):
        sign = 1.0
        if leg_id in [FRONT_RIGHT, REAR_RIGHT]:
            sign = -1.0
        default_angles = [79.19, 39.59, 11.3]
        default_angles[2] = default_angles[2] * sign
        return kinematic_to_model(default_angles)
        #return [50, 50, 0.5]

    def get_default_action(self):
        actions = [self.get_default_angles(leg_id) for leg_id in LEGS_NUM]
        return list(itertools.chain.from_iterable(actions))
        
        
    def reset_pose_for_leg(self, leg_id, default_angles=[0,0,0]):
        motors_ids = LEGS_TO_MOTOR_NAMES[leg_id]
        for i, m in enumerate(motors_ids):
            self._SetDesiredMotorAngleById(
                self.joint_name_to_id[m],
                default_angles[i])
        
    def reset_all_legs(self):
        for i in range(4):
            self.reset_pose_for_leg(
                i,
                default_angles=self.get_default_angles(i))
        
    def step(self):
        self._pybullet_client.stepSimulation()
        return self.GetObservation()

    def GetBasePosition(self):
        """Get the position of  aida's base.
        Returns:
          The position of  aida's base.
        """
        position, _ = (
            self._pybullet_client.getBasePositionAndOrientation(self.quadruped))
        return position

    def GetBaseLVelocity(self):
        """Get the linear velocity of  aida's base.
        Returns:
          The linear (x,y,z) velocity of  aida's base.
        """
        velocity, _ = (
            self._pybullet_client.getBaseVelocity(self.quadruped))
        return velocity

    def GetBaseAVelocity(self):
        """Get the angular velocity of  aida's base.
        Returns:
          The angular velocity of  aida's base.
        """
        _, velocity = (
            self._pybullet_client.getBaseVelocity(self.quadruped))
        return velocity

    def GetBaseOrientation(self):
        """Get the orientation of  aida's base, represented as quaternion.
        Returns:
          The orientation of  aida's base.
        """
        _, orientation = (
            self._pybullet_client.getBasePositionAndOrientation(self.quadruped))
        return orientation

    def GetActionDimension(self):
        """Get the length of the action list.
        Returns:
          The length of the action list.
        """
        return self.num_motors

    def GetObservationUpperBound(self):
        """Get the upper bound of the observation.
        Returns:
          The upper bound of an observation. See GetObservation() for the details
            of each element of an observation.
        """
        upper_bound = np.array([0.0] * self.GetObservationDimension())
        upper_bound[0:self.num_motors] = math.pi  # Joint angle.
        upper_bound[self.num_motors:2 * self.num_motors] = (
            motor.MOTOR_SPEED_LIMIT)  # Joint velocity.
        upper_bound[2 * self.num_motors:3 * self.num_motors] = (
            motor.OBSERVED_TORQUE_LIMIT)  # Joint torque.
        upper_bound[3 * self.num_motors:] = 1.0  # Quaternion of base orientation.
        return upper_bound

    def GetObservationLowerBound(self):
        """Get the lower bound of the observation."""
        return -self.GetObservationUpperBound()

    def GetObservationDimension(self):
        """Get the length of the observation list.
        Returns:
          The length of the observation list.
        """
        return len(self.GetObservation())

    def GetObservation(self):
        """Get the observations of  aida.
        It includes the angles, velocities, torques and the orientation of the base.
        Returns:
          The observation list. observation[0:8] are motor angles. observation[8:16]
          are motor velocities, observation[16:24] are motor torques.
          observation[24:28] is the orientation of the base, in quaternion form.
        """
        observation = []
        observation.extend(self.GetMotorAngles().tolist())
        observation.extend(self.GetMotorVelocities().tolist())
        observation.extend(self.GetMotorTorques().tolist())
        observation.extend(list(self.GetBaseOrientation()))
        return observation


    def ApplyAction(self, motor_commands):
        """Set the desired motor angles to the motors of the aida.
        The desired motor angles are clipped based on the maximum allowed velocity.
        If the pd_control_enabled is True, a torque is calculated according to
        the difference between current and desired joint angle, as well as the joint
        velocity. This torque is exerted to the motor. For more information about
        PD control, please refer to: https://en.wikipedia.org/wiki/PID_controller.
        Args:
        motor_commands: The 12 desired motor angles.
        """
        scaled_motor_commands = command_scale(motor_commands)
        scaled_motor_commands_with_directions = scaled_motor_commands * self._motor_direction 
        for motor_id, cmd in zip(
                self._motor_id_list, scaled_motor_commands_with_directions):
            self._SetDesiredMotorAngleById(motor_id, cmd)


    
    def GetMotorAngles(self):
        """Get the eight motor angles at the current moment.
        Returns:
          Motor angles.
        """
        motor_angles = [
            self._pybullet_client.getJointState(self.quadruped, motor_id)[0]
            for motor_id in self._motor_id_list
        ]
        motor_angles = np.multiply(motor_angles, self._motor_direction)
        return motor_angles

    def GetMotorVelocities(self):
        """Get the velocity of all eight motors .
        Returns:
          Velocities of all eight motors.
        """
        motor_velocities = [
            self._pybullet_client.getJointState(self.quadruped, motor_id)[1]
            for motor_id in self._motor_id_list
        ]
        motor_velocities = np.multiply(motor_velocities, self._motor_direction)
        return motor_velocities

    def GetMotorTorques(self):
        """Get the amount of torques the motors are exerting.
        Returns:
          Motor torques of all eight motors.
        """
        
        motor_torques = [
          self._pybullet_client.getJointState(self.quadruped, motor_id)[3]
          for motor_id in self._motor_id_list
        ]
        motor_torques = np.multiply(motor_torques, self._motor_direction)
        return motor_torques
