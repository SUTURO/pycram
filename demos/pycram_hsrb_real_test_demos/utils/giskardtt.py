import rospy

from giskard_msgs.msg import GiskardError
from giskardpy.python_interface.old_python_interface import OldGiskardWrapper
from giskardpy.python_interface.python_interface import GiskardWrapper
from giskardpy.suturo_types import ForceTorqueThresholds, ObjectTypes
from pycram.designators.action_designator import fts
from pycram.language import Code, Monitor
from pycram.plan_failures import SensorMonitoringCondition
import pycram.external_interfaces.giskard_new as giskardpy

giskardpy.init_giskard_interface()

js1 = {
    # 'arm_flex_joint': -0.05,
    'arm_lift_joint': 0.05,
    'arm_roll_joint': -1.5,
    # 'head_pan_joint': 0.1016423434842566,
    # 'head_tilt_joint': 0.0255536193297764,
    'wrist_flex_joint': -1.3,
    'wrist_roll_joint': 1.7,
}

config_for_placing = {
    'arm_flex_joint': -1.1,
    'arm_lift_joint': 1.15,
    'arm_roll_joint': 0,
    # 'head_pan_joint': 0.1016423434842566,
    # 'head_tilt_joint': 0.0255536193297764,
    'wrist_flex_joint': -1.6,
    'wrist_roll_joint': 0,
}



js2 = {
    'arm_flex_joint': -1,
    'arm_lift_joint': 0.25,
    'arm_roll_joint': -1,
    # 'head_pan_joint': 0.1016423434842566,
    # 'head_tilt_joint': 0.0255536193297764,
    'wrist_flex_joint': 0,
    'wrist_roll_joint': 1.7,
}


# config_for_placing = {'arm_lift_joint': 0.13,
#                       'arm_flex_joint': -0.16,
#                       'arm_roll_joint': -0.0145,
#                       'wrist_flex_joint': -1.417,
#                       'wrist_roll_joint': 0.0}

#
# def monitor_func():
#     der = fts.get_last_value()
#     print(der.wrench.force.x)
#     if abs(der.wrench.force.x) < 1.5:
#         return SensorMonitoringCondition
#     return False


previous_value = None


def monitor_func():
    der = fts.get_last_value()
    current_value = fts.get_last_value()

    prev_force_x = previous_value.wrench.force.x
    curr_force_x = current_value.wrench.force.x
    change_in_force_x = abs(curr_force_x - prev_force_x)
    print(f"Current Force X: {curr_force_x}, Previous Force X: {prev_force_x}, Change: {change_in_force_x}")

    def calculate_dynamic_threshold(previous_force_x):
        # Placeholder for a dynamic threshold calculation based on previous values
        # This function can be enhanced to calculate a threshold based on the history of values or other logic
        return max(0.1 * abs(previous_force_x), 1.5)  # Example: 10% of the previous value or a minimum of 1.5

    if change_in_force_x >= calculate_dynamic_threshold(previous_force_x=prev_force_x):
        print("Significant change detected")

        return SensorMonitoringCondition

    return False

torso = {
    'arm_lift_joint': 0.0
}
# monitor_func()
previous_value = fts.get_last_value()
try:
    plan = Code(lambda: giskardpy.test(torso)) >> Monitor(monitor_func)
    plan.perform()
except Exception as e:
    print(f"Exception type: {type(e).__name__}")

print('done')
# giskard.motion_goals.add_joint_position(js1)
# giskard.add_default_end_motion_conditions()
# giskard.execute()
# print('done')
