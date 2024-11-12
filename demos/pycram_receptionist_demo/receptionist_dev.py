import rospy

from pycram.datastructures.dataclasses import Color
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.process_module import real_robot
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

world = BulletWorld(WorldMode.DIRECT)

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")

robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.8, 2.8, 0.82]),
              color=Color(1, 0, 0, 1))

with real_robot:
    TalkingMotion("Hey this is a test").perform()
    rospy.sleep(2)
    LookAtAction([Pose([2, 2.3, 0.1])]).resolve().perform()

    # MoveTorsoAction([0.25]).resolve().perform()

    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    DetectAction(technique="human").resolve().perform()
    #print("next code")
    #HeadFollowAction(state="start").resolve().perform()

    rospy.sleep(6)


    # PickUpAction(object_designator_description=obj[0], arms=[Arms.LEFT], grasps=[Grasp.FRONT]).resolve().perform()

    #MoveTCPMotion(target=target, arm=Arms.LEFT).perform()


