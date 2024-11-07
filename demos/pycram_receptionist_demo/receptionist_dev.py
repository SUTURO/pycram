import pycram.external_interfaces.giskard as giskardpy
from demos.pycram_serve_breakfast_demo.utils.misc import sort_objects
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import *
from pycram.process_module import real_robot, simulated_robot
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

world = BulletWorld(WorldMode.DIRECT)

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_5.urdf")

robot = Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=Pose([1, 2, 0]))

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.8, 2.8, 0.82]),
              color=Color(1, 0, 0, 1))

with real_robot:
    # TalkingMotion("Hey this is a test").perform()

    # MoveTorsoAction([0.25]).resolve().perform()

    # ParkArmsAction([Arms.LEFT]).resolve().perform()
    pose = Pose([5, 2.3, -0.2])
    LookAtAction([pose]).resolve().perform()

    x = DetectAction(technique='all').resolve().perform()
    # HeadFollowAction(state="start").resolve().perform()
    obj = sort_objects(robot, x, ["Mueslibox"])


    # PickUpAction(object_designator_description=obj[0], arms=[Arms.LEFT], grasps=[Grasp.FRONT]).resolve().perform()

    #MoveTCPMotion(target=target, arm=Arms.LEFT).perform()


