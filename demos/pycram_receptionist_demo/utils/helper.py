from typing import Optional

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

from pycram.designators.action_designator import *
from pycram.designators.motion_designator import PointingMotion
from pycram.designators.object_designator import HumanDescription
from pycram.failures import PerceptionObjectNotFound
look_couch = Pose([3.8, 0.3, 0.75])


def get_attributes(guest: HumanDescription, trys: Optional[int] = 0):
    """
    storing attributes and face of person in front of robot
    :param guest: variable to store information in
    :param trys: failure handling
    """
    MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
    MoveJointsMotion(["head_tilt_joint"], [0.45]).perform()
    TalkingMotion("i will take a picture of you to recognize you later").perform()
    rospy.sleep(2.4)
    TalkingMotion("please look at me").perform()
    rospy.sleep(1.5)
    # remember face
    while trys < 1:
        try:
            keys = DetectAction(technique='human', state='face').resolve().perform()
            new_id = keys["keys"][0]
            guest.set_id(new_id)

            # get clothes and gender
            attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
            print(attr_list)
            guest.set_attributes(attr_list)
            rospy.loginfo(attr_list)
            break

        except PerceptionObjectNotFound:
            # failure handling, if human has stepped away
            TalkingMotion("please step in front of me").perform()
            rospy.sleep(3.5)
            trys += 1

            try:
                get_attributes(guest, trys=trys)

            except PerceptionObjectNotFound:
                trys += 1
                print("continue without attributes")

        return guest


def detect_point_to_seat(robot, no_sofa: Optional[bool] = False):
    """
    function to look for a place to sit and poit to it
    returns bool if free place found or not
    """

    # detect free seat
    seat = DetectAction(technique='location', state="sofa").resolve().perform()
    free_seat = False
    x = 3.6
    # loop through all seating options detected by perception
    if not no_sofa:
        for place in seat:
            if place[1] == 'False':

                pose_in_map = Pose([float(place[2]), float(place[3]), 0.85])
                print("place: " + str(place))

                lt = LocalTransformer()
                pose_in_robot_frame = lt.transform_pose(pose_in_map, robot.get_link_tf_frame("base_link"))
                if pose_in_robot_frame.pose.position.y > 0.25:
                    TalkingMotion("please take a seat to the left from me").perform()
                    pose_in_robot_frame.pose.position.y += 0.4

                elif pose_in_robot_frame.pose.position.y < -0.35:
                    TalkingMotion("please take a seat to the right from me").perform()
                    pose_in_robot_frame.pose.position.y -= 0.4

                else:
                    TalkingMotion("please take a seat in front of me").perform()

                pose_in_map = lt.transform_pose(pose_in_robot_frame, "map")
                free_seat = True
                break
    else:
        print("only chairs")
        for place in seat[1]:
            if place[0] == 'chair':
                if place[1] == 'False':
                    pose_in_map = Pose([float(place[2]), float(place[3]), 0.85])
                    TalkingMotion("please take a seat on the free chair").perform()
                    free_seat = True
                    x = 2.5
                    break

    if free_seat:
        pose_guest = PointStamped()
        pose_guest.header.frame_id = "map"
        pose_guest.point.x = pose_in_map.pose.position.x
        pose_guest.point.y = pose_in_map.pose.position.y
        pose_guest.point.z = 0.85

        MoveGripperMotion(GripperState.CLOSE, Arms.LEFT).perform()
        PointingMotion(pose_guest).perform()

        print("found seat")
        return pose_guest
    else:
        TalkingMotion("no free seat detected").perform()

    return free_seat


def detect_host_face(host: HumanDescription):
    found_host = False
    try:

        LookAtAction([look_couch]).resolve().perform()
        human_dict = DetectAction(technique='human', state='face').resolve().perform()
        id_humans = human_dict["keys"]
        print("id humans: " + str(id_humans))
        host_pose = human_dict[id_humans[0]]
        host.set_id(id_humans[0])
        host.set_pose(PoseStamped_to_Point(host_pose))
        return True

    except PerceptionObjectNotFound:
        return False


def identify_faces(host: HumanDescription, guest1: HumanDescription):
    LookAtAction([look_couch]).resolve().perform()
    counter = 0
    found_guest = False
    found_host = False
    while True:
        unknown = []
        try:

            if counter > 4 or (found_guest and found_host):
                break

            elif counter == 2:
                TalkingMotion("please look at me").perform()
                rospy.sleep(2.5)

            elif counter == 3:
                MoveJointsMotion(["head_pan_joint"], [-0.3]).perform()
                TalkingMotion("please look at me").perform()
                rospy.sleep(2.5)

            human_dict = DetectAction(technique='human', state='face').resolve().perform()
            print("face detect: " + str(human_dict))
            counter += 1
            id_humans = human_dict["keys"]

            # loop through detected face Ids
            for key in id_humans:
                print("key: " + str(key))
                print("counter: " + str(counter))

                # found guest
                if key == guest1.id:
                    # update pose
                    guest1_pose = human_dict[guest1.id]
                    found_guest = True
                    guest1.set_pose(PoseStamped_to_Point(guest1_pose))

                # found host
                elif key == host.id:
                    # update pose
                    found_host = True
                    host_pose = human_dict[host.id]
                    host.set_pose(PoseStamped_to_Point(host_pose))
                else:
                    # store unknown ids for failure handling
                    unknown.append(key)

        except PerceptionObjectNotFound:
            counter += 1
            print(counter)
            if counter == 3:
                config = {'head_pan_joint': -0.2}

                # TalkingMotion("please look at me", ).perform()
                rospy.sleep(2.5)

    # Failure Handling if at least one person was not recognized
    if not found_guest and not found_host:
        try:
            # both have not been recognized
            guest1.set_pose(PoseStamped_to_Point(human_dict[unknown[0]]))
            host.set_pose(PoseStamped_to_Point(human_dict[unknown[1]]))
        except Exception as e:
            print(e)
    else:
        # either guest or host was not found
        if not found_guest:
            try:
                guest1.set_pose(PoseStamped_to_Point(human_dict[unknown[0]]))
            except Exception as e:
                print(e)
        elif not found_host:
            try:
                host.set_pose(PoseStamped_to_Point(human_dict[unknown[0]]))
            except Exception as e:
                print(e)


def introduce(human1: HumanDescription, human2: HumanDescription):
    """
    Text for robot to introduce two people to each other and alternate gaze
    :param human1: the first human the robot talks to
    :param human2: the second human the robot talks to
    """
    pub_pose = rospy.Publisher('/human_pose', PointStamped, queue_size=10)
    print("pose:" + str(human1.pose))
    rospy.sleep(2)
    if human1.pose:
        pub_pose.publish(human1.pose)
        rospy.sleep(1.0)
        pub_pose.publish(human1.pose)
    TalkingMotion(f"Hey, {human1.name}").perform()
    rospy.sleep(2.5)

    if human2.pose:
        pub_pose.publish(human2.pose)
        rospy.sleep(1)
    TalkingMotion(f" This is {human2.name} and their favorite drink is {human2.fav_drink}").perform()
    rospy.sleep(2.2)
    TalkingMotion(f"Hey, {human2.name}").perform()
    rospy.sleep(2)

    if human1.pose:
        pub_pose.publish(human1.pose)
        rospy.sleep(1.5)
    TalkingMotion(f" This is {human1.name} and their favorite drink is {human1.fav_drink}").perform()

    rospy.sleep(1)


def PoseStamped_to_Point(pose: PoseStamped):
    """
    function to transform PoseStamped to PointStamped in '/map' frame
    """
    point_pose = PointStamped()
    point_pose.header.frame_id = "map"
    point_pose.point.x = pose.pose.position.x
    point_pose.point.y = pose.pose.position.y
    point_pose.point.z = pose.pose.position.z

    return point_pose


def describe(human: HumanDescription):
    """
    HRI-function for describing a human more detailed.
    the following will be stated: gender, headgear, clothing, brightness of clothes
    :param human: human to be described
    """
    pub_pose2 = rospy.Publisher('/human_pose', PointStamped, queue_size=10)
    
    if human.attributes != "False" and human.attributes is not None:
        print(human.attributes)

        if human.pose:
            pub_pose2.publish(human.pose)

        TalkingMotion(f"I will describe {human.name} further now").perform()
        rospy.sleep(1.5)

        # gender
        TalkingMotion(f"i think your gender is {human.attributes[0]}").perform()
        rospy.sleep(1.5)

        # headgear or not
        TalkingMotion(f"you are not wearing a hat").perform()
        rospy.sleep(1)

        # kind of clothes
        TalkingMotion(f"you are  {human.attributes[2]}").perform()
        rospy.sleep(1)

        # brightness of clothes
        TalkingMotion(f"you are wearing {human.attributes[3]}").perform()
        rospy.sleep(2.5)
        TalkingMotion("have fun at the party").perform()




