class HSRBDetectingReal(ProcessModule):
    """
    Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, desig: DetectingMotion) -> Any:
        # todo at the moment perception ignores searching for a specific object type so we do as well on real
        if desig.technique == 'human' and (desig.state == 'start' or desig.state == None):
            human_pose = query_human()
            return human_pose
        elif desig.state == "face":

            res = faces_query()
            id_dict = {}
            keys = []
            if res.res:
                for ele in res.res:
                    id_dict[int(ele.type)] = ele.pose[0]
                    keys.append(int(ele.type))
                id_dict["keys"] = keys
                return id_dict
            else:
                return []

        elif desig.state == "stop":
            stop_query()
            return "stopped"

        elif desig.technique == 'location':
            seat = desig.state
            seat_human_pose = query_specific_region(seat)

            if seat == "long_table" or seat == "popcorn_table":
                loc_list = []
                for loc in seat_human_pose[0].attribute:
                    print(f"location: {loc}, type: {type(loc)}")
                    loc_list.append(loc)
                print(loc_list)
                return loc_list
                # return seat_human_pose[0].attribute
            # if only one seat is checked
            # TODO check if still needed
            if seat != "sofa":
                return seat_human_pose[0].attribute[0][9:].split(',')
            # when whole sofa gets checked, a list of lists is returned
            res = []
            for i in seat_human_pose[0].attribute:
                res.append(i.split(','))

            print(res)
            return res

        elif desig.technique == 'attributes':
            human_pose_attr = query_human_attributes()
            counter = 0
            # wait for human to come
            while not human_pose_attr.res and counter < 6:
                human_pose_attr = query_human_attributes()
                counter += 1
                if counter > 3:
                    TalkingMotion("please step in front of me").resolve().perform()
                    rospy.sleep(2)

            if counter >= 3:
                return "False"

            # extract information from query
            gender = human_pose_attr.res[0].attribute[3][13:19]
            if gender[0] != 'f':
                gender = gender[:4]
            clothes = human_pose_attr.res[0].attribute[1][20:]
            brightness_clothes = human_pose_attr.res[0].attribute[0][5:]
            hat = human_pose_attr.res[0].attribute[2][20:]
            attr_list = [gender, hat, clothes, brightness_clothes]
            return attr_list

        # used when region-filter of robokudo should be used
        elif desig.technique == 'region':
            region = desig.state  # name of the region where should be perceived
            query_result = query_specific_region("region", region)
            perceived_objects = []

            for obj in query_result:
                # this has to be pose from pose stamped since we spawn the object with given header
                list = obj.pose
                if len(list) == 0:
                    continue
                obj_pose = Pose.from_pose_stamped(list[0])
                # obj_pose.orientation = [0, 0, 0, 1]
                # obj_pose_tmp = query_result.res[i].pose[0]
                obj_type = obj.type
                obj_size = obj.size
                # obj_color = query_result.res[i].color[0]

                # atm this is the string size that describes the object but it is not the shape size thats why string
                def extract_xyz_values(input_string):
                    # Split the input string by commas and colon to separate key-value pairs
                    # key_value_pairs = input_string.split(', ')

                    # Initialize variables to store the X, Y, and Z values
                    x_value = None
                    y_value = None
                    z_value = None

                    # todo: for now it is a string again, might be changed back. In this case we need the lower commented out code
                    xvalue = input_string[(input_string.find("x") + 2): input_string.find("y")]
                    y_value = input_string[(input_string.find("y") + 2): input_string.find("z")]
                    z_value = input_string[(input_string.find("z") + 2):]

                    # Iterate through the key-value pairs to extract the values
                    # for pair in key_value_pairs:
                    #     key, value = pair.split(': ')
                    #     if key == 'x':
                    #         x_value = float(value)
                    #     elif key == 'y':
                    #         y_value = float(value)
                    #     elif key == 'z':
                    #         z_value = float(value)

                    return x_value, y_value, z_value

                x, y, z = extract_xyz_values(obj_size)
                # size = (x, z / 2, y)
                # size_box = (x / 2, z / 2, y / 2)
                hard_size = (0.02, 0.02, 0.03)
                # TODO: add Bulletworld obj to fkt, replaced function add_ridig_box
                # gen_obj_desc = GenericObjectDescription("robokudo_object", [0, 0, 0], [0.1, 0.1, 0.1])
                # id = BulletWorld.load_generic_object_and_get_id(description=gen_obj_desc)
                # TODO: adjust right path, hardcoded for now
                # TODO: name is not unique anymore
                box_object = Object(obj_type, obj_type, pose=obj_pose, color=Color(0, 0, 0, 1), path="milk.stl")
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

                perceived_objects.append(box_desig)

            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(perceived_objects):
                object_dict[obj.name] = obj
            return object_dict

        else:
            query_result = send_query(ObjectDesignatorDescription(types=[ObjectType.MILK]))
            perceived_objects = []
            for i in range(0, len(query_result.res)):

                try:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                except IndexError:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose)
                    pass
                obj_type = query_result.res[i].type
                obj_size = None
                try:
                     obj_size = query_result.res[i].shape_size[0].dimensions
                except IndexError:
                    pass
                obj_color = None
                try:
                    obj_color = query_result.res[i].color[0]
                except IndexError:
                    pass

                # if desig.object_type:
                #     if not desig.object_type.lower() in obj_type.lower():
                #         pass

                color_switch = {
                    "red": Color(1, 0, 0, 1),
                    "yellow": Color(1, 1, 0, 1),
                    "green": Color(0, 1, 0, 1),
                    "cyan": Color(0, 1, 1, 1),
                    "blue": Color(0, 0, 1, 1),
                    "magenta": Color(1, 0, 1, 1),
                    "white": Color(1, 1, 1, 1),
                    "black": Color(0, 0, 0, 1),
                    "grey": Color(0.5, 0.5, 0.5, 1),
                    # add more colors if needed
                }

                color = color_switch.get(obj_color)

                if color is None:
                    color = Color(0, 0, 0, 1)


                hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]
                osize = [obj_size.x, obj_size.y, obj_size.z]
                # TODO: add Bulletworld obj to fkt, replaced function add_ridig_box
                # gen_obj_desc = GenericObjectDescription("robokudo_object", [0, 0, 0], [0.1, 0.1, 0.1])
                # id = BulletWorld.load_generic_object_and_get_id(description=gen_obj_desc)

                # TODO: adjust right path, hardcoded for now
                # TODO: name not unique anymore
                box_object = Object(obj_type, obj_type, pose=obj_pose, color=color, path="milk.stl")
                box_object.set_pose(obj_pose)
                box_desig = ObjectDesignatorDescription.Object(box_object.name, obj_type, box_object)

                perceived_objects.append(box_desig)

            # object_dict = {}
            #
            # for i, obj in enumerate(perceived_objects):
            #     print(obj.name)
            #     object_dict[obj.name] = obj
            return perceived_objects