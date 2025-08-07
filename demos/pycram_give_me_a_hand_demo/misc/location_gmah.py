# semantic_map_entries = [
#     SemanticMapEntry(name="couch_table", frame_id="map", type="table",
#                      position_x=5.3, position_y=1.44, position_z=0.43,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0,
#                      orientation_w=1,
#                      x_size=0.45, y_size=0.9, z_size=0.55),
#
#     SemanticMapEntry(name="long_table", frame_id="map", type="table",
#                      position_x=3.36, position_y=4.87, position_z=0.74,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0.999,
#                      orientation_w=0.006,
#                      x_size=0.75, y_size=1.2, z_size=0.7),
#
#     SemanticMapEntry(name="popcorn_table_center", frame_id="map", type="table",
#                      position_x=2.08, position_y=4.98, position_z=0.71,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0.71,
#                      orientation_w=0.71,
#                      x_size=0.7, y_size=0.59, z_size=0.6),
#
#     SemanticMapEntry(name="popcorn_table_left", frame_id="map", type="table",
#                      position_x=1.49, position_y=4.98, position_z=0.71,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0.71,
#                      orientation_w=0.71,
#                      x_size=0.7, y_size=0.59, z_size=0.6),
#
#     SemanticMapEntry(name="popcorn_table_right", frame_id="map", type="table",
#                      position_x=2.67, position_y=4.98, position_z=0.71,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0.71,
#                      orientation_w=0.71,
#                      x_size=0.7, y_size=0.59, z_size=0.6),
#
#     SemanticMapEntry(name="seat1", frame_id="map", type="couch_seat",
#                      position_x=4.6, position_y=0.31, position_z=0.15,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0.71,
#                      orientation_w=0.71,
#                      x_size=0.76, y_size=0.8, z_size=0.8),
#
#     SemanticMapEntry(name="seat2", frame_id="map", type="couch_seat",
#                      position_x=3.76, position_y=0.31, position_z=0.15,
#                      orientation_x=0,
#                      orientation_y=0,
#                      orientation_z=0.71,
#                      orientation_w=0.71,
#                      x_size=0.76, y_size=0.8, z_size=0.8),
from std_msgs.msg import String

from pycram.datastructures.pose import Pose

current_locations = {
    'popcorn_table_left': 1.49,
    'popcorn_table_right': 2.67,
    'popcorn_table_center': 2.08,
    'long_table' : 3.36,
    'couch_table':5.3
}

intermediate_locations = {
    'popcorn_table_left': Pose([1.44, 2.6, 0], [0,0,0.71,0.71]),
    'popcorn_table_right': Pose([1.44, 2.6, 0], [0,0,0.71,0.71]),
    'popcorn_table_center': Pose([1.44, 2.6, 0], [0,0,0.71,0.71]),
    'long_table' : Pose([3.73, 2.68, 0], [0,0,0.999,0.006]),
    'couch_table': Pose([3.75, 1.37, 0], [0,0,0,1])
}
class GMAH_misc():

    def intermediate_location(location:String) -> Pose:
        """
        Returns the intermediate location of the given location.
        :param: location: The location
        :return: The intermediate location
        """
        intermediate = intermediate_locations.get(location)
        print(intermediate)
        return intermediate

    def associated_location_by_pose(pose:Pose)-> String:
        """
        Returns the associated location of the given pose4
        :param: pose: The pose
        :return: The associated location
        """
        location = [key for key , val in intermediate_locations.items() if val == pose.pose.position.x]
        print(location[0])
        return location[0]