import rospy
from std_msgs.msg import String

from pycram.language import Monitor, Code
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def get_desired_objects(response):
    """
    Filters the desired object, its color and the place where Toya should search for it.
    :param response: The response from NLP.
    :return: The desired object.
    """

    tmp = response['item']
    tmp_color = tmp['propertyAttribute']
    tmp_obj = tmp['value']

    return (tmp_obj, tmp_color)