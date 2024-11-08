import json

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
    print(f"response: {response}")
    print(f"response_type: {type(response)}")
    tmp_str = str(response).split("{'sentence': ")
    print(f"tmp_str: {tmp_str}")
    tmp_str_sen = tmp_str[1].split(',')
    print(f"tmp_str_sen: {tmp_str_sen}")
    print(tmp_str_sen[0])

    return tmp_str_sen[0].split(" ")