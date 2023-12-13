import rospy
from std_msgs.msg import String
from tmc_msgs.msg import *


def talker(talk_string):

    """
    text to speech function
    :param talk_string: sentence that hsr will say
    :type talk_string: str
    """

    print("talker entry")
    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

    # fill message of type Voice with required data:
    texttospeech = Voice()
    # language 1 = english (0 = japanese)
    texttospeech.language = 1
    texttospeech.sentence = talk_string

    rospy.sleep(2)
    pub.publish(texttospeech)
    print('talker done')


def name_drink_talker(string_list):

    """
    function that introduces someone with name and their favorite drink
    :param string_list: list with two entrys, the first is the name the second the drink
    :type string_list: [str]
    """

    pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

    toyas_text = "Hey" + string_list[0] + "your favorite drink is " + string_list[
        1] + "nice to meet you."
    voice_msgs = Voice()
    # language 1 = english
    voice_msgs.language = 1
    voice_msgs.sentence = toyas_text

    rospy.sleep(2)
    pub.publish(voice_msgs)
