from neem_interface_python import rosprolog_client

ros_client = rosprolog_client.Prolog()


def coole_knowledge_func():
    # punkt ist wichtig, sonst stoppt query nicht
    # '' ist unterschied zu "" !
    x = ros_client.once("entry_pose('kitchen', Y).")
    print(x)


coole_knowledge_func()
