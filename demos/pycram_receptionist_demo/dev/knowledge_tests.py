from neem_interface_python import rosprolog_client

ros_client = rosprolog_client.Prolog()

# Prolog Interface Ros - Service Client, blockiert und wartet

# Knowrob geschrieben in Prolog
# once (ein Ergebnis) und all solutions (alle Ergebnisse)
# String an Knowrob

# 'http://www.ease-crc.org/ont/SOMA.owl#Kitchen'


def coole_knowledge_func():
    # punkt ist wichtig, sonst stoppt query nicht
    # '' ist unterschied zu "" !
    ros_client.once("member(X,[1,2,3]).")
    x = ros_client.once("save_me_and_coffee('Vanessa').")
    z = ros_client.once("has_type(Room, 'http://www.ease-crc.org/ont/SUTURO.owl#LivingRoom').")
    y = ros_client.once("has_type(Room, 'http://www.ease-crc.org/ont/SOMA.owl#Kitchen'), entry_pose(Room, EntryPose), "
                        "exit_pose(Room, ExitPose).")

    print(y)


coole_knowledge_func()
print("done")
