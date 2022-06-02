"""
# asap_helper.py

Utility functions to support execute_asap.py.
"""


def create_namespaced_topics(topic_list, sim=False, bee_name=""):
    """ Return appropriately prefixed topics

    topic_list: ["list, "of", "topics"]
    sim: True to use "/" namespacing
    bee_name: must be given if sim=False, lowercase
    """
    namespaced_topics = []
    if sim is False:
        namespaced_topics = ["/" + topic for topic in topic_list]
    else:
        if bee_name == "":
            raise("Error, bee_name must be specified for sim topics!")
        else:
            namespaced_topics = ["/" + bee_name + "/" + topic for topic in topic_list]

    namespaced_topics = ' '.join(namespaced_topics)  # add space between each name for rosbag
    namespaced_topics = " " + namespaced_topics  # add leading space
    return namespaced_topics
