import os
import sys
import rosbag

def get_rosbagtopics(file_path):
    baginfo = get_baginfo(file_path)
    return baginfo.keys()

def get_type_of_topics(file_path):
    baginfo = get_baginfo(file_path)
    types = []
    for index, info in enumerate(baginfo.values()):
        types.append(info[0])
    return types

def get_type_and_topic(file_path):
    type_and_topic_list = []
    baginfo = get_baginfo(file_path)
    for topic_type, topic in zip(baginfo.values(), baginfo.keys()):
        type_and_topic_list.append((topic_type[0], topic))
    return type_and_topic_list

def get_baginfo(file_path):
    if not os.path.exists(file_path):
        raise IOError("there are no file like %s" % file_path)
    elif file_path.find(".bag") <= 0:
        raise IOError("input should be bag file")
    else:
        bag = rosbag.Bag(file_path)
        return bag.get_type_and_topic_info()[1]

if __name__ == '__main__':
    try:
        file_path = sys.argv[1]
    except Exception, e:
        sys.exit("Please specify bag file. Example: get_rosbaginfo ex.bag")
    else:
        rostopic_list = get_rosbagtopics(file_path)
        topictype_list = get_type_of_topics(file_path)
        for rostopic, topic_type in zip(rostopic_list, topictype_list):
            print(rostopic, topic_type)
