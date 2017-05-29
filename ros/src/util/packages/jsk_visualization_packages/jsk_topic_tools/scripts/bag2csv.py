#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import rosbag

from optparse import OptionParser
import StringIO
from datetime import datetime
import uuid


def message_to_csv(stream, msg, flatten=False):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(stream, val, flatten)
    except:
        msg_str = str(msg)
        if msg_str.find(",") is not -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        stream.write("," + msg_str)

def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name,s]))
    except:
        stream.write("," + parent_content_name)

seq = 0
nowtime = datetime.now().strftime("%Y%m%d-%H%M%S")

def format_csv_filename(form, topic_name):
    global seq
    if form==None:
        return "Convertedbag.csv"
    ret = form.replace('%t', topic_name.replace('/','-'))
    ret = ret.replace('%s', str(seq))
    seq += 1
    ret = ret.replace('%d', nowtime)
    ret = ret.replace('%u', str(uuid.uuid4()))
    return ret
        
def bag_to_csv(options, args):
    try:
        bag = rosbag.Bag(args[0])
        streamdict= dict()
        stime = None
        if options.start_time:
            stime = rospy.Time(options.start_time)
        etime = None
        if options.end_time:
            etime = rospy.Time(options.end_time)
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)

    try:
        for topic, msg, time in bag.read_messages(topics=options.topic_names,
                                                  start_time=stime,
                                                  end_time=etime):
            if streamdict.has_key(topic):
                stream = streamdict[topic]
            else:
                stream = open(format_csv_filename(options.output_file_format, topic),'w')
                streamdict[topic] = stream
                # header
                if options.header:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')

            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(stream, msg, flatten=not options.header)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()

if __name__ == '__main__':
    rospy.init_node('bag2csv', anonymous=True)
    parser = OptionParser(usage="%prog [options] bagfile")
    parser.add_option("-a", "--all", dest="all_topics",
                      action="store_true",
                      help="exports all topics", default=False)
    parser.add_option("-t", "--topic", dest="topic_names",
                      action="append",
                      help="white list topic names", metavar="TOPIC_NAME")
    parser.add_option("-O", "--output", dest="output_file_format",
                      help="output file names format\n%t: topic name\n%s: sequential number\n%d: datetime (now)\n%u: uuid\ne.g.: -O jskbag-$t-$d.csv", metavar="DESTINATION")
    parser.add_option("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type="float")
    parser.add_option("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type="float")
    parser.add_option("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header / flatten array value")
    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.print_help()
        exit(0)

    bag_to_csv(options, args)
