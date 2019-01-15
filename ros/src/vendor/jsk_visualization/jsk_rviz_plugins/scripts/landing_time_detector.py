#!/usr/bin/env python

"""
Display the difference between reference and actual swing/support time according to hrpsys_ros_bridge/ContactState using
jsk_rviz_plugins/PictogramArray
"""

import rospy
from jsk_rviz_plugins.msg import Pictogram, PictogramArray
from geometry_msgs.msg import Quaternion
from hrpsys_ros_bridge.msg import ContactState, ContactStateStamped, ContactStatesStamped
from std_msgs.msg  import ColorRGBA
import message_filters

def callback(ref, act):
    "msgs = ContactStatesStamped"
    if len(ref_contact_states_queue) > buffer_size - 1:
        arr = PictogramArray()
        arr.header.frame_id = "/odom"
        arr.header.stamp = rospy.Time.now()
        for i, (ref_st, act_st) in enumerate(zip(ref.states, act.states)):
            picto = Pictogram()
            if ref_st.state.state == act_st.state.state:
                continue
            else:
                if [ref_st.state.state, act_st.state.state] == [ContactState.OFF, ContactState.ON]:
                    if [x.states[i].state.state for x in ref_contact_states_queue] == [ContactState.OFF] * buffer_size and [x.states[i].state.state for x in act_contact_states_queue] == [ContactState.OFF] * buffer_size:
                        picto.character = "fa-long-arrow-down"
                        picto.size = 1
                        picto.pose.orientation = Quaternion(0, -1, 0, 1)
                        picto.action = Pictogram.ADD
                        picto.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
                        rospy.loginfo("%s early landing %s [s]", ref_st.header.frame_id, str(ref_st.state.remaining_time))
                    elif [x.states[i].state.state for x in ref_contact_states_queue] == [ContactState.ON] * buffer_size and [x.states[i].state.state for x in act_contact_states_queue] == [ContactState.ON] * buffer_size:
                        picto.character = "up-bold"
                        picto.size = 1
                        picto.pose.orientation = Quaternion(0, -1, 0, 1)
                        picto.action = Pictogram.ADD
                        picto.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
                        rospy.loginfo("%s late taking off", ref_st.header.frame_id)
                        print "oso hanare"
                    else:
                        continue
                elif [ref_st.state.state, act_st.state.state] == [ContactState.ON, ContactState.OFF]:
                    if [x.states[i].state.state for x in ref_contact_states_queue] == [ContactState.OFF] * buffer_size and [x.states[i].state.state for x in act_contact_states_queue] == [ContactState.OFF] * buffer_size:
                        picto.character = "fa-long-arrow-down"
                        picto.size = 1
                        picto.pose.orientation = Quaternion(0, -1, 0, 1)
                        picto.action = Pictogram.ADD
                        picto.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
                        rospy.loginfo("%s late landing", ref_st.header.frame_id)
                    elif [x.states[i].state.state for x in ref_contact_states_queue] == [ContactState.ON] * buffer_size and [x.states[i].state.state for x in act_contact_states_queue] == [ContactState.ON] * buffer_size:
                        picto.character = "up-bold"
                        picto.size = 1
                        picto.pose.orientation = Quaternion(0, -1, 0, 1)
                        picto.action = Pictogram.ADD
                        picto.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
                        rospy.loginfo("%s early taking off %s [s]", ref_st.header.frame_id, str(ref_st.state.remaining_time))
                    else:
                        continue
            picto.header.frame_id = ref_st.header.frame_id
            picto.header.stamp = ref_st.header.stamp
            arr.pictograms.append(picto)
        if len(arr.pictograms) > 0:
            pub.publish(arr)
        ref_contact_states_queue.pop(0)
        act_contact_states_queue.pop(0)
    ref_contact_states_queue.append(ref)
    act_contact_states_queue.append(act)

if __name__ == "__main__":
    rospy.init_node("landing_time_detector")

    pub = rospy.Publisher("~pictogram_array", PictogramArray)
    ref_contact_states_sub = message_filters.Subscriber('~input_ref', ContactStatesStamped)
    act_contact_states_sub = message_filters.Subscriber('~input_act', ContactStatesStamped)

    buffer_size = 5
    ref_contact_states_queue = []
    act_contact_states_queue = []

    ts = message_filters.TimeSynchronizer([ref_contact_states_sub, act_contact_states_sub], 10)
    ts.registerCallback(callback)

    rospy.spin()
