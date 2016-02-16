# -*- coding: utf-8 -*-
import yaml
import sys

argvs = sys.argv
argc = len(argvs)
text = ""

if __name__ == "__main__":
    if (argc != 3):
        print '%python sync_generator.py argv[1] argv[2]'
        print '\t' + 'argv[1]: input *.yaml file'
        print '\t' + 'argv[2]: output *.cpp file'
        quit()
    f_config = open(argvs[1], 'r')
    f_generate = open(argvs[2], 'w')

    data = yaml.load(f_config)
    text = '/* ----header---- */\n'
    text += '/* common header */\n'
    text += '#include "ros/ros.h"\n'
    text += '#include <ros/callback_queue.h>\n'
    text += '#include <boost/circular_buffer.hpp>\n'
    text += '#include <vector>\n'
    text += '#include <stdio.h>\n'
    text += '#include <stdlib.h>\n'
    text += '#include <string.h>\n'
    text += '#include <signal.h>\n'
    text += '#include <sys/stat.h>\n'
    text += '#include <sys/select.h>\n'
    text += '#include <mqueue.h>\n'
    text += '#include <fcntl.h>\n'
    text += '#include <errno.h>\n'
    text += '#include <unistd.h>\n'
    text += '#include <pthread.h>\n'
    text += '#include "t_sync_message.h"\n'
    text += '/* user header */\n'
    text += '#include "%s.h"\n' % data['sub1_header']
    text += '#include "%s.h"\n' % data['sub2_header']
    text += '#include "%s.h"\n' % data['sync_sub1_header']
    text += '#include "%s.h"\n' % data['sync_sub2_header']

    text += '\n/* ----mode---- */\n'
    text += '#define _REQ_PUB %s\n\n' % data['req_pub_mode']

    text += '/* ----var---- */\n'
    text += '/* common var */\n'
    text += 'bool buf_flag;\n'
    text += 'pthread_mutex_t mutex;\n'
    text += '/* user var */\n'
    text += 'boost::circular_buffer<%s> %s_ringbuf(%s);\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1], data['sub1_ringbuf'])
    text += 'boost::circular_buffer<%s> %s_ringbuf(%s);\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1], data['sub2_ringbuf'])
    text += 'ros::Publisher %s_pub;\n' % data['pub1'].split('/')[-1]
    text += 'ros::Publisher %s_pub;\n' % data['pub2'].split('/')[-1]
    text += 'bool %s_flag;\n' % data['sync_sub1'].split('/')[-1]
    text += 'bool %s_flag;\n\n' % data['sync_sub2'].split('/')[-1]

    text += '/* ----function---- */\n'
    text += 'double fabs_time_diff(std_msgs::Header *timespec1, std_msgs::Header *timespec2) {\n'
    text += '    double time1 = (double)timespec1->stamp.sec + (double)timespec1->stamp.nsec/1000000000L;\n'
    text += '    double time2 = (double)timespec2->stamp.sec + (double)timespec2->stamp.nsec/1000000000L;\n\n'
    text += '    return fabs(time1 - time2);\n'
    text += '}\n\n'

    text += 'double get_time(const std_msgs::Header *timespec) {\n'
    text += '    return (double)timespec->stamp.sec + (double)timespec->stamp.nsec/1000000000L;\n'
    text += '}\n\n\n'


    text += '#if _REQ_PUB\n'
    text += '%s* p_%s_buf;\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
    text += '%s* p_%s_buf;\n\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])

    text += 'void %s_callback(const %s::ConstPtr& %s_msg) {\n' % (data['sub1'].split('/')[-1], data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
    text += '    pthread_mutex_lock(&mutex);\n'
    text += '    %s_ringbuf.push_front(*%s_msg);\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
    text += '    //%s is empty\n' % data['sub2'].split('/')[-1]
    text += '    if (%s_ringbuf.begin() == %s_ringbuf.end()) {\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '        pthread_mutex_unlock(&mutex);\n'
    text += '        ROS_INFO("%s ring buffer is empty");\n' % data['sub2'].split('/')[-1]
    text += '        return\n'
    text += '    }\n'
    text += '    buf_flag = true;\n'
    text += '    pthread_mutex_unlock(&mutex);\n'
    text += '}\n\n'

    text += 'void %s_callback(const %s::ConstPtr& %s_msg) {\n' % (data['sub2'].split('/')[-1], data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
    text += '    pthread_mutex_lock(&mutex);\n'
    text += '    %s_ringbuf.push_front(*%s_msg);\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '    //%s is empty\n' % data['sub1'].split('/')[-1]
    text += '    if (%s_ringbuf.begin() == %s_ringbuf.end()) {\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
    text += '        ROS_INFO("%s ring buffer is empty");\n' % data['sub1'].split('/')[-1]
    text += '        pthread_mutex_unlock(&mutex);\n'
    text += '        return;\n'
    text += '    }\n\n'

    text += '    buf_flag = true;\n'
    text += '    pthread_mutex_unlock(&mutex);\n'
    text += '}\n'
    text += '\n'
    text += 'void publish_msg(%s* p_%s_buf, %s* p_%s_buf)\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1], data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
    text += '{\n'
    text += '    ROS_INFO("publish");\n'
    text += '    %s_pub.publish(*p_%s_buf);\n' % (data['pub1'].split('/')[-1], data['sub1'].split('/')[-1])
    text += '    %s_pub.publish(*p_%s_buf);\n' % (data['pub2'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '}\n\n'

    text += 'bool publish() {\n'
    text += '    if (buf_flag) {\n'
    text += '        pthread_mutex_lock(&mutex)\n\n'

    text += '        //%s is empty\n' % data['sub1'].split('/')[-1]
    text += '        if (%s_ringbuf.begin() == %s_ringbuf.end()) {\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
    text += '            pthread_mutex_unlock(&mutex);\n'
    text += '            ROS_INFO("%s ring buffer is empty");\n'% data['sub1'].split('/')[-1]
    text += '            return false;\n'
    text += '        }\n\n'

    text += '        //%s is empty\n' % data['sub2'].split('/')[-1]
    text += '        if (%s_ringbuf.begin() == %s_ringbuf.end()) {\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '            pthread_mutex_unlock(&mutex);\n'
    text += '            ROS_INFO("%s ring buffer is empty");\n'% data['sub2'].split('/')[-1]
    text += '            return false;\n'
    text += '        }\n\n'

    if data['sched_policy'] == 1:
        text += '        // %s > %s\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        if (get_time(&(%s_ringbuf.front().header)) >= get_time(&(%s_ringbuf.front().header))) {\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '            p_%s_buf = &(%s_ringbuf.front());\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '            boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
        text += '            if (%s_ringbuf.size() == 1) {\n' % data['sub1'].split('/')[-1]
        text += '                p_%s_buf = &*it;\n' % data['sub1'].split('/')[-1]
        text += '                publish_msg(p_%s_buf, p_%s_buf);\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '                pthread_mutex_unlock(&mutex);\n'
        text += '                return true;\n'
        text += '            } else {\n'
        text += '                for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub1'].split('/')[-1]
        text += '                    if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['sub2'].split('/')[-1]
        text += '                        < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['sub2'].split('/')[-1]
        text += '                        p_%s_buf = &*(it-1);\n'  % data['sub1'].split('/')[-1]
        text += '                        break;\n'
        text += '                    }\n'
        text += '                }\n'
        text += '                if (it == %s_ringbuf.end()) {\n' % data['sub1'].split('/')[-1]
        text += '                    p_%s_buf = &(%s_ringbuf.back());\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
        text += '                }\n'
        text += '            }\n'
        text += '        }\n'
        text += '        // %s < %s\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        else {\n'
        text += '            p_%s_buf = &(%s_ringbuf.front());\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
        text += '            boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
        text += '            if (%s_ringbuf.size() == 1) {\n' % data['sub2'].split('/')[-1]
        text += '                p_%s_buf = &*it;\n' % data['sub2'].split('/')[-1]
        text += '                publish_msg(p_%s_buf, p_%s_buf);\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '                pthread_mutex_unlock(&mutex);\n'
        text += '                return true;\n'
        text += '            }\n\n'

        text += '            for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub2'].split('/')[-1]
        text += '                if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['sub1'].split('/')[-1]
        text += '                    < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['sub1'].split('/')[-1]
        text += '                    p_%s_buf = &*(it-1);\n' % data['sub2'].split('/')[-1]
        text += '                    break;\n'
        text += '                }\n'
        text += '            }\n\n'

        text += '            if (it == %s_ringbuf.end()) {\n' % data['sub2'].split('/')[-1]
        text += '                p_%s_buf = &(%s_ringbuf.back());\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '            }\n'
        text += '        }\n'
    elif data['sched_policy'] == 2:
        text += '        p_%s_buf = &(%s_ringbuf.front());\n' % (data['short_rate'].split('/')[-1], data['short_rate'].split('/')[-1])

        if data['short_rate'] == data['sub1'] :
            text += '        boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
            text += '        if (%s_ringbuf.size() == 1) {\n' % data['sub2'].split('/')[-1]
            text += '            p_%s_buf = &*it;\n' % data['sub2'].split('/')[-1]
            text += '            publish_msg(p_%s_buf, p_%s_buf);\n' % (data['short_rate'].split('/')[-1], data['sub2'].split('/')[-1])
            text += '            pthread_mutex_unlock(&mutex);\n'
            text += '            return true;\n'
            text += '        }\n\n'

            text += '        for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub2'].split('/')[-1]
            text += '            if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['short_rate'].split('/')[-1]
            text += '                < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['short_rate'].split('/')[-1]
            text += '                p_%s_buf = &*(it-1);\n' % data['sub2'].split('/')[-1]
            text += '                break;\n'
            text += '            }\n'
            text += '        }\n\n'

            text += '        if (it == %s_ringbuf.end()) {\n' % data['sub2'].split('/')[-1]
            text += '            p_%s_buf = &(%s_ringbuf.back());\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
            text += '        }\n'
        elif data['short_rate'] == data['sub2']:
            text += '        boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
            text += '        if (%s_ringbuf.size() == 1) {\n' % data['sub1'].split('/')[-1]
            text += '            p_%s_buf = &*it;\n' % data['sub1'].split('/')[-1]
            text += '            publish_msg(p_%s_buf, p_%s_buf);\n' % (data['short_rate'].split('/')[-1], data['sub1'].split('/')[-1])
            text += '            pthread_mutex_unlock(&mutex);\n'
            text += '            return true;\n'
            text += '        }\n\n'

            text += '        for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub1'].split('/')[-1]
            text += '            if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['short_rate'].split('/')[-1]
            text += '                < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['short_rate'].split('/')[-1]
            text += '                p_%s_buf = &*(it-1);\n' % data['sub1'].split('/')[-1]
            text += '                break;\n'
            text += '            }\n'
            text += '        }\n\n'

            text += '        if (it == %s_ringbuf.end()) {\n' % data['sub1'].split('/')[-1]
            text += '            p_%s_buf = &(%s_ringbuf.back());\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
            text += '        }\n'
        else :
            print "failed: sched_policy 2, short_rate unmatched sub1 or sub2"

    text += '        publish_msg(p_%s_buf, p_%s_buf);\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '        pthread_mutex_unlock(&mutex);\n'
    text += '        return true;\n'
    text += '    } else {\n'
    text += '        return false;\n'
    text += '    }\n'
    text += '}\n'
    text += '#else\n'
    text += '%s %s_buf;\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
    text += '%s %s_buf;\n\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])

    text += 'void %s_callback(const %s::ConstPtr& %s_msg) {\n' % (data['sub1'].split('/')[-1], data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
    text += '    pthread_mutex_lock(&mutex);\n'
    text += '    %s_ringbuf.push_front(*%s_msg);\n\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])

    text += '    //%s is empty\n' % data['sub2'].split('/')[-1]
    text += '    if (%s_ringbuf.begin() == %s_ringbuf.end()) {\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '        pthread_mutex_unlock(&mutex);\n'
    text += '        ROS_INFO("%s ring buffer is empty");\n' % data['sub2'].split('/')[-1]
    text += '        return;\n'
    text += '    }\n\n'

    text += '    buf_flag = true;\n\n'
    if data['sched_policy'] == 1:
        text += '    // %s > %s\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '    if (get_time(&(%s_ringbuf.front().header)) >= get_time(&(%s_ringbuf.front().header))) {\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        %s_buf = %s_ringbuf.front();\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
        text += '        if (%s_ringbuf.size() == 1) {\n' % data['sub1'].split('/')[-1]
        text += '            %s_buf = *it;\n' % data['sub1'].split('/')[-1]
        text += '            pthread_mutex_unlock(&mutex);\n'
        text += '            return;\n'
        text += '        } else {\n'
        text += '            for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub1'].split('/')[-1]
        text += '                if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['sub2'].split('/')[-1]
        text += '                    < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['sub2'].split('/')[-1]
        text += '                    %s_buf = *(it-1);\n' % data['sub1'].split('/')[-1]
        text += '                    break;\n'
        text += '                }\n'
        text += '            }\n'
        text += '            if (it == %s_ringbuf.end()) {\n' % data['sub1'].split('/')[-1]
        text += '                %s_buf = %s_ringbuf.back();\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
        text += '            }\n'
        text += '        }\n\n'

        text += '    } else {\n'
        text += '        %s_buf = %s_ringbuf.front();\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
        text += '        boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
        text += '        if (%s_ringbuf.size() == 1) {\n' % data['sub2'].split('/')[-1]
        text += '            %s_buf = *it;\n' % data['sub2'].split('/')[-1]
        text += '            pthread_mutex_unlock(&mutex);\n'
        text += '            return;\n'
        text += '        }\n\n'

        text += '        for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub2'].split('/')[-1]
        text += '            if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['sub1'].split('/')[-1]
        text += '                < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['sub1'].split('/')[-1]
        text += '                %s_buf = *(it-1);\n' % data['sub2'].split('/')[-1]
        text += '                break;\n'
        text += '            }\n'
        text += '        }\n\n'

        text += '        if (it == %s_ringbuf.end()) {\n' % data['sub2'].split('/')[-1]
        text += '            %s_buf = %s_ringbuf.back();\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        }\n'
        text += '    }\n'
    elif data['sched_policy'] == 2:
        text += '    %s_buf = %s_ringbuf.front();\n' % (data['short_rate'].split('/')[-1], data['short_rate'].split('/')[-1])
        if data['short_rate'] == data['sub1'] :
            text += '    boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
            text += '    if (%s_ringbuf.size() == 1) {\n' % data['sub2'].split('/')[-1]
            text += '        %s_buf = *it;\n' % data['sub2'].split('/')[-1]
            text += '        pthread_mutex_unlock(&mutex);\n'
            text += '        return;\n'
            text += '    }\n\n'

            text += '    for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub2'].split('/')[-1]
            text += '        if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['short_rate'].split('/')[-1]
            text += '            < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['short_rate'].split('/')[-1]
            text += '            %s_buf = *(it-1);\n' % data['sub2'].split('/')[-1]
            text += '            break;\n'
            text += '        }\n'
            text += '    }\n\n'

            text += '    if (it == %s_ringbuf.end()) {\n' % data['sub2'].split('/')[-1]
            text += '        %s_buf = %s_ringbuf.back();\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
            text += '    }\n'
        elif data['short_rate'] == data['sub2']:
            text += '    boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
            text += '    if (%s_ringbuf.size() == 1) {\n' % data['sub1'].split('/')[-1]
            text += '        %s_buf = *it;\n' % data['sub1'].split('/')[-1]
            text += '        pthread_mutex_unlock(&mutex);\n'
            text += '        return;\n'
            text += '    }\n\n'

            text += '    for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub1'].split('/')[-1]
            text += '        if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['short_rate'].split('/')[-1]
            text += '            < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['short_rate'].split('/')[-1]
            text += '            %s_buf = *(it-1);\n' % data['sub1'].split('/')[-1]
            text += '            break;\n'
            text += '        }\n'
            text += '    }\n\n'

            text += '    if (it == %s_ringbuf.end()) {\n' % data['sub1'].split('/')[-1]
            text += '        %s_buf = %s_ringbuf.back();\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
            text += '    }\n'
        else :
            print "failed: sched_policy 2, short_rate unmatched sub1 or sub2"

    text += '    pthread_mutex_unlock(&mutex);\n'
    text += '}\n\n'

    text += 'void %s_callback(const %s::ConstPtr& %s_msg) {\n' % (data['sub2'].split('/')[-1], data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
    text += '    pthread_mutex_lock(&mutex);\n'
    text += '    %s_ringbuf.push_front(*%s_msg);\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])

    text += '    //%s is empty\n' % data['sub1'].split('/')[-1]
    text += '    if (%s_ringbuf.begin() == %s_ringbuf.end()) {\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
    text += '        ROS_INFO("%s ring buffer is empty");\n' % data['sub1'].split('/')[-1]
    text += '        pthread_mutex_unlock(&mutex);\n'
    text += '        return;\n'
    text += '    }\n\n'

    text += '    buf_flag = true;\n\n'
    if data['sched_policy'] == 1:
        text += '    // %s > %s\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '    if (get_time(&(%s_ringbuf.front().header)) >= get_time(&(%s_ringbuf.front().header))) {\n' % (data['sub1'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        %s_buf = %s_ringbuf.front();\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
        text += '        if (%s_ringbuf.size() == 1) {\n' % data['sub1'].split('/')[-1]
        text += '            %s_buf = *it;\n' % data['sub1'].split('/')[-1]
        text += '            pthread_mutex_unlock(&mutex);\n'
        text += '            return;\n'
        text += '        } else {\n'
        text += '            for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub1'].split('/')[-1]
        text += '                if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['sub2'].split('/')[-1]
        text += '                    < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['sub2'].split('/')[-1]
        text += '                    %s_buf = *(it-1);\n' % data['sub1'].split('/')[-1]
        text += '                    break;\n'
        text += '                }\n'
        text += '            }\n'
        text += '            if (it == %s_ringbuf.end()) {\n' % data['sub1'].split('/')[-1]
        text += '                %s_buf = %s_ringbuf.back();\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
        text += '            }\n'
        text += '        }\n\n'

        text += '    } else {\n'
        text += '        %s_buf = %s_ringbuf.front();\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
        text += '        boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
        text += '        if (%s_ringbuf.size() == 1) {\n' % data['sub2'].split('/')[-1]
        text += '            %s_buf = *it;\n' % data['sub2'].split('/')[-1]
        text += '            pthread_mutex_unlock(&mutex);\n'
        text += '            return;\n'
        text += '        }\n\n'

        text += '        for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub2'].split('/')[-1]
        text += '            if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['sub1'].split('/')[-1]
        text += '                < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['sub1'].split('/')[-1]
        text += '                %s_buf = *(it-1);\n' % data['sub2'].split('/')[-1]
        text += '                break;\n'
        text += '            }\n'
        text += '        }\n\n'

        text += '        if (it == %s_ringbuf.end()) {\n' % data['sub2'].split('/')[-1]
        text += '            %s_buf = %s_ringbuf.back();\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
        text += '        }\n'
        text += '    }\n'
    elif data['sched_policy'] == 2:
        text += '    %s_buf = %s_ringbuf.front();\n' % (data['short_rate'].split('/')[-1], data['short_rate'].split('/')[-1])
        if data['short_rate'] == data['sub1'] :
            text += '    boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub2_header'].replace('/', '::'), data['sub2'].split('/')[-1])
            text += '    if (%s_ringbuf.size() == 1) {\n' % data['sub2'].split('/')[-1]
            text += '        %s_buf = *it;\n' % data['sub2'].split('/')[-1]
            text += '        pthread_mutex_unlock(&mutex);\n'
            text += '        return;\n'
            text += '    }\n\n'
            text += '    for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub2'].split('/')[-1]
            text += '        if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['short_rate'].split('/')[-1]
            text += '            < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['short_rate'].split('/')[-1]
            text += '            %s_buf = *(it-1);\n' % data['sub2'].split('/')[-1]
            text += '            break;\n'
            text += '        }\n'
            text += '    }\n\n'

            text += '    if (it == %s_ringbuf.end()) {\n' % data['sub2'].split('/')[-1]
            text += '        %s_buf = %s_ringbuf.back();\n' % (data['sub2'].split('/')[-1], data['sub2'].split('/')[-1])
            text += '    }\n'
        elif data['short_rate'] == data['sub2']:
            text += '    boost::circular_buffer<%s>::iterator it = %s_ringbuf.begin();\n' % (data['sub1_header'].replace('/', '::'), data['sub1'].split('/')[-1])
            text += '    if (%s_ringbuf.size() == 1) {\n' % data['sub1'].split('/')[-1]
            text += '        %s_buf = *it;\n' % data['sub1'].split('/')[-1]
            text += '        pthread_mutex_unlock(&mutex);\n'
            text += '        return;\n'
            text += '    }\n\n'

            text += '    for (it++; it != %s_ringbuf.end(); it++) {\n' % data['sub1'].split('/')[-1]
            text += '        if (fabs_time_diff(&(%s_ringbuf.front().header), &((it-1)->header))\n' % data['short_rate'].split('/')[-1]
            text += '            < fabs_time_diff(&(%s_ringbuf.front().header), &(it->header))) {\n' % data['short_rate'].split('/')[-1]
            text += '            %s_buf = *(it-1);\n' % data['sub1'].split('/')[-1]
            text += '            break;\n'
            text += '        }\n'
            text += '    }\n\n'

            text += '    if (it == %s_ringbuf.end()) {\n' % data['sub1'].split('/')[-1]
            text += '        %s_buf = %s_ringbuf.back();\n' % (data['sub1'].split('/')[-1], data['sub1'].split('/')[-1])
            text += '    }\n'
        else :
            print "failed: sched_policy 2, short_rate unmatched sub1 or sub2"

    text += '    pthread_mutex_unlock(&mutex);\n'
    text += '}\n\n'

    text += 'bool publish() {\n'
    text += '    if (buf_flag) {\n'
    text += '        pthread_mutex_lock(&mutex);\n'
    text += '        // scan_ringbuf.clear();\n'
    text += '        // image_ringbuf.clear();\n'
    text += '        // scan_ringbuf.push_front(scan_buf);\n'
    text += '        // image_ringbuf.push_front(image_buf);\n'
    text += '        ROS_INFO("publish");\n'
    text += '        %s_pub.publish(%s_buf);\n' % (data['pub1'].split('/')[-1], data['sub1'].split('/')[-1])
    text += '        %s_pub.publish(%s_buf);\n' % (data['pub2'].split('/')[-1], data['sub2'].split('/')[-1])
    text += '        pthread_mutex_unlock(&mutex);\n'
    text += '        return true;\n'
    text += '    } else {\n'
    text += '        ROS_INFO("publish failed");\n'
    text += '        return false;\n'
    text += '    }\n'
    text += '}\n'
    text += '#endif\n\n'

    text += 'void %s_callback(const %s::ConstPtr& %s_msg) {\n' % (data['sync_sub1'].split('/')[-1], data['sync_sub1_header'].replace('/', '::'), data['sync_sub1'].split('/')[-1])
    text += '    if (%s_flag) {\n' % data['sync_sub1'].split('/')[-1]
    text += '        %s_flag = false;\n' % data['sync_sub1'].split('/')[-1]
    text += '        %s_flag = false;\n' % data['sync_sub2'].split('/')[-1]
    text += '        return;\n'
    text += '    }\n\n'

    text += '    %s_flag = true;\n' % data['sync_sub1'].split('/')[-1]
    text += '    if (%s_flag) {\n' % data['sync_sub2'].split('/')[-1]
    text += '        ROS_INFO("catch publish request");\n'
    text += '        if(!publish()) {\n'
    text += '            /* when to publish is failure, republish */\n'
    text += '            struct timespec sleep_time;\n'
    text += '            sleep_time.tv_sec = 0;\n'
    text += '            sleep_time.tv_nsec = 200000000; //5Hz\n'
    text += '            while (!publish() || ros::ok())\n'
    text += '                nanosleep(&sleep_time, NULL);\n'
    text += '        }\n'
    text += '        %s_flag = false;\n' % data['sync_sub1'].split('/')[-1]
    text += '        %s_flag = false;\n' % data['sync_sub2'].split('/')[-1]
    text += '    }\n'
    text += '}\n'

    text += 'void %s_callback(const %s::ConstPtr& %s_msg) {\n' % (data['sync_sub2'].split('/')[-1], data['sync_sub2_header'].replace('/', '::'), data['sync_sub2'].split('/')[-1])
    text += '    if (%s_flag) {\n' % data['sync_sub2'].split('/')[-1]
    text += '        %s_flag = false;\n' % data['sync_sub1'].split('/')[-1]
    text += '        %s_flag = false;\n' % data['sync_sub2'].split('/')[-1]
    text += '        return;\n'
    text += '    }\n\n'

    text += '    %s_flag = true;\n' % data['sync_sub2'].split('/')[-1]
    text += '    if (%s_flag) {\n' % data['sync_sub1'].split('/')[-1]
    text += '        ROS_INFO("catch publish request");\n'
    text += '        if(!publish()) {\n'
    text += '            /* when to publish is failure, republish */\n'
    text += '            struct timespec sleep_time;\n'
    text += '            sleep_time.tv_sec = 0;\n'
    text += '            sleep_time.tv_nsec = 200000000; //5Hz\n'
    text += '            while (!publish() || ros::ok())\n'
    text += '                nanosleep(&sleep_time, NULL);\n'
    text += '        }\n'
    text += '        %s_flag = false;\n' % data['sync_sub1'].split('/')[-1]
    text += '        %s_flag = false;\n' % data['sync_sub2'].split('/')[-1]
    text += '    }\n'
    text += '}\n\n'

    text += 'void* thread(void* args)\n'
    text += '{\n'
    text += '    ros::NodeHandle nh_rcv;\n'
    text += '    ros::CallbackQueue rcv_callbackqueue;\n'
    text += '    nh_rcv.setCallbackQueue(&rcv_callbackqueue);\n'
    text += '    ros::Subscriber %s_sub = nh_rcv.subscribe("%s", 5, %s_callback);\n' % (data['sync_sub1'].split('/')[-1], data['sync_sub1'].split('/')[-1], data['sync_sub1'].split('/')[-1])
    text += '    ros::Subscriber %s_sub = nh_rcv.subscribe("%s", 5, %s_callback);\n' % (data['sync_sub2'].split('/')[-1], data['sync_sub2'].split('/')[-1], data['sync_sub2'].split('/')[-1])
    text += '    while (nh_rcv.ok())\n'
    text += '        rcv_callbackqueue.callAvailable(ros::WallDuration(1.0f));\n'
    text += '    return NULL;\n'
    text += '}\n\n'

    text += 'int main(int argc, char **argv) {\n'
    text += '    ros::init(argc, argv, "%s");\n' % data['node_name'].split('/')[-1]
    text += '    ros::NodeHandle nh;\n\n'

    text += '    /* create server thread */\n'
    text += '    pthread_t th;\n'
    text += '    pthread_create(&th, NULL, thread, (void *)NULL );\n\n'

    text += '    ros::Subscriber %s_sub = nh.subscribe("%s", 1, %s_callback);\n' % (data['sub1'].split('/')[-1], data['sub1'], data['sub1'].split('/')[-1])
    text += '    ros::Subscriber %s_sub = nh.subscribe("%s", 1, %s_callback);\n' % (data['sub2'].split('/')[-1], data['sub2'], data['sub2'].split('/')[-1])
    text += '    %s_pub = nh.advertise<%s>("%s", 5);\n' % (data['pub1'].split('/')[-1], data['sub1_header'].replace('/', '::'), data['pub1'])
    text += '    %s_pub = nh.advertise<%s>("%s", 5);\n' % (data['pub2'].split('/')[-1], data['sub2_header'].replace('/', '::'), data['pub2'])
    text += '    while (!buf_flag) {\n'
    text += '        ros::spinOnce();\n'
    text += '    }\n'
    text += '    if(!publish()) {\n'
    text += '        /* when to publish is failure, republish */\n'
    text += '        struct timespec sleep_time;\n'
    text += '        sleep_time.tv_sec = 0;\n'
    text += '        sleep_time.tv_nsec = 200000000; //5Hz\n'
    text += '        while (!publish() || ros::ok())\n'
    text += '            nanosleep(&sleep_time, NULL);\n'
    text += '    }\n\n'

    text += '    ros::spin();\n\n'

    text += '    /* shutdown server thread */\n'
    text += '    ROS_INFO("wait until shutdown a thread");\n'
    text += '    pthread_kill(th, SIGINT);\n'
    text += '    pthread_join(th, NULL);\n\n'

    text += '    return 0;\n'
    text += '}\n'

    f_generate.write(text)

    f_config.close()
    f_generate.close()

    print "generate "
