#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

"""
@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
import std_msgs


time_to_launch = 0

'''Used as a tutorial for loading and using diagnostic updater.

DummyClass and dummy_diagnostics show how to use a diagnostic_updater
class.
'''

def dummy_diagnostic(stat):
    # stat is supposed to be of type diagnostic_updater.DiagnosticStatusWrapper
    # DiagnosticStatusWrapper is a derived class of
    # diagnostic_msgs.msg.DiagnosticStatus that provides a set of convenience
    # methods.

    # summary sets the level and message.
    # As opposed to the C++ API, there is no summaryf function: use python's
    # string formatting instead
    if time_to_launch < 10:
        # summary for formatted text.
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
            "Buckle your seat belt. Launch in %f seconds!" % time_to_launch)
    else:
        # summary for unformatted text. It's just the same ;)
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
            "Launch is in a long time. Have a soda.")

    # add is used to append key-value pairs.
    # Again, as oppose to the C++ API, there is no addf function. The second
    # argument is always converted to a str using the str() function.
    stat.add("Diagnostic Name", "dummy")
    # add transparently handles conversion to string (using str()).
    stat.add("Time to Launch", time_to_launch)
    # add allows arbitrary printf style formatting.
    stat.add("Geeky thing to say", "The square of the time to launch %f is %f" % \
        (time_to_launch, time_to_launch * time_to_launch) )

    # As opposed to the C++ diagnostic function which modifies its argument,
    # the python version must return the modified message.
    return stat


class DummyClass:
    def produce_diagnostics(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "This is a silly updater.")
        stat.add("Stupidicity of this updater", 1000.)
        return stat


class DummyTask(diagnostic_updater.DiagnosticTask):
    def __init__(self):
        diagnostic_updater.DiagnosticTask.__init__(self,
            "Updater Derived from DiagnosticTask")

    def run(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
            "This is another silly updater.")
        stat.add("Stupidicity of this updater", 2000.)
        return stat


def check_lower_bound(stat):
    if time_to_launch > 5:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Lower-bound OK")
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Too low")
    stat.add("Low-Side Margin", time_to_launch - 5)
    return stat


def check_upper_bound(stat):
    if time_to_launch < 10:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Upper-bound OK")
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Too high")
    stat.add("Top-Side Margin", 10 - time_to_launch)
    return stat


if __name__=='__main__':
    rospy.init_node("diagnostic_updater_example")

    # The Updater class advertises to /diagnostics, and has a
    # ~diagnostic_period parameter that says how often the diagnostics
    # should be published.
    updater = diagnostic_updater.Updater()

    # The diagnostic_updater.Updater class will fill out the hardware_id
    # field of the diagnostic_msgs.msg.DiagnosticStatus message. You need to
    # use the setHardwareID() method to set the hardware ID.
    #
    # The hardware ID should be able to identify the specific device you are
    # working with.  If it is not appropriate to fill out a hardware ID in
    # your case, you should call setHardwareID("none") to avoid warnings.
    # (A warning will be generated as soon as your node updates with no
    # non-OK statuses.)
    updater.setHardwareID("none")
    # Or...
    updater.setHardwareID("Device-%i-%i" % (27, 46) )

    # Diagnostic tasks are added to the Updater. They will later be run when
    # the updater decides to update.
    # As opposed to the C++ API, there is only one add function. It can take
    # several types of arguments:
    #  - add(task): where task is a DiagnosticTask
    #  - add(name, fn): add a DiagnosticTask embodied by a name and function
    updater.add("Function updater", dummy_diagnostic)
    dc = DummyClass()
    updater.add("Method updater", dc.produce_diagnostics)

    # Internally, updater.add converts its arguments into a DiagnosticTask.
    # Sometimes it can be useful to work directly with DiagnosticTasks. Look
    # at FrequencyStatus and TimestampStatus in update_functions for a
    # real-life example of how to make a DiagnosticTask by deriving from
    # DiagnosticTask.

    # Alternatively, a FunctionDiagnosticTask is a derived class from
    # DiagnosticTask that can be used to create a DiagnosticTask from
    # a function. This will be useful when combining multiple diagnostic
    # tasks using a CompositeDiagnosticTask.
    lower = diagnostic_updater.FunctionDiagnosticTask("Lower-bound check",
        check_lower_bound)
    upper = diagnostic_updater.FunctionDiagnosticTask("Upper-bound check",
        check_upper_bound)

    # If you want to merge the outputs of two diagnostic tasks together, you
    # can create a CompositeDiagnosticTask, also a derived class from
    # DiagnosticTask. For example, we could combine the upper and lower
    # bounds check into a single DiagnosticTask.
    bounds = diagnostic_updater.CompositeDiagnosticTask("Bound check")
    bounds.addTask(lower)
    bounds.addTask(upper)

    # We can then add the CompositeDiagnosticTask to our Updater. When it is
    # run, the overall name will be the name of the composite task, i.e.,
    # "Bound check". The summary will be a combination of the summary of the
    # lower and upper tasks (see DiagnosticStatusWrapper.mergeSummary for
    # details on how the merging is done). The lists of key-value pairs will be
    # concatenated.
    updater.add(bounds)

    # You can broadcast a message in all the DiagnosticStatus if your node
    # is in a special state.
    updater.broadcast(0, "Doing important initialization stuff.")

    pub1 = rospy.Publisher("topic1", std_msgs.msg.Bool)
    pub2_temp = rospy.Publisher("topic2", std_msgs.msg.Bool)
    rospy.sleep(2) # It isn't important if it doesn't take time.

    # Some diagnostic tasks are very common, such as checking the rate
    # at which a topic is publishing, or checking that timestamps are
    # sufficiently recent. FrequencyStatus and TimestampStatus can do these
    # checks for you.
    #
    # Usually you would instantiate them via a HeaderlessTopicDiagnostic
    # (FrequencyStatus only, for topics that do not contain a header) or a
    # TopicDiagnostic (FrequencyStatus and TimestampStatus, for topics that
    # do contain a header).
    #
    # Some values are passed to the constructor as pointers. If these values
    # are changed, the FrequencyStatus/TimestampStatus will start operating
    # with the new values.
    #
    # Refer to diagnostic_updater.FrequencyStatusParam and
    # diagnostic_updater.TimestampStatusParam documentation for details on
    # what the parameters mean:
    freq_bounds = {'min':0.5, 'max':2} # If you update these values, the
    # HeaderlessTopicDiagnostic will use the new values.
    pub1_freq = diagnostic_updater.HeaderlessTopicDiagnostic("topic1", updater,
        diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10))

    # Note that TopicDiagnostic, HeaderlessDiagnosedPublisher,
    # HeaderlessDiagnosedPublisher and DiagnosedPublisher all descend from
    # CompositeDiagnosticTask, so you can add your own fields to them using
    # the addTask method.
    #
    # Each time pub1_freq is updated, lower will also get updated and its
    # output will be merged with the output from pub1_freq.
    pub1_freq.addTask(lower) # (This wouldn't work if lower was stateful).

    # If we know that the state of the node just changed, we can force an
    # immediate update.
    updater.force_update()

    # We can remove a task by refering to its name.
    if not updater.removeByName("Bound check"):
        rospy.logerr("The Bound check task was not found when trying to remove it.")

    while not rospy.is_shutdown():
        msg = std_msgs.msg.Bool()
        rospy.sleep(0.1)

        # Calls to pub1 have to be accompanied by calls to pub1_freq to keep
        # the statistics up to date.
        msg.data = False
        pub1.publish(msg)
        pub1_freq.tick()

        # We can call updater.update whenever is convenient. It will take care
        # of rate-limiting the updates.
        updater.update()