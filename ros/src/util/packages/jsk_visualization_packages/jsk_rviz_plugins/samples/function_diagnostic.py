#!/usr/bin/env python
# coding: utf-8

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs

temperature = 0

def check_temprature(stat):

    #エラー判定
    if temperature < 10:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Temperature OK")
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Too high")

    #エラー情報を追加
    stat.add("Top-Side Margin", 10 - temperature)
    return stat


if __name__=='__main__':
    rospy.init_node("diagnostic_updater_example2")

    # Updaterのオブジェクトを作成
    updater = diagnostic_updater.Updater()

    # ハードウェアIDを設定
    updater.setHardwareID("Sensor1")

    # ハードエラーチェック機能の追加
    updater.add("upper-temperature",check_temprature)

    while not rospy.is_shutdown():
        
        temperature+=0.1
        if temperature>=20:
            temperature=0

        print temperature

        #Topic publish
        updater.update()
        rospy.sleep(0.1)
