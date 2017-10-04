#!/usr/bin/env python
# coding: utf-8

from subprocess import call, Popen


class ROSCommand(object):
    @staticmethod
    def call(command_line=["echo", "plz input command line as list."]):
        result = None
        if "ros" in command_line[0]:
            result = call(command_line)
            print("{} => result={}".format(" ".join(command_line), result))
        else:
            print("command ({}) is unavailable".format(command_line[0]))
        return result

    @staticmethod
    def popen(command_line=["echo", "plz input command line as list."], stdout=None):
        result = None
        if "ros" in command_line[0]:
            if stdout is None:
                stdout = open("/dev/null", "w")
                result = Popen(command_line, stdout=stdout)
                stdout.close()
            else:
                result = Popen(command_line, stdout=stdout)
            print("{} => pid={}".format(" ".join(command_line), result))
        else:
            print("command ({}) is unavailable".format(command_line[0]))
        return result
