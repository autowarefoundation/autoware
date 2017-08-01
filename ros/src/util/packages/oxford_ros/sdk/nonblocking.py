#!/usr/bin/python

import sys
import os
import termios
import time
import select
import tty


class NonblockingKeybInput:
    
    @staticmethod
    def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    
    def __init__ (self):
        self.setNonBlock()
    
    def spacePressed (self):
        if NonblockingKeybInput.isData():
            c = sys.stdin.read(1)
            return (c==' ')
        
    def readUntilSpace (self):
        while (True):
            c = sys.stdin.read(1)
            if c==' ':
                return
            else:
                continue
        
    def setNonBlock (self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.nonblocking = True
        
    def setBlock (self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.nonblocking = False



if __name__ == '__main__' :
    
    isPause = NonblockingKeybInput()
    
    try:
        while True:
            print ("Hello")
            if isPause.spacePressed():
                isPause.readUntilSpace()
            time.sleep(0.1)
    except KeyboardInterrupt:
        isPause.setBlock()
        print ("Done")
        sys.exit()