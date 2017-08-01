import os
import sys
import multiprocessing.dummy as mp
from collections import deque
from copy import copy
import time


class FileCollector:
    
    def __init__ (self, _fileList, _ReadFunc, queue_length=10):
        self.length = queue_length
        self.queue = deque(maxlen=self.length)
        self.ReadFunc = _ReadFunc
        self.fileList = _fileList
        self.cond = mp.Condition()
        self.process = mp.Process(target=self.producer)
        self.stop = mp.Event()
        self.process.start()
        self.full = mp.Event()
        
    def close (self):
        self.stop.set()
        self.full.set()
        print ("Stopping...")
        time.sleep(0.1)
        self.process.join()
        
    def pick (self):
        self.full.set()
        self.cond.acquire()
        if (len(self.queue)==0) :
            self.cond.wait()
        # XXX
        val = copy(self.queue.popleft())
        self.cond.release()
        return val
    
    def producer (self):
        i = 0
        while True:
            if self.stop.is_set():
                return
            self.cond.acquire()
            if len(self.queue) < self.length:
                curData = self.ReadFunc(self.fileList[i])
                i += 1
                self.queue.append(curData)
            else:
                self.full.wait()
            self.cond.notifyAll()
            self.cond.release()
#         self.cond.acquire()
#         self.cond.notify()
        