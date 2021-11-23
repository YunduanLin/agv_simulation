import numpy as np

big_M = 99999

class Package:
    def __init__(self, orig, dest, index):
        # OD pair (orig is a work station, dest is a drop station)
        self.orig, self.dest = orig, dest
        self.index = index

        self.state = 'waiting'  # ['waiting', 'loading', 'completed']


class Workstation:
    '''
    A class for work station in the warehouse

    constant
    :arg loc (tuple): Location of the work station.
    :arg index (int): Index for the work station.
    :arg process_time (int): The time needed to process a package after the agv arrives at the work station.
    :arg buffer_capacity (int): Tha maximum number of agvs that can be added to the queue.

    real-time information
    :arg state (str): Real-time state of the agv. {'idle', 'moving', 'rotating', 'loading', 'unloading', 'waiting'}
    :arg packages (list[Package]): List of packages waiting at the work station.
    :arg queue (list[Package]): List of packages that are ready for pick up, order according to agvs.
    :arg occupied_time (int): The remaining time of loading the first package in the queue.
    '''

    def __init__(self, x, y, index):
        # position x and y
        self.index = index
        self.process_time = 2
        self.buffer_capacity = 2

        self.loc = (x, y)
        self.occupied_time = 0
        self.serving_package = None
        self.packages = []
        self.queue = []

    def add_to_queue(self, package):
        '''
        Add a package to the queue
        :param package: the package that are ready for picking up
        :return: current agv waiting time
        '''
        if self.queue:
            self.queue.append(package)
        else:
            self.occupied_time = self.process_time
            self.serving_package = package
        self.packages.remove(package)

        return self.occupied_time + len(self.queue) * self.process_time - 1

    def next_step(self):
        '''
        Process the queues and packages
        :return:
        '''
        if self.occupied_time > 1:
            self.occupied_time -= 1
        elif self.queue:
            self.occupied_time = self.process_time
            self.serving_package = self.queue.pop(0)

class Dropstation:
    def __init__(self, x, y, index):
        # position x and y
        self.loc = (x, y)
        self.index = index
        self.packages = []
