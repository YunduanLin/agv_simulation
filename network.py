from numpy import np
from agent import *

class Warehousemap:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.grid = np.zeros(self.height, self.width)
        self.list_ws, self.list_ds, self.list_agv = [], [], [] # Collections of workstations, dropstations and agvs
        self.cnt_ws, self.cnt_ds, self.cnt_agv = 0, 0, 0

        self.t = 0

        self.generate_shortest_path()

    def add_agent(self, x, y, node_type, dest=None, n=None):
        if node_type == 'workstation':
            # add workstation
            node_ws = Workstation(x, y, self.cnt_ws+1)
            self.list_ws.append(node_ws)
            self.cnt_ws += 1
            self.grid[x, y] = 1
        elif node_type == 'dropstation':
            # add dropstation
            node_ds = Dropstation(x, y, self.cnt_ds+1)
            self.list_ws.append(node_ds)
            self.cnt_ds += 1
            self.grid[x, y] = 1
        elif node_type == 'agv':
            # add agv
            node_agv = Agv(x, y, self.cnt_agv+1, n)
            self.list_agv.append(node_agv)
            self.cnt_agv += 1

    def add_package(self):
        # TODO: add packages
        return


    def next_step(self):
        for agv in self.list_agv:
            agv.next_step()
        for ws in self.list_ws:
            ws.next_step()
        return

    # TODO: find the shortest path between two nodes while the velocity at origin/destination should be 0
    #       1. avoid through stations
    #       2. find path
    def generate_shortest_path(self):

        self.dist = np.ones(self.cnt_ws + self.cnt_ds, self.cnt_ws + self.cnt_ds)