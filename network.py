import numpy as np
from agent import *

class Warehousemap:
    def __init__(self, height, width, agents):
        self.height, self.width = height, width
        self.grid = np.zeros((self.height, self.width))
        self.list_ws, self.list_ds, self.list_agv = [], [], [] # Collections of workstations, dropstations and agvs
        self.cnt_ws, self.cnt_ds, self.cnt_agv = 0, 0, 0

        self.t = 0

        for agent in agents:
            self.add_agent(**agent)

        for agv in self.list_agv:
            agv.generate_shortest_path(self.grid)

    def add_agent(self, x, y, node_type, n=None):
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
