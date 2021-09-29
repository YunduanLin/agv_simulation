import numpy as np
from agent import *

class Warehousemap:
    def __init__(self, height, width, agents, packages):
        self.height, self.width = height, width
        self.grid = np.zeros((self.height, self.width))
        self.list_ws, self.list_ds, self.list_agv = [], [], [] # Collections of workstations, dropstations and agvs
        self.cnt_ws, self.cnt_ds, self.cnt_agv = 0, 0, 0
        self.cnt_p = 0

        self.t = 0

        for agent in agents:
            self.add_agent(**agent)

        for package in packages:
            self.add_package(**package)

        for agv in self.list_agv:
            agv.generate_shortest_path(self.grid)

    def add_agent(self, x, y, node_type, n=None):
        if node_type == 'workstation':
            # add workstation
            node_ws = Workstation(x, y, self.cnt_ws)
            self.list_ws.append(node_ws)
            self.cnt_ws += 1
            self.grid[x, y] = 1
        elif node_type == 'dropstation':
            # add dropstation
            node_ds = Dropstation(x, y, self.cnt_ds)
            self.list_ds.append(node_ds)
            self.cnt_ds += 1
            self.grid[x, y] = 1
        elif node_type == 'agv':
            # add agv
            node_agv = Agv(x, y, self.cnt_agv, n)
            self.list_agv.append(node_agv)
            self.cnt_agv += 1

    def add_package(self, orig, dest, agv = None):
        package = Package(self.list_ws[orig], self.list_ds[dest], self.cnt_p)
        self.cnt_p += 1
        self.list_ws[orig].packages.append(package)
        if agv is not None:
            self.list_agv[agv].assigned_packages.append(package)


    def next_step(self):
        for agv in self.list_agv:
            if agv.state == 'idle':
                if agv.packages:
                    package = agv.packages.pop()
                    agv.pathfinding(agv.loc, package.orig.loc)
                    agv.pathfinding(package.orig.loc, package.dest.loc)
                    agv.actions.append(('loading', package))
                    agv.actions.append(('unloading', package))
                    agv.state = 'moving'
                    agv.dest = agv.path.pop(0)
                    agv.next_step()
            else:
                agv.next_step()

        for ws in self.list_ws:
            ws.next_step()
        return
