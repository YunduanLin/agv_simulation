import numpy as np
import matplotlib.pyplot as plt
from agent import *
from patch import patch

class Warehousemap:
    def __init__(self, height, width, agents, packages):
        self.height, self.width = height, width
        self.list_ws, self.list_ds, self.list_agv = [], [], [] # Collections of workstations, dropstations and agvs
        self.cnt_ws, self.cnt_ds, self.cnt_agv = 0, 0, 0
        self.cnt_p = 0

        self.t = 0

        self.mat_patch = np.empty((self.height, self.width), dtype=object)

        for agent in agents:
            self.add_agent(**agent)

        self.initialize_patches()

        # for package in packages:
        #     self.add_package(**package)
        #
        # for agv in self.list_agv:
        #     agv.generate_shortest_path(self.grid)

    def add_agent(self, x, y, node_type, n=None):
        if node_type == 'workstation':
            # add workstation
            node_ws = Workstation(x, y, self.cnt_ws)
            self.list_ws.append(node_ws)
            self.cnt_ws += 1
        elif node_type == 'dropstation':
            # add dropstation
            node_ds = Dropstation(x, y, self.cnt_ds)
            self.list_ds.append(node_ds)
            self.cnt_ds += 1
        elif node_type == 'agv':
            # add agv
            node_agv = Agv(x, y, self.cnt_agv, n)
            self.list_agv.append(node_agv)
            self.cnt_agv += 1

    def to_ind(self, i, j):
        return int(i * self.width + j)

    def to_cord(self, ind):
        return ind // self.width, ind % self.width

    def update_station_patch(self, list_s):
        for s in list_s:
            x, y = s.loc
            patch_ind = self.to_ind(x, y)
            self.mat_patch[x, y].reset_dist()
            if x>0:
                self.mat_patch[x-1, y].set_pd_dist(2)
                self.dist[4*patch_ind, 4*(patch_ind-self.width)+2] = 0
                self.dist[4*(patch_ind-self.width)+2, 4*patch_ind] = 0
            if y>0:
                self.mat_patch[x, y-1].set_pd_dist(3)
                self.dist[4*patch_ind+1, 4*(patch_ind-1)+3] = 0
                self.dist[4*(patch_ind-1)+3, 4*patch_ind+1] = 0
            if x<self.height-1:
                self.mat_patch[x+1, y].set_pd_dist(0)
                self.dist[4*patch_ind+2, 4*(patch_ind+self.width)] = 0
                self.dist[4*(patch_ind+self.width), 4*patch_ind+2] = 0
            if y<self.width-1:
                self.mat_patch[x, y+1].set_pd_dist(1)
                self.dist[4*patch_ind+3, 4*(patch_ind+1)+1] = 0
                self.dist[4*(patch_ind+1)+1, 4*patch_ind+3] = 0

    def initialize_patches(self):
        self.cnt_node = 4 * self.height * self.width
        self.node = np.arange(self.cnt_node)
        self.dist = np.ones((self.cnt_node, self.cnt_node)) * np.inf

        v_straight, v_rotation = 1 / self.list_agv[0].max_velocity, self.list_agv[0].rotate_time

        for i in range(self.height):
            for j in range(self.width):
                # generate patches and link four intra-patch nodes
                patch_ind = self.to_ind(i, j)
                patch_dir = (- 2 * (i % 2) + 1, 2 * (j % 2) - 1)
                self.mat_patch[i, j] = patch(patch_ind, patch_dir, v_rotation)

                # link inter-patch nodes
                if i>0:
                    self.dist[4*patch_ind, 4*(patch_ind-self.width)+2] = v_straight
                    self.dist[4*(patch_ind-self.width)+2, 4*patch_ind] = v_straight

                if j>0:
                    self.dist[4*patch_ind+1, 4*(patch_ind-1)+3] = v_straight
                    self.dist[4*(patch_ind-1)+3, 4*patch_ind+1] = v_straight

        # change links for station and surrounding patches
        self.update_station_patch(self.list_ws)
        self.update_station_patch(self.list_ds)

        for i in range(self.height):
            for j in range(self.width):
                patch_ind = self.to_ind(i, j)
                self.dist[4*patch_ind:4*patch_ind+4, 4*patch_ind:4*patch_ind+4] = self.mat_patch[i,j].dist

    def add_package(self, orig, dest, agv = None):
        package = Package(self.list_ws[orig], self.list_ds[dest], self.cnt_p)
        self.cnt_p += 1
        self.list_ws[orig].packages.append(package)
        if agv is not None:
            self.list_agv[agv].assigned_packages.append(package)


    def next_step(self):
        for agv in self.list_agv:
            agv.next_step()
            if agv.state == 'idle':
                if agv.assigned_packages:
                    package = agv.assigned_packages[0]
                    if agv.path:
                        agv.pathfinding(agv.path[-1], package.orig.loc)
                    else:
                        agv.pathfinding(agv.loc, package.orig.loc)
                    agv.pathfinding(agv.path[-1], package.dest.loc)
                    agv.actions.append(('loading', package))
                    agv.actions.append(('unloading', package))
                    agv.state = 'moving'
                    agv.dest = agv.path[0]
                    agv.next_step()

        for ws in self.list_ws:
            ws.next_step()

        self.t += 1
        return

    def plot_warehouse(self):
        fig_warehouse = plt.figure(figsize=(12,12))
        ax = fig_warehouse.add_subplot(111)

        x_ticks = np.arange(0, self.width+1, 1)
        y_ticks = np.arange(-self.height, 1, 1)
        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.grid(color='gray')

        eps = 0.1
        for i in range(self.height):
            for j in range(self.width):
                x, y = [j+0.5, j+eps, j+0.5, j+1-eps], [-i-eps, -i-0.5, -i-1+eps, -i-0.5]

                id1, id2 = np.where(self.mat_patch[i,j].dist<np.inf)
                for k in range(len(id1)):
                    if id1[k]!=id2[k]:
                        ax.arrow(x[id1[k]], y[id1[k]], x[id2[k]]-x[id1[k]], y[id2[k]]-y[id1[k]],
                                 width=0.05,edgecolor='none',alpha=self.mat_patch[i,j].dist[id1[k],id2[k]]/5+0.1)

        ax.set_xlim(0, self.width)
        ax.set_ylim(-self.height, 0)
        return fig_warehouse
