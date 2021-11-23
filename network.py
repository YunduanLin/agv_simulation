import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from copy import deepcopy
from agent import *
from agv import Agv
from patch import Patch
import os
import imageio

plt.ioff()

MARKER = {(0,0):'o', (0,1):'>', (0,-1):'<', (1,0):'v', (-1,0):'^'}

class Warehousemap:
    '''
    :arg: height/width (int): the number of patches representing the height/width of warehousemap
    :arg: list_ws/list_ds/list_agv (list): the list of workstations/dropstations/agvs
    :arg: cnt_ws/cnt_ds/cnt_agv/cnt_package/cnt_node (int):
            the total number of workstations/dropstations/agvs/packages/nodes
    :arg: mat_patch (matrix): the matrix
    :arg: dist (matrix): the distance matrix between nodes
    :arg: t (int): timer
    '''

    def __init__(self, height, width, agents, packages, dir_path):
        self.height, self.width = height, width
        self.mat_patch = np.empty((self.height, self.width), dtype=object)

        self.list_ws, self.list_ds, self.list_agv, self.list_package = [], [], [], []
        self.cnt_ws, self.cnt_ds, self.cnt_agv, self.cnt_package = 0, 0, 0, 0

        self.t = 0
        self.dir_path = dir_path

        for agent in agents:
            self.add_agent(**agent)
        for package in packages:
            self.add_package(**package)

        self.initialize_patches()
        self.generate_graph()

    def add_agent(self, x, y, agent_type, n=None):
        '''
        Add agents (workstation/dropstation/agv)
        :param x/y (int): location
        :param agent_type: identify the type of agents
        :param n: capacity for agv
        '''
        if agent_type == 'workstation':
            ws = Workstation(x, y, self.cnt_ws)
            self.list_ws.append(ws)
            self.cnt_ws += 1
        elif agent_type == 'dropstation':
            ds = Dropstation(x, y, self.cnt_ds)
            self.list_ds.append(ds)
            self.cnt_ds += 1
        elif agent_type == 'agv':
            agv = Agv(x, y, self.cnt_agv, n)
            self.list_agv.append(agv)
            self.cnt_agv += 1

    def add_package(self, orig, dest, agv=None):
        '''
        Add packages
        :param orig: the index of workstation where the package will be picked up
        :param dest: the index of dropstation where the package will be dropped off
        :param agv: the index of agv which this package is assigned to
        '''
        package = Package(self.list_ws[orig], self.list_ds[dest], self.cnt_package)
        self.list_package.append(package)
        self.cnt_package += 1
        self.list_ws[orig].packages.append(package)
        if agv is not None:
            self.list_agv[agv].assigned_packages.append(package)

    # convert patch coordinate to patch index
    def coord_to_ind(self, coord):
        return int(coord[0] * self.width + coord[1])

    # convert patch index to patch coordinate
    def ind_to_coord(self, ind):
        return ind // self.width, ind % self.width

    # convert node coordinate to node index
    def coord3d_to_node(self, coord3d):
        return self.coord_to_ind(coord3d[:2]) * 4 + coord3d[2]

    # convert node index to node coordinate
    def node_to_coord3d(self, node):
        x, y = self.ind_to_coord(node // 4)
        return x, y, node % 4

    def int_loc(self, coord):
        return int(coord[0]), int(coord[1])

    # udpate intra-patch links of station and surrounding patches
    def update_station_patch(self, list_s):
        for s in list_s:
            x, y = s.loc
            patch_ind = self.coord_to_ind((x, y))
            # update station patch
            self.mat_patch[x, y].reset_dist()
            # update the surrounding patches
            if x > 0:
                self.mat_patch[x - 1, y].set_pd_dist(2)
                self.dist[4 * (patch_ind - self.width) + 2, 4 * patch_ind] = 0
            if y > 0:
                self.mat_patch[x, y - 1].set_pd_dist(3)
                self.dist[4 * (patch_ind - 1) + 3, 4 * patch_ind + 1] = 0
            if x < self.height - 1:
                self.mat_patch[x + 1, y].set_pd_dist(0)
                self.dist[4 * (patch_ind + self.width), 4 * patch_ind + 2] = 0
            if y < self.width - 1:
                self.mat_patch[x, y + 1].set_pd_dist(1)
                self.dist[4 * (patch_ind + 1) + 1, 4 * patch_ind + 3] = 0

    # initialize all patches for the grids
    def initialize_patches(self):
        self.cnt_node = 4 * self.height * self.width
        self.dist = np.ones((self.cnt_node, self.cnt_node)) * np.inf

        d_straight, d_rotation = 1 / self.list_agv[0].max_velocity, self.list_agv[0].rotate_time

        for i in range(self.height):
            for j in range(self.width):
                # generate patches and link four intra-patch nodes
                patch_ind = self.coord_to_ind((i, j))
                patch_dir = (- 2 * (i % 2) + 1, - 2 * (j % 2) + 1)
                self.mat_patch[i, j] = Patch(patch_ind, patch_dir, d_rotation)

                # link inter-patch nodes
                if i > 0:
                    self.dist[4 * patch_ind, 4 * (patch_ind - self.width) + 2] = d_straight
                    self.dist[4 * (patch_ind - self.width) + 2, 4 * patch_ind] = d_straight
                if j > 0:
                    self.dist[4 * patch_ind + 1, 4 * (patch_ind - 1) + 3] = d_straight
                    self.dist[4 * (patch_ind - 1) + 3, 4 * patch_ind + 1] = d_straight

        # change intra-patch links for station and surrounding patches
        self.update_station_patch(self.list_ws)
        self.update_station_patch(self.list_ds)

        # assign intra-patch distance matrix to the map distance matrix
        for i in range(self.height):
            for j in range(self.width):
                patch_ind = self.coord_to_ind((i, j))
                self.dist[4 * patch_ind:4 * patch_ind + 4, 4 * patch_ind:4 * patch_ind + 4] = self.mat_patch[i, j].dist

    # generate networkx graph for the entire layout
    def generate_graph(self):
        tmp = (self.dist < np.inf).astype(int)
        np.fill_diagonal(tmp, 0)
        self.nx_graph = nx.from_numpy_matrix(tmp, create_using=nx.DiGraph)
        for e in self.nx_graph.edges:
            nx.set_edge_attributes(self.nx_graph, {e: {'cost': self.dist[e]}})

    #
    def pathfinding(self, coord3d_orig, coord_dest, end_at_station=True):
        '''
        Find the shortest path from an original node to a destination patch and convert it to a path
        :param coord3d_orig (triple): the node coordinate of the origin which implies the direction
        :param coord_dest (tuple): the patch coordinate of the destination station
                which does not determine the final arrived nodes
        :return: agv_path (list):
        '''
        node_orig = self.coord3d_to_node(coord3d_orig)
        ind_dest = self.coord_to_ind(coord_dest)
        shortest_path_length, shortest_path = np.inf, []

        # Use astar algorithm in networkx to find the shortest path
        for i in range(4):
            # If there's not path between two nodes, the algorithm will raise NetworkXNoPath error.
            try:
                path, path_length = nx.astar_path(self.nx_graph, node_orig, ind_dest * 4 + i, weight='cost'), \
                                    nx.astar_path_length(self.nx_graph, node_orig, ind_dest * 4 + i, weight='cost')
                if (path_length < shortest_path_length) | (
                        path_length == shortest_path_length) & (len(path) < len(shortest_path)):
                    shortest_path_length, shortest_path = path_length, path
            except nx.NetworkXNoPath:
                pass

        # Identify the nodes where the agv needs rotation
        agv_path = [coord3d_orig]
        i = 1
        while i < len(shortest_path) - 1:
            x, y, z_1 = self.node_to_coord3d(shortest_path[i])
            _, _, z_2 = self.node_to_coord3d(shortest_path[i + 1])
            if z_1 != (z_2 + 2) % 4:
                agv_path.append((x, y, z_1))
            i += 1
        # If the agv needs to load/unload at the destination, the path do not need to include the last few dummy nodes.
        if end_at_station:
            dest_of_path = self.node_to_coord3d(shortest_path[-3])
            if (dest_of_path[0] != agv_path[-1][0]) | (dest_of_path[1] != agv_path[-1][1]):
                agv_path.append(dest_of_path)
        else:
            agv_path.append(self.node_to_coord3d(shortest_path[-1]))
        return agv_path[1:]

    # Find the route for each agv according to the assigned packages
    def route_planning(self):
        for agv in self.list_agv:
            loc_start = agv.loc
            for package in agv.assigned_packages:
                agv.paths.append(self.pathfinding(loc_start, package.orig.loc))
                agv.paths.append(self.pathfinding(agv.paths[-1][-1], package.dest.loc))
                loc_start = agv.paths[-1][-1]
                agv.actions = agv.actions + ['loading', 'unloading']
            agv.paths.append(self.pathfinding(loc_start, agv.loc_return, False))
            agv.actions.append('completed')

    def is_loop_blocked(self, loc, dir, mat_blocked):
        heading = dir
        next_loc = (loc[0]+heading[0], loc[1]+heading[1])
        is_blocked = 0
        for i in range(3):
            try:
                is_blocked = is_blocked + mat_blocked[next_loc]
                tmp = self.mat_patch[next_loc].direction
                heading = (tmp[0] - heading[0], tmp[1] - heading[1])
                next_loc = (next_loc[0]+heading[0], next_loc[1]+heading[1])
            except IndexError:
                pass
        return is_blocked==3

    def next_step(self):
        print(f'============== Time t={self.t} ==============')
        mat_blocked = np.zeros((self.height, self.width), dtype=bool)
        for ws in self.list_ws:
            mat_blocked[ws.loc] = True
        for ds in self.list_ds:
            mat_blocked[ds.loc] = True
        for agv in self.list_agv:
            if agv.state=='loading':
                package = agv.loaded_packages[-1]
                ws = package.orig
                if not ws.queue:
                    continue
                if ws.queue.index(package) < ws.buffer_capacity:
                    continue
            if agv.state!='completed':
                mat_blocked[self.int_loc(agv.loc[:2])] = True

        for agv in self.list_agv:
            print(agv)
            tmp_agv = deepcopy(agv)
            tmp_agv.next_step()
            cur_loc = self.int_loc(agv.loc[:2])
            next_loc = self.int_loc(tmp_agv.loc[:2])
            if (cur_loc[0]!=next_loc[0]) | (cur_loc[1]!=next_loc[1]):
                if mat_blocked[next_loc] | self.is_loop_blocked(next_loc, tmp_agv.heading, mat_blocked):
                    print(f'Agv {agv.index} delays at {agv.loc}.')
                    agv.occupied_time = 1
                    agv.velocity = 0
                else:
                    mat_blocked[next_loc] = True
            agv.next_step()
        for ws in self.list_ws:
            ws.next_step()

        self.t += 1

    def run_simulation(self):
        is_exist = os.path.exists(self.dir_path)
        if not is_exist:
            os.makedirs(self.dir_path)

        completed = False
        while not completed:
            ax, fig_warehouse = self.visualization()
            fig_warehouse.savefig(f'{self.dir_path}{self.t}.png')
            self.next_step()
            completed = True
            for agv in self.list_agv:
                completed = completed & (agv.state=='completed')
        print('============== CLEARED ==============')

        images = []
        for i in range(self.t):
            images.append(imageio.imread(f'{self.dir_path}{i}.png'))
        imageio.mimsave(f'{self.dir_path}visulization.gif', images)

    def visualization(self):
        ax, fig_warehouse = self.plot_warehouse()
        for agv in self.list_agv:
            x, y = agv.loc[0], agv.loc[1]
            ax.scatter(y+0.5, -x-0.5, color='darkorange',s=2000/self.width, marker=MARKER[agv.heading], zorder=15)
        for ws in self.list_ws:
            x, y = ws.loc[0], ws.loc[1]
            ax.annotate(len(ws.packages), xy=(y+0.5, -x-0.5), xytext=(y+0.45, -x-0.55), color='white', weight='bold')
        for ds in self.list_ds:
            x, y = ds.loc[0], ds.loc[1]
            ax.annotate(len(ds.packages), xy=(y+0.5, -x-0.5), xytext=(y+0.45, -x-0.55), color='white', weight='bold')
        ax.set_title(f't={self.t}')
        return ax, fig_warehouse

    def plot_warehouse(self, show_direction = True):
        fig_warehouse = plt.figure(figsize=(12, 12))
        ax = fig_warehouse.add_subplot(111)

        x_ticks = np.arange(0, self.width + 1, 1)
        y_ticks = np.arange(-self.height, 1, 1)
        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.grid(color='gray')

        for ws in self.list_ws:
            x, y = ws.loc
            ax.fill_between([y, y+1], [-x-1, -x-1], [-x,-x], color='steelblue')

        for ds in self.list_ds:
            x, y = ds.loc
            ax.fill_between([y, y+1], [-x-1, -x-1], [-x,-x], color='lightcoral')

        if show_direction:
            eps = 0.1
            for i in range(self.height):
                for j in range(self.width):
                    x, y = [j + 0.5, j + eps, j + 0.5, j + 1 - eps], [-i - eps, -i - 0.5, -i - 1 + eps, -i - 0.5]

                    id1, id2 = np.where(self.mat_patch[i, j].dist < np.inf)
                    for k in range(len(id1)):
                        if id1[k] != id2[k]:
                            ax.arrow(x[id1[k]], y[id1[k]], x[id2[k]] - x[id1[k]], y[id2[k]] - y[id1[k]],
                                     width=0.05, edgecolor='none', facecolor='gray',
                                     alpha=- self.mat_patch[i, j].dist[id1[k], id2[k]] / 10 + 0.5)

        ax.set_xlim(0, self.width)
        ax.set_ylim(-self.height, 0)
        return ax, fig_warehouse
