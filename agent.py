import numpy as np

big_M = 99999


class Agv:
    '''
    A class for agv in the warehouse.

    constant
    :arg index (int): Index for the specific agv among all agvs in the warehouse.
    :arg max_velocity (float): The largest cruising velocity of agv.
    :arg acceleration (float): The acceleration of agv.
    :arg deceleration (float): The deceleration of agv.
    :arg unload_time (int): The time needed to unload (drop) a package after arriving at the drop station.
    :arg rotate_time (int): The time needed to rotate a 90 degree angle.
    :arg capacity (int): Tha maximum number of packages that can be loaded on agv.

    real-time information
    :arg loc (tuple): Real-time location of the agv.
    :arg velocity (float): Real-time velocity of the agv.
    :arg heading (tuple): Real-time heading of the agv. {(0,1), (0,-1), (1,0), (-1,0)}.
    :arg state (str): Real-time state of the agv. {'idle', 'moving', 'rotating', 'loading', 'unloading', 'waiting'}

    :arg packages (list[Package]): List of packages loaded.
    :arg actions (list[tuple]): List of load/unload actions. Each tuple include an action (load/unload) and a package
    :arg path (list[tuple]): List of critical points (Turning points and action points) that describe the path of agv.
                             Typically, the velocity of agv at these points should be 0.

    :arg dest (tuple): The location of current destination which is generally the first argument of path.
    :arg occupied_time (int): The pre-occupied time of an action that cannot be finished in 1 unit of time.
    '''

    def __init__(self, x, y, index, n=1):
        self.index = index
        self.max_velocity, self.acceleration, self.deceleration = 1, 0.5, 0.5
        self.unload_time, self.rotate_time = 2, 4
        self.capacity = n

        self.loc = (x, y)
        self.velocity, self.heading = 0, (0, 0)
        self.state = 'idle'

        self.loaded_packages = []
        self.assigned_packages = []
        self.actions = []
        self.path = []

        self.dest = ()
        self.occupied_time = 0

    def __str__(self):
        str = f"""
Agv {self.index} real-time information:
State: {self.state}
Velocity: {self.velocity}
Location: {self.loc}
Heading: {self.heading}
Next destination: {self.dest}
"""
        return str
    def next_step(self):
        '''
        A function describes the next-step action of agv.
        :return:
        '''
        # TODO: 1. detect collision and solution
        #       2. what happened if the working station is full
        #       3. load one package at a time, extend to multiple packages

        # the location of agv and the current destination should be in the same row/column
        assert (self.dest[0] == self.loc[0]) | (self.dest[1] == self.loc[1])
        # agv is still in progress of some actions (rotating, loading, unloading)
        if self.occupied_time > 0:
            self.occupied_time -= 1

        else:
            # if heading does not align with current destination, rotate.
            if self.dest:
                heading_target = (np.sign(self.dest[0] - self.loc[0]), np.sign(self.dest[1] - self.loc[1]))
                if heading_target != self.heading:
                    assert self.velocity == 0
                    self.state = 'rotating'
                    # turn 90/180
                    cnt_turn = max(abs(heading_target[0] - self.heading[0]), abs(heading_target[1] - self.heading[1]))
                    self.occupied_time = self.rotate_time * cnt_turn - 1
                    self.heading = (np.sign(self.dest[0] - self.loc[0]), np.sign(self.dest[1] - self.loc[1]))
                    return
            else:
                # path is finished, agv becomes idle
                self.state = 'idle'
                return

            self.state = 'moving'
            _, self.velocity, dist_move = self.move(
                self.velocity, abs(self.loc[0] - self.dest[0]) + abs(self.loc[1] - self.dest[1]))
            self.loc = (self.loc[0] + self.heading[0] * dist_move, self.loc[1] + self.heading[1] * dist_move)

            # if agv arrives at the target destination, point to the next destination
            if self.loc == self.dest:
                self.path.pop(0)
                self.dest = self.path[0] if self.path else ()

                # load/unload packages at the origin/destination
                if self.actions:
                    action, package = self.actions[0]
                    if (action == 'loading') & (self.loc == package.orig.loc):
                        assert package in self.assigned_packages
                        self.occupied_time = package.orig.add_to_queue(package)
                        self.state = package.state = 'loading'
                        self.loaded_packages.append(package)
                        self.assigned_packages.remove(package)
                        self.actions.pop(0)
                    elif (action == 'unloading') & (self.loc == package.dest.loc):
                        assert package in self.loaded_packages
                        self.state, package.state = 'unloading', 'completed'
                        self.occupied_time = self.unload_time
                        self.loaded_packages.remove(package)
                        self.actions.pop(0)

        return

    def move(self, velocity, dist_target):
        t_total = 0
        velocity_end, dist_move = 0, 0

        # the least distance that agv needs to go before decelerating to 0 if it first accelerates to max_velocity
        dist_max_velocity_min = (self.max_velocity + velocity) / 2 * (
                self.max_velocity - velocity) / self.acceleration \
                                + self.max_velocity / 2 * self.max_velocity / self.deceleration

        # when the agv does not need to accelerate to max_velocity
        if dist_max_velocity_min > dist_target:
            # find the max velocity: (v+x)(x-v)/2a + x^2/2d = dist_target
            velocity_u = np.sqrt((dist_target + velocity ** 2 / 2 / self.acceleration) / (
                    1 / 2 / self.acceleration + 1 / 2 / self.deceleration))
            t_acc = (velocity_u - velocity) / self.acceleration
            t_total = t_acc + velocity_u / self.deceleration

            # if the agv can arrive at the target within 1 unit of time
            if t_total <= 1:
                dist_move = dist_target
                t_total = 1
            elif t_acc > 1:  # end within acceleration period
                velocity_end = velocity + self.acceleration * 1
                dist_move = (velocity + velocity_u) / 2 * 1
            else:  # end within deceleration period
                velocity_end = velocity_u - self.deceleration * (1 - t_acc)
                dist_move = (velocity + velocity_u) / 2 * t_acc + (velocity_u + velocity_end) / 2 * (1 - t_acc)
        # when the agv can cruise at the max_velocity
        else:
            t_acc = (self.max_velocity - velocity) / self.acceleration
            t_dec = self.max_velocity / self.deceleration
            t_const = (dist_target - (velocity + self.max_velocity) / 2 * t_acc - self.max_velocity / 2 * t_dec) \
                      / self.max_velocity
            t_total = t_acc + t_dec + t_const

            if t_acc > 1:  # end within acceleration period
                velocity_end = velocity + self.acceleration * 1
                dist_move = (velocity + velocity_end) / 2 * 1
            elif t_acc + t_const > 1:  # end within constant speed period
                velocity_end = self.max_velocity
                dist_move = (velocity + self.max_velocity) / 2 * t_acc + self.max_velocity * (1 - t_acc)
            else:  # end within deceleration speed period
                velocity_end = self.max_velocity - self.deceleration * (1 - t_acc - t_const)
                dist_move = (velocity + self.max_velocity) / 2 * t_acc + self.max_velocity * t_const + \
                            (self.max_velocity + velocity_end) / 2 * (1 - t_acc - t_const)
        return np.ceil(t_total), velocity_end, dist_move

    def find_feasible_interval(self, l_grid, cord):
        '''
        A function calculate the maximum straight-line distance of a starting point on both sides.
        :arg l_grid (List[int]): layout of warehouse in one dimension.
        :arg cord (int): the cordinate of starting point.
        :return: cord_min, cord_max: the cordinate of points on both sides.
        '''
        tmp = np.where(l_grid[:cord] != 0)[0]
        cord_min = (tmp[-1] + 1) if len(tmp) > 0 else 0
        tmp = np.where(l_grid[cord:] != 0)[0]
        cord_max = (cord + tmp[0] - 1) if len(tmp) > 0 else (len(l_grid) - 1)
        return cord_min, cord_max

    def to_ind(self, cord):
        return cord[0] * self.width + cord[1]

    def to_cord(self, ind):
        return (ind // self.width, ind % self.width)

    def to_surrounding(self, cord):
        list_cord = [cord]
        if self.grid[cord]:
            list_cord = []
            if (cord[0] > 0):
                list_cord = list_cord + [(cord[0] - 1, cord[1])] if (self.grid[cord[0] - 1, cord[1]] == 0) else []
            if (cord[1] > 0):
                list_cord = list_cord + [(cord[0], cord[1] - 1)] if (self.grid[cord[0], cord[1] - 1] == 0) else []
            if (cord[0] < self.height - 1):
                list_cord = list_cord + [(cord[0] + 1, cord[1])] if (self.grid[cord[0] + 1, cord[1]] == 0) else []
            if (cord[1] < self.width - 1):
                list_cord = list_cord + [(cord[0], cord[1] + 1)] if (self.grid[cord[0], cord[1] + 1] == 0) else []
        return list_cord

    def generate_shortest_path(self, grid):
        '''
        A function describes find the shortest path for each pair of grids.
        :arg grid (Array): layout of warehouse, indicating where are the stations.
        :return:
        '''
        self.grid = grid
        self.height, self.width = grid.shape
        self.cnt_grid = self.height * self.width
        dist = big_M * np.ones((self.height * self.width, self.height * self.width))
        next_grid = np.repeat([np.arange(0, self.cnt_grid, 1)], self.cnt_grid, axis=0)
        self.dist_straight = [self.move(0, dist)[0] for dist in range(max(self.height, self.width))]

        # initialize straight-line distance for each pair of grids
        for i in range(self.height):
            for j in range(self.width):
                if self.grid[i, j] == 0:  # the grid is not a station
                    ind = self.to_ind((i, j))
                    # horizontal
                    j_min, j_max = self.find_feasible_interval(self.grid[i, :], j)
                    dist[ind, self.to_ind((i, j_min)):self.to_ind((i, j + 1))] = self.dist_straight[:j - j_min + 1][
                                                                                 ::-1]
                    dist[ind, self.to_ind((i, j)):self.to_ind((i, j_max + 1))] = self.dist_straight[:j_max - j + 1]
                    # vertical
                    i_min, i_max = self.find_feasible_interval(self.grid[:, j], i)
                    dist[ind, self.to_ind((i_min, j)):self.to_ind((i + 1, j)):self.width] = self.dist_straight[
                                                                                            :i - i_min + 1][::-1]
                    dist[ind, self.to_ind((i, j)):self.to_ind((i_max + 1, j)):self.width] = self.dist_straight[
                                                                                            :i_max - i + 1]
                else:
                    dist[i, i] = 0

        # use Floyd algorithm to update the shortest distance and path
        for k in range(self.cnt_grid):
            for i in range(self.cnt_grid):
                for j in range(self.cnt_grid):
                    if dist[i, k] + dist[k, j] + self.rotate_time < dist[i, j]:
                        dist[i, j] = dist[i, k] + dist[k, j] + self.rotate_time  # add rotation time at each turning
                        next_grid[i, j] = next_grid[i, k]

        # use the shortest distance of surrounding grids to replace the shortest path between stations.
        for i in range(self.cnt_grid):
            for j in range(self.cnt_grid):
                cord_i, cord_j = self.to_cord(i), self.to_cord(j)
                if (self.grid[cord_i] == 1) | (self.grid[cord_j] == 1):
                    list_cord_i, list_cord_j = self.to_surrounding(cord_i), \
                                               self.to_surrounding(cord_j)
                    list_ind_i, list_ind_j = [self.to_ind(cord) for cord in list_cord_i], \
                                             [self.to_ind(cord) for cord in list_cord_j]
                    dist_pair = np.array([[dist[ind_i, ind_j] for ind_j in list_ind_j] for ind_i in list_ind_i])
                    ind_min = np.argmin(dist_pair)
                    ind_sur_i, ind_sur_j = list_ind_i[ind_min // len(list_ind_j)], list_ind_j[ind_min % len(list_ind_j)]
                    if (i != ind_sur_i):
                        next_grid[i, j] = ind_sur_i
                    elif (ind_sur_i == ind_sur_j):
                        next_grid[i, j] = j
                    else:
                        next_grid[i, j] = next_grid[ind_sur_i, ind_sur_j]

        self.dist, self.next_grid = dist, next_grid

    def pathfinding(self, orig, dest):
        ind_orig, ind_dest = self.to_ind(orig), self.to_ind(dest)
        l_path = []
        ind_tmp = self.next_grid[ind_orig, ind_dest]
        while ind_tmp != ind_dest:
            l_path = l_path + [self.to_cord(ind_tmp)]
            ind_tmp = self.next_grid[ind_tmp, ind_dest]

        self.path = self.path + l_path


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
        self.buffer_capacity = 10

        self.loc = (x, y)
        self.occupied_time = 0
        self.packages = []
        self.queue = []

    def add_to_queue(self, package):
        '''
        Add a package to the queue
        :param package: the package that are ready for picking up
        :return: current agv waiting time
        '''
        if self.queue:
            self.occupied_time = self.process_time + 1  # next step of ws is called after agv
        self.queue.append(package)
        self.packages.remove(package)

        return self.occupied_time + (len(self.queue) - 1) * self.process_time

    def next_step(self):
        '''
        Process the queues and packages
        :return:
        '''
        assert len(self.queue) <= self.buffer_capacity

        if self.queue:
            if self.occupied_time > 1:
                self.occupied_time -= 1
            else:
                self.occupied_time = self.process_time
                self.queue.pop(0)


class Dropstation:
    def __init__(self, x, y, index):
        # position x and y
        self.loc = (x, y)
        self.index = index
