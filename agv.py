import numpy as np

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
    :arg loc (tuple/triple): Real-time location of the agv.
    :arg velocity (float): Real-time velocity of the agv.
    :arg heading (tuple): Real-time heading of the agv. {(0,1), (0,-1), (1,0), (-1,0)}.
    :arg state (str): Real-time state of the agv. {'idle', 'moving', 'rotating', 'loading', 'unloading', 'waiting'}

    :arg packages (list[Package]): List of packages loaded.
    :arg actions (list[tuple]): List of load/unload actions. Each tuple include an action (load/unload) and a package
    :arg paths (list[tuple]): List of paths. Each path is a sequence of turning points that describe the path of agv.\
    :arg dest (triple): The location of current destination which is generally the first argument of path.
    :arg occupied_time (int): The pre-occupied time of an action that cannot be finished in 1 unit of time.
    '''

    def __init__(self, x, y, index, n=1):
        self.index = index
        self.max_velocity, self.acceleration, self.deceleration = 1, 0.5, 0.5
        self.unload_time, self.rotate_time = 2, 4
        self.capacity = n

        self.loc = (x, y, 0)
        self.velocity, self.heading = 0, (0, 0)
        self.state = 'idle'

        self.loaded_packages = []
        self.assigned_packages = []
        self.actions = []
        self.paths = []
        self.path = []

        self.dest = ()
        self.occupied_time = 0

    def __str__(self):
        str = f"""
Agv {self.index} real-time information:
State: {self.state}{f' with {self.occupied_time} time left' if self.occupied_time > 0 else ''}
Velocity: {self.velocity}
Location: {self.loc}
Heading: {self.heading}
Next destination: {self.dest}
Path: {self.path}
Loaded packages: {[package.index for package in self.loaded_packages] if self.loaded_packages else 'None'}
"""
        return str
    def next_step(self):
        '''
        A function describes the next-step action of agv.
        :return:
        '''

        # agv is still in progress of some actions (rotating, loading, unloading)
        if self.occupied_time > 0:
            self.occupied_time -= 1
            return

        if self.state in ['idle', 'loading', 'unloading']:
            if self.paths:
                self.state = 'moving'
                self.path = self.paths[0]
                self.dest = self.path[0]
            else:
                self.state = 'idle'
                return
        else:
            # if agv arrives at the target destination, point to the next destination
            if (self.loc[0] == self.dest[0]) & (self.loc[1] == self.dest[1]):
                self.loc = self.dest
                self.path.pop(0)
                self.dest = self.path[0] if self.path else ()

                # load/unload packages at the origin/destination
                if not self.dest:
                    action = self.actions[0]
                    if action == 'loading':
                        package = self.assigned_packages[0]
                        self.occupied_time += package.orig.add_to_queue(package)
                        self.state = package.state = 'loading'
                        self.loaded_packages.append(package)
                        self.assigned_packages.remove(package)
                    elif action == 'unloading':
                        package = self.loaded_packages[0]
                        self.state, package.state = 'unloading', 'completed'
                        self.occupied_time += self.unload_time - 1
                        self.loaded_packages.remove(package)

                    self.actions.pop(0)
                    self.paths.pop(0)
                    return

        if self.dest:
            # the location of agv and the current destination should be in the same row/column
            assert (self.dest[0] == self.loc[0]) | (self.dest[1] == self.loc[1])
            heading_target = (np.sign(self.dest[0] - self.loc[0]), np.sign(self.dest[1] - self.loc[1]))
            # if heading does not align with current destination, rotate.
            if heading_target != self.heading:
                assert self.velocity == 0
                self.state = 'rotating'
                # turn 90/180
                cnt_turn = max(abs(heading_target[0] - self.heading[0]), abs(heading_target[1] - self.heading[1]))
                self.occupied_time = self.rotate_time * cnt_turn - 1
                self.heading = (np.sign(self.dest[0] - self.loc[0]), np.sign(self.dest[1] - self.loc[1]))
                return

        self.state = 'moving'
        _, self.velocity, dist_move = self.move(
            self.velocity, abs(self.loc[0] - self.dest[0]) + abs(self.loc[1] - self.dest[1]))
        self.loc = (self.loc[0] + self.heading[0] * dist_move, self.loc[1] + self.heading[1] * dist_move, )
        return

    def move(self, velocity, dist_target):
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