import numpy as np

# keys are directions, values are node connection with the former two keep straight and latter two rotate
CONNECT_STRAIGHT = {
    (1, 1): ([1, 2], [3, 0]),
    (-1, -1): ([3, 0], [1, 2]),
    (1, -1): ([1, 0], [3, 2]),
    (-1, 1): ([3, 2], [1, 0])
}

CONNECT_ROTATION = {
    (1, 1): ([1, 2], [0, 3]),
    (-1, -1): ([0, 3], [1, 2]),
    (1, -1): ([1, 0], [2, 3]),
    (-1, 1): ([2, 3], [1, 0])
}

class patch:
    def __init__(self, ind, direction, v_rotation):
        '''
        define patch
        each patch is split as 4 nodes: 0: up; 1: left; 2: down; 3: right.
        :param ind (int): the index of patch
        :param direction (tuple): allowed one-way direction (horizontal, vertical)
        '''
        self.ind = ind
        self.direction = direction
        self.v_rotation = v_rotation
        self.dist = np.ones((4,4)) * np.inf
        self.dist[CONNECT_STRAIGHT[direction]] = 0
        self.dist[CONNECT_ROTATION[direction]] = v_rotation
        np.fill_diagonal(self.dist, 0)

    def reset_dist(self):
        self.dist = np.ones((4,4)) * np.inf

    def set_pd_dist(self, ind):
        is_inout = np.median(self.dist, axis=1) < np.inf
        if is_inout[ind]:
            ind2 = np.sum(np.where(is_inout)[0])-ind
            self.dist[ind2, ind] = 0
        else:
            tmp = np.where(self.dist[:,ind]<np.inf)[0]
            ind2 = tmp[(tmp!=ind) & (tmp!=(ind+2)%4)]
            self.dist[ind2, ind] = 0
