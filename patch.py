import numpy as np

M = np.inf

# keys are directions, values are a tuple of nodes which indicate node connection
# allowed straight direction
CONNECT_STRAIGHT = {
    (1, 1): ([1, 0], [3, 2]),
    (-1, -1): ([3, 2], [1, 0]),
    (1, -1): ([1, 2], [3, 0]),
    (-1, 1): ([3, 0], [1, 2])
}

# allowed ratation direction
CONNECT_ROTATION = {
    (1, 1): ([1, 0], [2, 3]),
    (-1, -1): ([2, 3], [1, 0]),
    (1, -1): ([1, 2], [0, 3]),
    (-1, 1): ([0, 3], [1, 2])
}


class Patch:
    '''
    Patch is the basic element of warehouse map.
    Each patch is split as 4 nodes: 0: up; 1: left; 2: down; 3: right.
    :arg: ind (int): index of patch
    :arg: direction (tuple): allowed one-way direction (horizontal, vertical), align with the patch index
    :arg: dist (matrix): distance matrix of the intra-patch nodes.
    '''

    def __init__(self, ind, direction, v_rotation):
        self.ind = ind
        self.direction = direction
        self.dist = np.ones((4, 4)) * M
        self.dist[CONNECT_STRAIGHT[direction]] = 0
        self.dist[CONNECT_ROTATION[direction]] = v_rotation
        np.fill_diagonal(self.dist, 0)

    def reset_dist(self):
        '''
        For patches where the stations located, reset the distance matrix to unconnected
        '''
        self.dist = np.ones((4, 4)) * M

    def set_pd_dist(self, node):
        '''
        For patches that are used for pick up and drop, create special connection to facilitate shortest path
        :param node: the node {0,1,2,3} that is adjacent to the station patch.
        '''
        # identify whether the node is an origin node or a destination node within this patch
        is_origin = np.median(self.dist, axis=1) < M
        if is_origin[node]:
            # connect the other origin node to this adjacent node
            node_o = np.sum(np.where(is_origin)[0]) - node
        else:
            # connect the origin node which is not on the opposite side to this adjacent node
            tmp = np.where(self.dist[:, node] < M)[0]
            node_o = tmp[(tmp != node) & (tmp != (node + 2) % 4)]

        self.dist[node_o, node] = 0
