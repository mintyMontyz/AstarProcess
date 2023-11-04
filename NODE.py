
class node(object):
    def __init__(self, x, y, traversable=True):
        self.x = x
        self.y = y
        self.g_cost = 0
        self.h_cost = 0
        self.parent_node = None
        self.traversable = traversable

    def return_f_cost(self):
        return self.h_cost + self.g_cost
