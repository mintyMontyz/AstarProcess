from NODE import node


def find_dist(node_a, node_b):
    dist_x = abs(node_b.x - node_a.x)
    dist_y = abs(node_b.y - node_a.y)

    if dist_x < dist_y:
        dist_x, dist_y = dist_y, dist_x

    return 14 * dist_y + 10 * (dist_x - dist_y)


def node_min_f_cost(matrix):

    if len(matrix) > 0:
        current = matrix[0]
        for i in matrix:
            if i.return_f_cost() < current.return_f_cost() or (i.return_f_cost() == current.return_f_cost() and i.h_cost < current.h_cost):
                current = i
        return current


def return_node_from_pos(x, y, matrix):
    return matrix[y][x]


def get_neighbour(current, matrix):
    neighbours = []

    for x in range(-1, 2):
        for y in range(-1, 2):
            if x == 0 and y == 0:
                continue

            check_x = current.x + x
            check_y = current.y + y

            if 0 <= check_x < len(matrix) and 0 <= check_y < len(matrix[0]):
                neighbours.append(matrix[check_y][check_x])
    return neighbours


def retrace_path(start, end):
    path = []
    current_node = end

    while current_node != start:
        if current_node.parent_node is not None:
            path.append(current_node)
            current_node = current_node.parent_node

    return path
