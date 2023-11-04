import pygame as pg
import pygame.draw
from time import sleep

from Astar import run_a_star

pg.init()
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 800
screen = pg.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

WHITE, BLACK, RED, GREEN, PURPLE, BLUE, YELLOW = (255, 255, 255), (0, 0, 0), (255, 0, 0), (0, 255, 0), (255, 0, 247), (0, 43, 255), (255, 205, 0)

node_count_x, node_count_y = 12, 12

clock = pygame.time.Clock()

class drawn_node(object):
    def __init__(self, x_, y_, size_x, size_y):
        self.x = x_
        self.y = y_
        self.size_x = size_x
        self.size_y = size_y
        self.colour = None

    def update(self, colour_code):
        if colour_code == 0:
            self.colour = WHITE  # nothing
        elif colour_code == 1:
            self.colour = GREEN  # open
        elif colour_code == 2:
            self.colour = RED  # closed
        elif colour_code == 3:
            self.colour = BLACK  # non traversable
        elif colour_code == 4:
            self.colour = PURPLE  # start
        elif colour_code == 5:
            self.colour = BLUE  # end
        elif colour_code == 6:
            self.colour = YELLOW  # path

    def draw(self, screen_):
        pg.draw.rect(screen_, self.colour, (self.x, self.y, self.size_x, self.size_y))


def create_drawn_nodes(space_between_nodes):
    size_x = SCREEN_WIDTH//node_count_x - space_between_nodes
    size_y = SCREEN_HEIGHT//node_count_y - space_between_nodes

    drawn_nodes = [[drawn_node(x * (size_x + space_between_nodes) + space_between_nodes//2, y * (size_y + space_between_nodes) + space_between_nodes//2, size_x, size_y) for x in range(node_count_x)] for y in range(node_count_y)]

    return drawn_nodes


draw_nodes = create_drawn_nodes(5)


def colour_nodes(path, matrix, open_, closed, start_, end_):
    for x in range(len(draw_nodes)):
        for y in range(len(draw_nodes[x])):
            draw_nodes[x][y].update(0)
            if matrix[x][y] in open_:
                draw_nodes[x][y].update(1)
            if matrix[x][y] in closed:
                draw_nodes[x][y].update(2)
            if matrix[x][y] in path:
                draw_nodes[x][y].update(6)
            if not matrix[x][y].traversable:
                draw_nodes[x][y].update(3)
            if matrix[x][y] == start_:
                draw_nodes[x][y].update(4)
            if matrix[x][y] == end_:
                draw_nodes[x][y].update(5)


def keypress():
    keys = pygame.key.get_pressed()
    if keys[pg.K_SPACE]:
        return pg.mouse.get_pos(), 0
    elif keys[pg.K_1]:
        return pg.mouse.get_pos(), 1
    elif keys[pg.K_9]:
        return pg.mouse.get_pos(), 2
    elif keys[pg.K_RETURN]:
        return False, 3
    return None, None


def setup_nodes(mouse_pos, action_type):
    if mouse_pos is not None:
        if action_type is not None and action_type != 3:
            for i in range(len(draw_nodes)):
                for j in range(len(draw_nodes[i])):
                    if draw_nodes[i][j].x <= mouse_pos[0] <= draw_nodes[i][j].x + draw_nodes[i][j].size_x and draw_nodes[i][j].y <= mouse_pos[1] <= draw_nodes[i][j].y + draw_nodes[i][j].size_y:
                        if action_type == 0:
                            draw_nodes[i][j].update(3)
                            return i, j
                        elif action_type == 1:
                            draw_nodes[i][j].update(4)
                        elif action_type == 2:
                            draw_nodes[i][j].update(5)


def show():
    screen.fill(BLACK)
    for x in range(len(draw_nodes)):
        for y in range(len(draw_nodes[x])):
            draw_nodes[x][y].draw(screen)

    pg.display.flip()


obstacles = []


def main():
    finished_ = False
    setup = True
    FPS = 60
    open_, closed, matrix = [], [], []
    start_vec = (0, 0)
    end_vec = (0, 0)

    for x in draw_nodes:
        for y in x:
            y.update(0)
    running = True
    while running:
        clock.tick(FPS)
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
        if setup:
            FPS = 60
            matrix.clear()
            open_.clear()
            closed.clear()
            finished_ = False
            if setup_nodes(keypress()[0], keypress()[1]) is not None:
                if setup_nodes(keypress()[0], keypress()[1]) not in obstacles:
                    obstacles.append(setup_nodes(keypress()[0], keypress()[1]))
            if keypress()[0] is not None:
                for i in range(len(draw_nodes)):
                    for j in range(len(draw_nodes[i])):
                        if keypress()[1] == 1 or keypress()[1] == 2:
                            if draw_nodes[i][j].x <= keypress()[0][0] <= draw_nodes[i][j].x + draw_nodes[i][j].size_x and draw_nodes[i][j].y <= keypress()[0][1] <= draw_nodes[i][j].y + draw_nodes[i][j].size_y:
                                 if keypress()[1] == 1:
                                    start_vec = (j, i)
                                 elif keypress()[1] == 2:
                                     end_vec = (j, i)

            if keypress()[1] == 3:
                setup = keypress()[0]
                sleep(0.5)
        else:
            if not finished_:
                FPS = 10
                path, matrix, open_, closed, start_, end_, finished_ = run_a_star(start_vec, end_vec, (node_count_x, node_count_y), obstacles, matrix, open_, closed)
                colour_nodes(path, matrix, open_, closed, start_, end_)
            else:
                FPS = 60
            if keypress()[1] == 3 and not keypress()[0]:
                setup = True
                for x in draw_nodes:
                    for y in x:
                        y.update(0)
                obstacles.clear()
                sleep(0.5)

        show()
    pg.quit()


if __name__ == "__main__":
    main()
