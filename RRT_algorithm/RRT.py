import argparse
import pygame
import time

from RRTbasePY import RRTGraph 
from RRTbasePY import RRTMap

from typing import Tuple


SLOW_MO_SLEEP: float = 0.2


def is_quit_event(event: pygame.event.Event):
    return event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE


def run(map: RRTMap,
        graph: RRTGraph,
        goal: Tuple[float, float],
        control: bool = False,
        slow: bool = False,
        slowmo_sleep: float = SLOW_MO_SLEEP):
    '''
    Run the simulation with the ability to slow down, or step through the execution
    '''
    map.drawMap()
    running: bool = True
    iteration: int = 0
    while running and (not graph.path_to_goal()):
        event: pygame.event.Event
        for event in pygame.event.get():
            running = not is_quit_event(event)

        if iteration % 5 == 0:
            X, Y, Parent = graph.bias(goal)
            # TODO[RS]: move this to the map class as a function and call it without referencing class constants
            pygame.draw.circle(map.map, map.GRAY, (X[-1], Y[-1]), map.NODE_RAD + 2, 0)
            pygame.draw.line(map.map, map.BLUE, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.EDGE_THICKNESS)
        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.GRAY, (X[-1], Y[-1]), map.NODE_RAD + 2, 0)
            pygame.draw.line(map.map, map.BLUE, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.EDGE_THICKNESS)

        if iteration % 5 == 0:
            pygame.display.update()
            if control:
                event: pygame.event.Event = pygame.event.wait(0)
                running = not is_quit_event(event)
            elif slow:
                time.sleep(slowmo_sleep)
        iteration += 1

    #smoothed_path_coords = graph.optimize_path()
    #map.drawPath(graph.getPathCoords(),smoothed_path_coords)
    map.drawPath_1(graph.getPathCoords())
    pygame.display.update()


def main(args):
    dimensions = (750, 500)
    start = (100, 250)
    goal = (600, 250)
    obsdim = 31
    obstacles = [[275,300], [275,200], [375,125], [375,375], [475,300], [475,200]]
    prohibited_zone = [275,200,475,300]
    pygame.init()

    map = RRTMap(start, goal, dimensions, obsdim, obstacles, prohibited_zone)
    graph = RRTGraph(start, goal, dimensions, obsdim, obstacles, prohibited_zone)

    msg: str = '-' * 30 + '\n'
    msg += 'Usage:\n'
    msg += '\tr: restart the sinulation in full speed\n'
    msg += '\ts: restart the sinulation in slow motion [200 ms step]\n'
    msg += '\tc: restart the sinulation in a controlled step [press key to advance]\n'

    print(msg)

    controlled: bool = args.controlled
    slow_motion: bool = args.slowmo

    while True:
        run(map, graph, goal, controlled, slow_motion, args.sleep)
        
        controlled = False
        slow_motion = False
        e: pygame.event.Event = pygame.event.wait(0)
        if e.type == pygame.KEYDOWN and e.key == pygame.K_r:
            print('Restarting the simulation')
        elif e.type == pygame.KEYDOWN and e.key == pygame.K_c:
            print('Running in controlled step')
            controlled = True
        elif e.type == pygame.KEYDOWN and e.key == pygame.K_s:
            print('Running in slow motion')
            slow_motion = True
        else:
            pygame.quit()
            break

        map.reset(start, goal, dimensions, obsdim, obstacles, prohibited_zone)
        graph.reset(start, goal, dimensions, obsdim, obstacles, prohibited_zone)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Paramaters")
    parser.add_argument("-s", "--slowmo", action="store_true",
                        help="Slow down the simulation")
    parser.add_argument("-l", "--sleep", type=float, default=SLOW_MO_SLEEP,
                        help=f"If slowmo is enabled, the sleep period [default: {SLOW_MO_SLEEP}]")
    parser.add_argument("-c", "--controlled", action="store_true",
                        help="Step through the simulation")
    args = parser.parse_args()
    main(args)
