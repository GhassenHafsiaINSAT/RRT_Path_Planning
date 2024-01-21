from RRTbasePY import RRTGraph
from RRTbasePY import RRTMap

def find_path_coordinates(dimensions=(600, 800), start=(50, 50), goal=(510, 510), obsdim=30, obsnum=50, iteration=0, obstacles=[[150, 150], [260, 260], [390, 390]]):
    map = RRTMap(start, goal, dimensions, obsdim, obsnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obstacles)

    obstacles = graph.addObstacles(obstacles)

    while not graph.path_to_goal():
        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
        else:
            X, Y, Parent = graph.expand()

        iteration += 1

    path_coords = graph.getPathCoords()
    #graph.refined_path = graph.optimize_path()
    smoothed_path_coords = graph.optimize_path()
    return smoothed_path_coords

if __name__ == '__main__':
    path_coordinates = find_path_coordinates()
    print("Path Coordinates:", path_coordinates)
