from RRTbasePY import RRTGraph

def find_path_coordinates( start=(88, 245), goal=(675, 75),  dimensions=(750, 500), my_dim = 38, obsdim=31, obstacles=[[250, 175], [250, 325], [375, 125], [375, 375], [500, 175], [500, 325], [224,206]], adv_obs = [750,250],prohibited_zone = [0, 0, 10, 10] ):
    graph = RRTGraph(start, goal, dimensions, my_dim, obsdim, obstacles,adv_obs, prohibited_zone)
    iteration = 0

    while not graph.path_to_goal():
        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
        else:
            X, Y, Parent = graph.expand()

        iteration += 1

    smoothed_path_coords = graph.optimize_path()
    return smoothed_path_coords

if __name__ == '__main__':
    path_coordinates = find_path_coordinates()
