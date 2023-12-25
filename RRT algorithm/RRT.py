import pygame
from RRTbasePY import RRTGraph 
from RRTbasePY import RRTMap
#import optimisation

def main(): 
    dimensions=(600,800)
    start=(50,50)
    goal=(510,510)
    obsdim=30
    obsnum=50
    iteration=0
    obstacles=[[150,150],[260,260],[390,390]]

    pygame.init()
    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGraph(start,goal,dimensions,obsdim,obstacles)

    obstacles=graph.addObstacles(obstacles)
    map.drawMap(obstacles)

    """"
    while(True):
        x,y = graph.sample_envir()
        n=graph.number_of_nodes()
        graph.add_node(n,x,y)
        graph.add_edge(n-1,n)
        x1,y1=graph.x[n],graph.y[n]
        x2,y2=graph.x[n-1],graph.y[n-1]
        if (graph.isFree()):
            pygame.draw.circle(map.map,map.red,(graph.x[n],graph.y[n]),map.nodeRad,map.nodeThikness)
            if not graph.crossObstacle(x1,x2,y1,y2):
                pygame.draw.line(map.map,map.blue,(x1,y1),(x2,y2),map.edgeThikness)

        pygame.display.update()
    """
    #while (iterarion<500):
    while (not graph.path_to_goal()):
        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1],Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1],Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
        else :    
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1],Y[-1]), map.nodeRad+2, 0)
            pygame.draw.line(map.map, map.blue, (X[-1],Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
        
        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1
    map.drawPath(graph.getPathCoords()) 
    map.drawPath_1(graph.getPathCoords_1())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)
    
if __name__ == '__main__':
    main()