import math

def Path_Optimizer(PointsARR=[[]],ObstaclesARR=[[]]) :
    RAYON_OBSTACLE=65
    RAYON_ROBOT=150

    ShortCut_Bool= False
    for startpoint in PointsARR:
        for nextpoint in PointsARR[len(PointsARR)-1:PointsARR.index(startpoint):-1]:
            if ShortCut_Bool == True:
                if nextpoint == PointsARR[len(PointsARR)-1] :
                    del PointsARR[PointsARR.index(startpoint)+1:len(PointsARR)-1]
                    break
                else :
                    del PointsARR[PointsARR.index(startpoint)+1:PointsARR.index(nextpoint)+1]
                    break
            for obstacle in ObstaclesARR:
                ShortCut_Bool=optimizer(startpoint[0],startpoint[1],nextpoint[0],nextpoint[1],obstacle[0],obstacle[1],RAYON_ROBOT,RAYON_OBSTACLE)
                if ShortCut_Bool == False:
                    break
                    

class InfiniteSlope(Exception):
    pass

def optimizer(CurrentX_robot,CurrentY_robot,TargetX_robot,TargetY_robot,X_obstacle,Y_obstacle,Rayon_robot,Rayon_obstacle):

    try:
        if CurrentX_robot == TargetX_robot:
            raise InfiniteSlope
        else :
            Slope_CurrentTargetCenter=(TargetY_robot-CurrentY_robot)/(TargetX_robot-CurrentX_robot) #Pente de la droite passant le center actuel et le centre souhaité
            Offset_CurrentTargetCenter=CurrentY_robot-Slope_CurrentTargetCenter*TargetX_robot #Ordonnée à l'origine de la droite passant le center actuel et le centre souhaité
            Dist_Obstacle_CenterTraject=math.fabs(-Slope_CurrentTargetCenter*X_obstacle+Y_obstacle-Offset_CurrentTargetCenter)/math.sqrt(Slope_CurrentTargetCenter**2+Offset_CurrentTargetCenter**2) #Distance entre centre obstacle et la trajectoire du centre du robot
            if Dist_Obstacle_CenterTraject >= (Rayon_robot+Rayon_obstacle):
                return True 
            else:
                DistCurrent_TargetCenter=math.sqrt((CurrentX_robot-TargetX_robot)**2+(TargetY_robot-CurrentY_robot)**2) #Distance entre le centre actuel et le centre souhaité
                MaxDist_ObsInSegment=math.sqrt(Dist_Obstacle_CenterTraject**2+DistCurrent_TargetCenter**2) #Distance maximale entre le centre du robot et le centre de l'obstacle pour laquelle le projeté appartient au segment formé par le centre actuel et souhaité 
                DistCurrentCenter_Obst=math.sqrt((CurrentX_robot-X_obstacle)**2+(CurrentY_robot-Y_obstacle)**2) #Distance réelle entre centre actuel et centre obstacle
                DistTargetCenter_Obst=math.sqrt((TargetX_robot-X_obstacle)**2+(TargetY_robot-Y_obstacle)**2) ##Distance réelle entre centre souhaité et centre obstacle
                if DistCurrentCenter_Obst >= MaxDist_ObsInSegment:
                    return True
                elif DistTargetCenter_Obst >= MaxDist_ObsInSegment:
                    return True
                else :
                    return False
    except InfiniteSlope:
        Dist_Obstacle_CenterTraject=math.fabs(CurrentX_robot-X_obstacle)
        if Dist_Obstacle_CenterTraject >= (Rayon_robot+Rayon_obstacle):
                return True 
        else:
            DistCurrent_TargetCenter=math.fabs(TargetY_robot-CurrentY_robot)
            MaxDist_ObsInSegment=math.sqrt(Dist_Obstacle_CenterTraject**2+DistCurrent_TargetCenter**2)
            DistCurrentCenter_Obst=math.sqrt((CurrentX_robot-X_obstacle)**2+(CurrentY_robot-Y_obstacle)**2) 
            DistTargetCenter_Obst=math.sqrt((TargetX_robot-X_obstacle)**2+(TargetY_robot-Y_obstacle)**2) 
            if DistCurrentCenter_Obst >= MaxDist_ObsInSegment:
                return True
            elif DistTargetCenter_Obst >= MaxDist_ObsInSegment:
                return True
            else :
                return False