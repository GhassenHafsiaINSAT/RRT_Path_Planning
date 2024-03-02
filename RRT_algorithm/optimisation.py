import math


RAYON_OBSTACLE = 65
RAYON_ROBOT = 150


def Path_Optimizer(PointsARR=None, ObstaclesARR=None):
    if PointsARR is None:
        PointsARR = [[]]
    if ObstaclesARR is None:
        ObstaclesARR = [[]]

    ShortCut_Bool = False
    for startpoint in PointsARR:
        for nextpoint in PointsARR[len(PointsARR) - 1:PointsARR.index(startpoint):-1]:
            if ShortCut_Bool:
                if nextpoint == PointsARR[len(PointsARR) - 1] :
                    del PointsARR[PointsARR.index(startpoint) + 1:len(PointsARR) - 1]
                    break
                else:
                    del PointsARR[PointsARR.index(startpoint) + 1:PointsARR.index(nextpoint) + 1]
                    break
            for obstacle in ObstaclesARR:
                ShortCut_Bool = optimizer(startpoint[0], startpoint[1],
                                          nextpoint[0], nextpoint[1],
                                          obstacle[0], obstacle[1],
                                          RAYON_ROBOT, RAYON_OBSTACLE)
                if not ShortCut_Bool:
                    break


class InfiniteSlope(Exception):
    pass


def optimizer(current_x_robot,
              current_y_robot,
              target_x_robot,
              target_y_robot,
              x_obstacle,
              y_obstacle,
              rayon_robot,
              rayon_obstacle):

    try:
        if current_x_robot == target_x_robot:
            raise InfiniteSlope
        else:
            # Pente de la droite passant le center actuel et le centre souhaité
            Slope_CurrentTargetCenter = (target_y_robot - current_y_robot) / (target_x_robot - current_x_robot)

            # Ordonnée à l'origine de la droite passant le center actuel et le centre souhaité
            Offset_CurrentTargetCenter = current_y_robot - Slope_CurrentTargetCenter * target_x_robot

            # Distance entre centre obstacle et la trajectoire du centre du robot
            Dist_Obstacle_CenterTraject = math.fabs(-Slope_CurrentTargetCenter * x_obstacle + y_obstacle - Offset_CurrentTargetCenter) / math.sqrt(Slope_CurrentTargetCenter ** 2 + Offset_CurrentTargetCenter ** 2)

            if Dist_Obstacle_CenterTraject >= (rayon_robot + rayon_obstacle):
                return True 

            # Distance entre le centre actuel et le centre souhaité
            DistCurrent_TargetCenter = math.sqrt((current_x_robot - target_x_robot) ** 2 + (target_y_robot - current_y_robot) ** 2)

            # Distance maximale entre le centre du robot et le centre de l'obstacle pour laquelle le projeté appartient au segment formé par le centre actuel et souhaité
            MaxDist_ObsInSegment = math.sqrt(Dist_Obstacle_CenterTraject ** 2 + DistCurrent_TargetCenter ** 2)

            # Distance réelle entre centre actuel et centre obstacle
            DistCurrentCenter_Obst = math.sqrt((current_x_robot - x_obstacle) ** 2 + (current_y_robot - y_obstacle) ** 2)

            # Distance réelle entre centre souhaité et centre obstacle
            DistTargetCenter_Obst = math.sqrt((target_x_robot - x_obstacle) ** 2 + (target_y_robot - y_obstacle) ** 2)

            if DistCurrentCenter_Obst >= MaxDist_ObsInSegment:
                return True
            
            if DistTargetCenter_Obst >= MaxDist_ObsInSegment:
                return True

            return False

    except InfiniteSlope:
        Dist_Obstacle_CenterTraject = math.fabs(current_x_robot - x_obstacle)
        if Dist_Obstacle_CenterTraject >= (rayon_robot + rayon_obstacle):
            return True

        DistCurrent_TargetCenter = math.fabs(target_y_robot - current_y_robot)
        MaxDist_ObsInSegment = math.sqrt(Dist_Obstacle_CenterTraject ** 2 + DistCurrent_TargetCenter ** 2)
        DistCurrentCenter_Obst = math.sqrt((current_x_robot - x_obstacle) ** 2 + (current_y_robot-y_obstacle) ** 2)
        DistTargetCenter_Obst = math.sqrt((target_x_robot - x_obstacle) ** 2 + (target_y_robot-y_obstacle) ** 2)
        if DistCurrentCenter_Obst >= MaxDist_ObsInSegment:
            return True
        elif DistTargetCenter_Obst >= MaxDist_ObsInSegment:
            return True
        else:
            return False
        # tej ya 3oss
