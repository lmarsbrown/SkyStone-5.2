package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Utils.Transform;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class Path {
    private List<Transform> points = new ArrayList<>();
    private Transform end ;
    public void addPoint(Transform point)
    {
        points.add(point);
    }
    public void setEndPoint(Transform point)
    {
        end = point;
    }
    protected void setPathFollowVec(Robot_Controller robot)
    {
        if(robot.checkOnPath(points.get(points.size()-1),end))
        {
            if(Math.hypot(robot.robot.pos.x-end.x,robot.robot.pos.y-end.y)<robot.lookahead)
            {
                robot.ending = true;
                robot.setPursuitPath(robot.robot.pos,end);
            }
            else
            {
                robot.setPursuitPath(points.get(points.size()-1),end);
            }
        }
        else
        {
            for(int i = points.size()-1; i < 1; i++)
            {
                if(robot.checkOnPath(points.get(i-1),points.get(i)))
                {
                    robot.setPursuitPath(points.get(i-1),points.get(i));
                    break;
                }
            }
        }
    }
}
