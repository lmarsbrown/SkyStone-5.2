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
    protected boolean setPathFollowVec(Robot_Controller robot)
    {
        robot.jank = String.valueOf(robot.robot.pos.x-end.x);
        if(Math.hypot(robot.robot.pos.x-end.x,robot.robot.pos.y-end.y)<50)
        {
            robot.setVec(new Transform(0,0,0),0);
            return true;
        }
        else
        {
        if(robot.checkOnPath(points.get(points.size()-1),end))
        {
            robot.ending = true;
            robot.setPursuitPath(robot.robot.pos,end);
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
            return false;
        }
    }
}
