package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Transform;

import static java.lang.Math.abs;
import static java.lang.Math.min;
@Deprecated
public class Robot_Controller_Old {
    DcMotor rfm, lfm, rbm, lbm;
    private Robot_Localizer robot;
    private double lookahead;
    public String xtremeJankTelem = "";
    public Robot_Controller_Old(DcMotor rfm, DcMotor lfm, DcMotor rbm, DcMotor lbm, double lookahead, Robot_Localizer robot)
    {
        this.rfm = rfm;
        this.lfm = lfm;
        this.rbm = rbm;
        this.lbm = lbm;
        this.robot = robot;
        this.lookahead = lookahead;
    }
    public void setVec(Transform dir,double power)
    {
        double sideMultiplierInverse                   = abs(dir.x + dir.y)+abs(dir.r);
        double sideMultiplier = min(sideMultiplierInverse, 1) / sideMultiplierInverse;

        lfm.setPower( (dir.y * sideMultiplier - dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rfm.setPower((dir.y * sideMultiplier + dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
        lbm.setPower(  (dir.y * sideMultiplier + dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rbm.setPower( (dir.y * sideMultiplier - dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
    }
    public void changePursuitDir(Transform p1, Transform p2)
    {
        p1.setOrigin(robot.pos,false);
        p2.setOrigin(robot.pos,false);
        double m  = (p2.y-p1.y)/(p2.x-p1.x);
        double b = -m*p1.x+p1.y;
        double x;
        if(p2.x>p1.x)
        {
            x = (-b*m + Math.sqrt(-b*b+lookahead*lookahead+m*m*lookahead*lookahead)) / (1 + m*m);
        }
        else
        {
            x = (-b*m - Math.sqrt(-b*b+lookahead*lookahead+m*m*lookahead*lookahead)) / (1 + m*m);
        }
        Transform pathTarget = new Transform((m*x+b),-x,0);
        pathTarget.normalize();
        pathTarget.rotate(new Transform(0,0,0),-robot.pos.r);

        double offset = Math.atan2(pathTarget.y,pathTarget.x)-((robot.pos.r%Math.PI)%-Math.PI);
        double rPower = (1-(1/(1+0.5*offset*offset)))*Math.signum(offset);
        pathTarget.r = rPower;
        setVec(pathTarget,1);
        p1 = p1.getAdded(robot.pos);
        p2 = p2.getAdded(robot.pos);
        xtremeJankTelem = pathTarget.x+" "+pathTarget.y;
    }
}
