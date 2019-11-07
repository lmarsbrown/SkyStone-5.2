package org.firstinspires.ftc.teamcode.Robot;

import android.telephony.MbmsStreamingSession;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Transform;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class Robot_Controller {
    DcMotor rfm, lfm, rbm, lbm;
    private Transform p1;
    private Transform p2;
    protected Robot_Localizer robot;
    protected double lookahead;
    protected boolean ending = false;
    public Robot_Controller(DcMotor rfm,DcMotor lfm,DcMotor rbm,DcMotor lbm,double lookahead, Robot_Localizer robot)
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
        double sideMultiplierInverse                   = abs(-dir.x - dir.y)+abs(dir.r);
        double sideMultiplier = min(sideMultiplierInverse, 1) / sideMultiplierInverse;

        lfm.setPower( (dir.y * sideMultiplier - dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rfm.setPower((dir.y * sideMultiplier + dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
        lbm.setPower(  (dir.y * sideMultiplier + dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rbm.setPower( (dir.y * sideMultiplier - dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
    }
    public void setPursuitPath(Transform p1,Transform p2)
    {
        /*dir.normalize();
        if(dir.x == 0)dir.x = 0.00000001;
        p1 = robot.pos;
        p2 = robot.pos.getAdded(dir);*/
        this.p1 = p1;
        this.p2 = p2;
        changePursuitDir();
    }
    public void changePursuitDir()
    {
        if(ending)
        {
            p1.setOrigin(robot.pos,false);
            p2.setOrigin(robot.pos,false);
            double m  = (p2.y-p1.y)/(p2.x-p1.x);
            double b = -m*p1.x+p1.y;
            double x;
            if(m>0)
            {
                x = (-b*m + Math.sqrt(-b*b+lookahead*lookahead+m*m*lookahead*lookahead)) / (1 + m*m);
            }
            else
            {
                x = (-b*m - Math.sqrt(-b*b+lookahead*lookahead+m*m*lookahead*lookahead)) / (1 + m*m);
            }
            Transform pathTarget = new Transform(x,-(m*x+b),0);
            pathTarget.normalize();
            pathTarget.rotate(new Transform(0,0,0),-robot.pos.r);

            double offset = Math.atan2(pathTarget.y,pathTarget.x)-((robot.pos.r%Math.PI)%-Math.PI);
            double rPower = (1-(1/(1+0.5*offset*offset)))*Math.signum(offset);
            pathTarget.r = rPower;
            setVec(pathTarget,0.5);
            p1 = p1.getAdded(robot.pos);
            p2 = p2.getAdded(robot.pos);
        }
        else
        {
            Transform travelVec = new Transform(p2.x-p1.x,p2.y-p1.y,0);
            travelVec.normalize();
            double offset = Math.atan2(travelVec.y,travelVec.x)-((robot.pos.r%Math.PI)%-Math.PI);
            double rPower = (1-(1/(1+0.5*offset*offset)))*Math.signum(offset);
            travelVec.r = rPower;
            setVec(travelVec,0.5);
        }
    }
    protected boolean checkOnPath(Transform p1, Transform p2)
    {

        double m  = (p2.y-p1.y)/(p2.x-p1.x);
        double b = -m*p1.x+p1.y;
        return b*b<lookahead*lookahead+m*m+b*b;
    }
    public void setPathFollow(Path path)
    {

    }
}


