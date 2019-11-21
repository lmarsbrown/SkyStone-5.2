package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Interval;
import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.MyMath;
import org.firstinspires.ftc.teamcode.Utils.Transform;

import java.util.Vector;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class Robot_Controller {
    DcMotor rfm, lfm, rbm, lbm;
    private Robot_Localizer robot;
    public String telem;
    private Transform stableOffset = new Transform(0,0,0);
    private Transform pMovement = new Transform(0,0,0);
    public Interval inter;
    private double doneCount = 0;

    public Robot_Controller(DcMotor rfm, DcMotor lfm, DcMotor rbm, DcMotor lbm,  Robot_Localizer robot)
    {
        this.rfm = rfm;
        this.lfm = lfm;
        this.rbm = rbm;
        this.lbm = lbm;
        this.robot = robot;
    }
    public void setVec(Transform dir, double power)
    {
        double sideMultiplierInverse                   = abs(dir.x + dir.y)+abs(dir.r);
        double sideMultiplier = min(sideMultiplierInverse, 1) / sideMultiplierInverse;

        lfm.setPower( (-dir.y * sideMultiplier + dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rfm.setPower((-dir.y * sideMultiplier - dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
        lbm.setPower(  (-dir.y * sideMultiplier - dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rbm.setPower( (-dir.y * sideMultiplier + dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
    }
    public void stableVec(Transform dir, double power,boolean holdR)
    {
        double len = dir.getLength();
        Transform steps = robot.steps.clone();
        steps.normalize();
        stableOffset.x = (dir.x - steps.x*1)+(stableOffset.x*0.0);
        stableOffset.y = (dir.y = steps.y*1)+(stableOffset.y*0.0);
        stableOffset.r = dir.r - (steps.r*0.1);
        stableOffset.normalize();
        telem = steps.x+" "+steps.y;

        dir.x += stableOffset.x;
        dir.y += stableOffset.y;

        if(!holdR)dir.r += stableOffset.r;
        else
        {
            double turnToOffset = (dir.r-((robot.pos.r%(Math.PI*2))%-(Math.PI*2)));
            double turnToMulti = (1-(0.5/(1+turnToOffset*turnToOffset)))*Math.signum(turnToOffset);
            if(Math.abs(turnToOffset)>0.03)setVec(new Transform(0,0,turnToMulti),1);
        }
        dir.normalize();
        dir.scale(len);


        pMovement = dir;
        setVec(dir,power);
    }
    public double gotoPointLoop(Transform point, boolean near,boolean rote)
        {
        Transform dir = new Transform(point.x-robot.pos.x,point.y-robot.pos.y,0);
        dir.normalize();
        dir.rotate(new Transform(0,0,0),-robot.pos.r);
        double goalDist = Math.hypot(robot.pos.x-point.x,robot.pos.y-point.y);
        double fPower = 1-(0.8/(0.00003*goalDist*goalDist+1));
        telem = fPower+"";
        double rOffset = ((Math.atan2(dir.y,dir.x)-((robot.pos.r%(Math.PI*2))%-(Math.PI*2))));
        double rPower = 0;
        if(!near)rPower = ((1-(1/(1+0.5*rOffset*rOffset)))*Math.signum(rOffset))*fPower;
        dir.r = rPower;

        if(goalDist<10)
        {
            double turnToOffset = (point.r-((robot.pos.r%(Math.PI*2))%-(Math.PI*2)));
            double turnToMulti = (1-(0.7/(1+turnToOffset*turnToOffset)))*Math.signum(turnToOffset);
            if(Math.abs(turnToOffset)>0.03&&rote)setVec(new Transform(0,0,turnToMulti),1);
            else setVec(new Transform(0,0,0),0);doneCount++;return doneCount;
        }
        else
        {
            setVec(dir,fPower);
        }
        doneCount = 0;
        return doneCount;
    }
    public void followPathLoop(Vector<Transform> path, double lookahead)
    {
        Transform cPoint = path.get(0);
        Transform nPoint = path.get(1);
        int index = 0;
        boolean ending = false;

        if(Math.abs((cPoint.getSetOrigin(robot.pos,false).getLength()+nPoint.getSetOrigin(robot.pos,false).getLength())-10)<10)
        {
            cPoint = nPoint;
            index++;
            if(index<path.size())
            {
                nPoint = path.get(index+1);
            }
            else
            {
                ending = true;
            }
        }
        if(path.get(index).getSetOrigin(robot.pos,false).getLength()>=lookahead||ending)
        {
            gotoPointLoop(cPoint,ending&&cPoint.getSetOrigin(robot.pos,false).getLength()<lookahead,ending);
        }
        else
        {
            double f = cPoint.getSetOrigin(robot.pos,false).getLength();
            double s = Math.sqrt((lookahead+f)*(lookahead-f));
            Transform dir = new Transform(s,f,cPoint.r);
            Transform relativeDir = robot.pos.clone();
            relativeDir.setOrigin(cPoint,false);
            dir.normalize();
            dir.rotate(robot.pos,Math.PI-Math.atan2(relativeDir.y,relativeDir.x));
            cPoint = dir;
            gotoPointLoop(dir,false,false);
        }

    }
    public void gotoPoint(Transform point,boolean near)
    {
        robot.onLocalize = (q)->{
            if(gotoPointLoop(point,near,true)>50)robot.onLocalize = null;
            return 0;
        };
    }
}
