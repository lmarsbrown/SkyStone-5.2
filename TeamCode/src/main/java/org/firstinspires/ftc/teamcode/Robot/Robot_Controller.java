package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Interval;
import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.Transform;

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
    public boolean stat = false;
    private PIDController rPid = new PIDController(0.65,0.13,-1.8,0);
    private PIDController mPid = new PIDController(0.023,0,0.3,0);
    private PIDController vXPid = new PIDController(1,0,0,0.7);
    private PIDController vYPid = new PIDController(1,0,0,  0.7);
    private PIDController vRPid = new PIDController(-1,0,0,  0);
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    private double ytigniny = 0;
    private boolean flip =  false;

    public Robot_Controller(DcMotor rfm, DcMotor lfm, DcMotor rbm, DcMotor lbm,  Robot_Localizer robot)
    {
        this.rfm = rfm;
        this.lfm = lfm;
        this.rbm = rbm;
        this.lbm = lbm;
        this.robot = robot;

    }
    public Robot_Controller(DcMotor rfm, DcMotor lfm, DcMotor rbm, DcMotor lbm,  Robot_Localizer robot,boolean flipX)
    {
        this.rfm = rfm;
        this.lfm = lfm;
        this.rbm = rbm;
        this.lbm = lbm;
        this.robot = robot;
        this.flip = flipX;

    }
    public void setVec(Transform tDir, double power)
    {
        Transform dir = tDir.clone();
        dir.normalize();
        double sideMultiplierInverse                   = abs(dir.x + dir.y)+abs(dir.r);
        double sideMultiplier = min(sideMultiplierInverse, 1) / sideMultiplierInverse;

        lfm.setPower( (-dir.y * sideMultiplier + dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rfm.setPower((-dir.y * sideMultiplier - dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
        lbm.setPower(  (-dir.y * sideMultiplier - dir.x * sideMultiplier + dir.r * sideMultiplier)*power );
        rbm.setPower( (-dir.y * sideMultiplier + dir.x * sideMultiplier - dir.r * sideMultiplier)*power );
    }


    public void setVec(Transform rDir, double power, boolean PID)
    {
        if(PID)
        {
            Transform tDir = rDir.clone();
            tDir.normalize();
            double sideMultiplierInverse                  = abs(tDir.x + tDir.y)+abs(tDir.r);
            double sideMultiplier = min(sideMultiplierInverse, 1) / sideMultiplierInverse;
            Transform speed = robot.speed.clone();
            vXPid.setGoal(tDir.x*power);
            vXPid.update(speed.x*0.002);
            vYPid.setGoal(tDir.y*power);
            vYPid.update(speed.y*0.002);
            vRPid.setGoal(tDir.r*power);
            vRPid.update(speed.r/(2*Math.PI));
            Transform dir = new Transform(vXPid.power,vYPid.power,vRPid.power);

            dir.normalize();





            lfm.setPower( (-dir.y * sideMultiplier + dir.x * sideMultiplier + dir.r * sideMultiplier)  );
            rfm.setPower((-dir.y * sideMultiplier - dir.x * sideMultiplier - dir.r * sideMultiplier)  );
            lbm.setPower(  (-dir.y * sideMultiplier - dir.x * sideMultiplier + dir.r * sideMultiplier)  );
            rbm.setPower( (-dir.y * sideMultiplier + dir.x * sideMultiplier - dir.r * sideMultiplier)  );
            TelemetryPacket p = new TelemetryPacket();
            //p.put("Target X",tDir.x);
            p.put("Target Y",tDir.y);
            //p.put("Real X",robot.steps.x);
            p.put("Real Y",robot.speed.y*0.002);
            //p.put("X Power",vXPid.power);
            p.put("Y Power",vYPid.power);
            dashboard.sendTelemetryPacket(p);
        }
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

        dir.x += stableOffset.x;
        dir.y += stableOffset.y;

        if(!holdR)dir.r += stableOffset.r;
        else
        {
            double turnToOffset = (dir.r-((robot.pos.r%(Math.PI*2))%-(Math.PI*2)));
            telem = turnToOffset+"";
            double turnToMulti = (1-(0.5/(1+turnToOffset*turnToOffset)))*Math.signum(turnToOffset);
            if(abs(turnToOffset)>0.03)setVec(new Transform(0,0,turnToMulti),1);
        }
        dir.normalize();
        dir.scale(len);


        pMovement = dir;
        setVec(dir,power);
    }
    //  TODO: Add turn + forward
    // TODO: Make speed and slop into params
    public double gotoPointLoop(Transform point,boolean end, double minSpeed, double maxSpeed, double slop, double startR)
    {
        Transform dir = new Transform(point.x-robot.pos.x,point.y-robot.pos.y,0);
        dir.normalize();
        dir.rotate(new Transform(0,0,0),-robot.pos.r);
        double goalDist = Math.hypot(robot.pos.x-point.x,robot.pos.y-point.y);
        double fPower = -mPid.update(goalDist);
        double rOffset = Math.PI+((Math.atan2(dir.y,dir.x)-((robot.pos.r%(Math.PI))%-(Math.PI))));
        dir.r = (startR-robot.pos.r)*Math.min(fPower,1);

        if(goalDist<slop||stat)
        {
            stat = true;
            double turnToOffset = (((point.r%(Math.PI*2))%-(Math.PI*2))-((robot.pos.r%(Math.PI*2))%-(Math.PI*2)));
            telem = turnToOffset+"";
            //double turnToMulti = (1-(0.7/(1+turnToOffset*turnToOffset)))*Math.signum(turnToOffset);
            double turnToMulti = -rPid.update(turnToOffset);
            if(abs(turnToOffset)>0.04&&end)setVec(new Transform(0,0,turnToMulti),1);
            else {setVec(new Transform(0,0,0),0);doneCount++;stat = false;return doneCount;}
        }
        else
        {
            setVec(dir,Math.max(Math.min(fPower,maxSpeed),minSpeed));
        }
        doneCount = 0;
        return doneCount;
    }
    /*
    public void followPathLoop(Vector<Transform> path, double lookahead)
    {
        Transform cPoint = path.get(0);
        Transform nPoint = path.get(1);+
        int index = 0;
        boolean ending = false;
        if(cPoint.getSetOrigin(robot.pos,false).getLength()<10)
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
        gotoPointLoop(cPoint,ending&&cPoint.getSetOrigin(robot.pos,false).getLength()<lookahead,ending);
        if(path.get(index).getSetOrigin(robot.pos,false).getLength()<lookahead&&!ending)
        {
            Transform p = robot.pos.getSetOrigin(cPoint,false);
            double d = Math.hypot(p.x,p.y);
            double step = Math.sqrt((lookahead+d)*(lookahead-d));
            cPoint.stepAtPoint(nPoint.getSetOrigin(cPoint,false),step);
        }
    }*/

    public void gotoPoint(Transform point, boolean end,double minSpeed, double maxSpeed, double slop, Lambda callback)
    {
        mPid.reset(0);
        rPid.reset(0);
        if(flip)
        {
            point.x *= -1;
            point.r *= -1;
        }
        double startR = robot.pos.r;
        Interval callbackThread = new Interval((Object obj)->{
            callback.call(new Object());
            return 1;
        },1);
        robot.onLocalize = (q)->{
            double count = gotoPointLoop(point,end,minSpeed,maxSpeed,slop,startR);
            if(count>10||(count>0&&!end)){robot.onLocalize = null;callbackThread.start();}
            return 0;
        };
    }
    public void clearGoto()
    {
        robot.onLocalize = null;
    }
}