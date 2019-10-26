package org.firstinspires.ftc.teamcode.Odmetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.*;

public class Robot_Localizer{
    //Var init
    DcMotor encLeft;
    DcMotor encRight;
    DcMotor encSide;
    public Transform pos = new Transform(0,0,0);
    private Transform pPos = new Transform(0,0,0);
    double wheelDia = 72;
    double countsPerRev = 2400;
    double wheelCirc;
    double encDist = 385;
    double fOffsetMulti = (Math.cos(Math.PI*0.5-Math.atan(197.50/83.89)));
    public String telemetryA = "";
    public String telemetryB = "";
    public String telemetryC = "";
    private double calibrationConstant = 1;
    private double calibationCount = 1;
    public Robot_Localizer(DcMotor encLeft,DcMotor encRight,DcMotor encSide)
    {
        //Setting vars to constructor values
        this.encLeft = encLeft;
        this.encRight = encRight;
        this.encSide = encSide;
        wheelCirc = wheelDia * Math.PI;
        countsPerRev = 1/countsPerRev;
        //Setting previous pos var to current values to reset values on restart
        Transform rPosToastal = getRPosTotal();
        pPos.x = rPosToastal.r;
        pPos.y = 0.5*(rPosToastal.x+rPosToastal.y);
        pPos.r = ((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2);
    }
    //Calculates new position based on encoder values
    public void relocalize()
    {
        telemetryA = String.valueOf(encLeft.getCurrentPosition());
        telemetryB = String.valueOf(encRight.getCurrentPosition());
        //telemetryC = String.valueOf(encSide.getCurrentPosition());
        //Getting step vars
        Transform steps = getSteps();
        double rStep = steps.r;
        Transform rPosToastal = getRPosTotal();

        //Blending rotation and forward/sideways motion
        pos.r += rStep;

        Transform arc = getArc(steps);
        pos.x = arc.x;
        pos.y = arc.y;
    }
    //All the calibration functions
    public void startCalibration()
    {
        calibationCount = 1;
    }
    public void addCalibationStep(double rote)
    {
        calibrationConstant += 1;
        calibationCount++;
    }
    public void endCalibration()
    {
        calibrationConstant /= calibationCount;
    }

    private Transform getRPosTotal()
    {
        return new Transform(encRight.getCurrentPosition()*countsPerRev*wheelCirc,(encLeft.getCurrentPosition()*countsPerRev*wheelCirc),encSide.getCurrentPosition()*countsPerRev*wheelCirc);
    }
    //Returns a vector containing all the steps
    /**REVIEW*/
    private Transform getSteps()
    {
        //Calculating step vars
        Transform rPosToastal = getRPosTotal();
        double fStep = 0.5*(rPosToastal.x+rPosToastal.y)-pPos.y;
        double sStep = rPosToastal.r-pPos.x;
        double rStep = ((rPosToastal.x-rPosToastal.y)/(encDist))-pPos.r;
        pPos = new Transform(rPosToastal.r,0.5*(rPosToastal.x+rPosToastal.y),(rPosToastal.x-rPosToastal.y)/(encDist));
        return new Transform(sStep,fStep,rStep);
    }
    private Transform getArc(Transform steps)
    {
        if(steps.r == 0)steps.r = 0.00000001;
        double arcRad = steps.y/steps.r;
        Transform relativeArcPos = new Transform(Math.cos(Math.PI-steps.r)*arcRad+(pos.x+arcRad),Math.sin(Math.PI-steps.r)*arcRad+pos.y,pos.r);
        relativeArcPos.rotate(pos,pos.r);
        //relativeArcPos.rotate(pos.getV2(),1);
        return relativeArcPos;
    }
}