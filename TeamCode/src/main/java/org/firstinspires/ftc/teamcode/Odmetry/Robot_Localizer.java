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
        telemetryC = String.valueOf(encSide.getCurrentPosition());
        //Getting step vars
        Transform steps = getSteps();
        double rStep = steps.r;
        Transform rPosToastal = getRPosTotal();

        //Blending rotation and forward/sideways motion
        pos.r += rStep;

        Vector2 arc = getArc(steps);
        pos.x = arc.x;
        pos.y = arc.y;


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
        double rStep = (((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2)-pPos.r);
        pPos = new Transform(rPosToastal.r,0.5*(rPosToastal.x+rPosToastal.y),((rPosToastal.x-rPosToastal.y)*Math.PI*2)/(Math.PI*encDist*2));
        return new Transform(sStep,fStep,rStep);
    }
    private Vector2 getArc(Transform steps)
    {
        if(steps.r == 0)steps.r = 0.00000001;
        double arcRad = steps.y/steps.r;
        Vector2 relativeArcPos = new Vector2(Math.cos(Math.PI-steps.r)*arcRad+(pos.x+arcRad),Math.sin(Math.PI-steps.r)*arcRad+pos.y);
        //relativeArcPos.rotate(pos.getV2(),1);
        return relativeArcPos;
    }
}