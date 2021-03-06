package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.Transform;

public class Robot_Localizer {
    //Var init
    DcMotor encLeft;
    DcMotor encRight;
    DcMotor encSide;
    public Transform pos = new Transform(0, 0, 0);
    private Transform pPos = new Transform(0, 0, 0);
    double wheelDia = 72;
    double countsPerRev = 2400;
    double wheelCirc;
    double encDist = 385;
    double fOffsetMulti = (Math.cos(Math.PI * 0.5 - Math.atan(197.50 / 83.89)));
    public String telemetryA = "";
    public String telemetryB = "";
    public String telemetryC = "";
    public Transform speed = new Transform(0,0,0);
    private double calibrationConstant = 1;
    private double calibrationCount = 1;
    private boolean calibrating = false;
    private double pTime = 0;
    protected Lambda onLocalize;
    protected Transform steps;

    private ElapsedTime runtime;

    public Robot_Localizer(DcMotor encLeft, DcMotor encRight, DcMotor encSide, double calibrationConst) {
        //Setting vars to constructor values
        this.calibrationConstant = calibrationConst;
        this.encLeft = encLeft;
        this.encRight = encRight;
        this.encSide = encSide;
        wheelCirc = wheelDia * Math.PI;
        countsPerRev = 1 / countsPerRev;
        //Setting previous pos var to current values to reset values on restart
        Transform rPosToastal = getRPosTotal();
        pPos.x = rPosToastal.r;
        pPos.y = 0.5 * (rPosToastal.x + rPosToastal.y);
        pPos.r = ((rPosToastal.x - rPosToastal.y) * Math.PI * 2) / (Math.PI * encDist * 2);

        runtime = new ElapsedTime();
    }

    //Calculates new position based on encoder values
    public void relocalize() {

        telemetryA = String.valueOf(encLeft.getCurrentPosition());
        telemetryB = String.valueOf(encRight.getCurrentPosition());
        telemetryC = String.valueOf(encSide.getCurrentPosition());
        //Getting step vars
        steps = getSteps(getRPosTotal());



        double t = runtime.milliseconds();
        double timePassed = t-pTime;
        pTime = t;
        speed = getVSteps();
        speed = new Transform(speed.r,0.5 * (speed.x + speed.y),((speed.x - speed.y) / (encDist))*calibrationConstant);





        double rStep = steps.r;
        Transform rPosToastal = getRPosTotal();

        //Blending rotation and forward/sideways motion

        Transform arc = getArc(steps);

        pos.r += rStep*calibrationConstant;

        pos.x = arc.x;
        pos.y = arc.y;

        try {
            onLocalize.call(new Object());
        }
        catch(Throwable err)
        {

        }
    }

    public Lambda getOnLocalize() {
        return onLocalize;
    }

    //All the calibration functions
    public void startCalibration() {
        calibrationCount = 1;
        calibrating = true;
    }

    public void addCalibationStep(double reaRote) {
        calibrationConstant += reaRote/pos.r;
        calibrationCount++;
    }

    public void endCalibration() {
        calibrationConstant /= calibrationCount;
        calibrating = false;
        calibrationCount = 1;
    }
    public double getCalibrationConst() {
        if(!calibrating)return calibrationConstant;
        else throw(new RuntimeException("Cannot get calibrationd const while calibrating"));
    }

    private Transform getRPosTotal() {
        return new Transform(encRight.getCurrentPosition() * countsPerRev * wheelCirc, (encLeft.getCurrentPosition() * countsPerRev * wheelCirc), encSide.getCurrentPosition() * countsPerRev * wheelCirc);
    }
//Returns a vector containing all the steps

    /**
     * REVIEW
     */
    private Transform getSteps(Transform rPosToastal) {
        double fStep = 0.5 * (rPosToastal.x + rPosToastal.y) - pPos.y;
        double sStep = rPosToastal.r - pPos.x;
        double rStep = ((rPosToastal.x - rPosToastal.y) / (encDist)) - pPos.r;
        pPos = new Transform(rPosToastal.r, 0.5 * (rPosToastal.x + rPosToastal.y), (rPosToastal.x - rPosToastal.y) / (encDist));
        return new Transform(sStep, fStep, rStep);
    }
    private Transform getVSteps() {
        return new Transform(((DcMotorEx)encRight).getVelocity() * countsPerRev * wheelCirc, (((DcMotorEx)encLeft).getVelocity() * countsPerRev * wheelCirc), ((DcMotorEx)encSide).getVelocity() * countsPerRev * wheelCirc);
    }

    private Transform getArc(Transform steps) {
        if (steps.r == 0) steps.r = 0.00000001;
        double d = Math.hypot(steps.x, steps.y);
        double arcRad = d / steps.r;
        Transform relativeArcPos = new Transform(Math.cos(Math.PI - steps.r) * arcRad + (pos.x + arcRad), (Math.sin(Math.PI - steps.r) * arcRad) + pos.y, pos.r);
        relativeArcPos.rotate(pos, pos.r + Math.atan2(steps.x, steps.y));
        //relativeArcPos.rotate(pos.getV2(),1);
        return relativeArcPos;
    }
}