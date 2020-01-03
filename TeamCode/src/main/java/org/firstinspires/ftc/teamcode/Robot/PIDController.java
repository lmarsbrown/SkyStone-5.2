package org.firstinspires.ftc.teamcode.Robot;

class PIDController
{
    private double pGain;
    private double iGain;
    private double dGain;
    private double iVal = 0;
    private double pErr = 0;
    private double goal;
    public double power = 0;
    public PIDController(double p, double i, double d,double goal)
    {
        pGain = p;
        iGain = i;
        dGain = d;
        this.goal = goal;
    }
    public void reset(double goal)
    {
        iVal = 0;
        pErr = 0;
        this.goal = goal;
    }
    public void setGoal(double goal)
    {
        this.goal = goal;
    }
    public double update(double input)
    {
        double error = goal - input;

        iVal+=error;

        iVal = Math.max(Math.min(iVal,1),-1);

        double dVal = error-pErr;

        power = pGain*error+iGain*iVal+dGain*dVal;

        pErr = error;

        return power;
    }
}