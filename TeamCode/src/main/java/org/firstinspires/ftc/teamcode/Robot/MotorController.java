package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorController {
    private DcMotor  motor;
    private PIDController controller;
    private boolean stopOnComplete;
    private int goal=0;
    public MotorController(DcMotor motor, PIDController controller)
    {
        this.motor = motor;
        this.controller = controller;
    }
    public void setTarget(int target,boolean stopOnComplete)
    {
        this.controller.setGoal(target);
        this.stopOnComplete = stopOnComplete;
        goal = target;
    }
    public void updateController()
    {
        int pos = this.motor.getCurrentPosition();
        if(!stopOnComplete||(pos<goal-50||pos>goal+50))
        {
            this.controller.update(pos);
            this.motor.setPower(-this.controller.power);
        }
        else
        {
            this.motor.setPower(0);
        }
    }
}
