package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.Utils.Transform;


@TeleOp(name="\"row\"*3+\"ur boat\"", group="Iterative Opmode")
//@Disabled
public class DriverMode extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor center = null;
    private DcMotor right = null;
    private DcMotor left = null;
    private Robot_Localizer rowboat;
    private  Robot_Controller control;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor collector;
    DcMotor horizontal_extender;
    DcMotor vertical_extender;
    double gp1_percent_pwr;
    double gp2_percent_pwr;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        collector           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        rowboat.relocalize();
        Transform tVec = new Transform(gamepad1.right_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        // tVec.rotate(new Transform(0,0,0),-rowboat.pos.r);
        if(gamepad1.left_bumper) gp1_percent_pwr = 0.25;
        else gp1_percent_pwr = 1;

        if(gamepad2.left_bumper) gp2_percent_pwr = 0.25;
        else gp2_percent_pwr = 1;

        control.setVec(tVec, gp1_percent_pwr);

        if(gamepad1.a) collector.setPower(0.4 * gp1_percent_pwr);
        else collector.setPower(0);

        if(gamepad2.dpad_up) horizontal_extender.setPower(gp2_percent_pwr);
        else if(gamepad2.dpad_down) horizontal_extender.setPower(-gp2_percent_pwr);
        else horizontal_extender.setPower(0);

        vertical_extender.setPower(gamepad2.right_stick_y * 0.6 * gp2_percent_pwr);

        telemetry.addData("r",rowboat.pos.r);
        telemetry.addData("x",rowboat.pos.x);
        telemetry.addData("y",rowboat.pos.y);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
