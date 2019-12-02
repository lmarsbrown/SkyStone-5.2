package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.Utils.Transform;


@TeleOp(name="Robot-Centric Driving", group="Iterative Opmode")
//@Disabled
public class RobotCentric extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Robot_Localizer rowboat;
    private Robot_Controller control;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor horizontal_extender;
    private DcMotor vertical_extender;

    private Servo collector_arm;
    private Servo foundation_mover;

    private CRServo outer_collector;
    private CRServo inner_collector;

    private double gp1_percent_pwr;
    private double gp2_percent_pwr;

    private Transform saved_robot_pos;
    private Transform robot_vector;

    private boolean going_to_pt;

    private DigitalChannel limit_switch_front;
    private DigitalChannel limit_switch_back;

    @Override
    public void init() {
        leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");

        collector_arm       = hardwareMap.get(Servo.class, "collector_arm");
        foundation_mover    = hardwareMap.get(Servo.class, "Foundation_mover");

        outer_collector     = hardwareMap.get(CRServo.class, "outer_collector");
        inner_collector     = hardwareMap.get(CRServo.class, "inner_collector");

        limit_switch_back   = hardwareMap.get(DigitalChannel.class, "limit_switch1");
        limit_switch_front  = hardwareMap.get(DigitalChannel.class, "limit_switch2");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vertical_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);

        going_to_pt = false;

        collector_arm.setPosition(0.403);
        foundation_mover.setPosition(0);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        rowboat.relocalize();
        robot_vector = new Transform(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.left_bumper) gp1_percent_pwr = 0.25;
        else if (gamepad1.right_bumper) gp1_percent_pwr = 0.4;
        else gp1_percent_pwr = 1;

        if (gamepad2.left_trigger > 0.8) gp2_percent_pwr = 0.25;
        else if (gamepad2.right_trigger > 0.8) gp2_percent_pwr = 0.4;
        else gp2_percent_pwr = 1;

        if (gamepad1.y) {
            saved_robot_pos = rowboat.pos.clone();
            saved_robot_pos.r = (saved_robot_pos.r % (2 * Math.PI)) % -(2 * Math.PI);
        }

        if (gamepad1.x && saved_robot_pos != null && !going_to_pt) {
            going_to_pt = true;
            control.gotoPoint(saved_robot_pos, true, true, 0.2, (Object obj) -> {
                going_to_pt = false;
                return 0;
            });
        }

        if (gamepad1.a) foundation_mover.setPosition(0.57);
        if (gamepad1.b) foundation_mover.setPosition(0);

        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            going_to_pt = false;
            control.clearGoto();
        }

        if (!going_to_pt) control.setVec(robot_vector, gp1_percent_pwr);

        if (gamepad2.dpad_down && limit_switch_front.getState())
            horizontal_extender.setPower(-gp2_percent_pwr * 0.5);
        else if (gamepad2.dpad_up && limit_switch_back.getState())
            horizontal_extender.setPower(gp2_percent_pwr * 0.5);
        else horizontal_extender.setPower(0);

        if(gamepad2.left_stick_y > 0)      vertical_extender.setPower(-gamepad2.left_stick_y * gp2_percent_pwr * 0.5);
        else if(gamepad2.left_stick_y < 0) vertical_extender.setPower(-gamepad2.left_stick_y * gp2_percent_pwr);
        else                               vertical_extender.setPower(0);

        if(gamepad2.right_stick_y > 0.1) {
            collector_arm.setPosition(0.403);
            inner_collector.setPower(-gamepad2.right_stick_y);
            outer_collector.setPower(-gamepad2.right_stick_y);
        } else if(gamepad2.right_stick_y < -0.1) {
            collector_arm.setPosition(0.72);
            inner_collector.setPower(-gamepad2.right_stick_y);
            outer_collector.setPower(-gamepad2.right_stick_y);
        } else {
            inner_collector.setPower(0);
            outer_collector.setPower(0);
        }

        telemetry.addData("X Position", rowboat.pos.x);
        telemetry.addData("Y Position", rowboat.pos.y);
        telemetry.addData("Rotation", rowboat.pos.r);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
