package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.Utils.Transform;


@TeleOp(name="Nico's Field-Centric Driving", group="Iterative Opmode")
//@Disabled
public class FieldCentric extends OpMode {
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
    private Servo left_foundation_mover;
    private Servo right_foundation_mover;
    private Servo right_stone_collector_arm;
    private Servo left_stone_collector_arm;
    private Servo right_stone_collector;
    private Servo left_stone_collector;

    private Servo capstone_arm;

    private CRServo outer_collector;
    private CRServo inner_collector;

    private double gp1_percent_pwr;
    private double gp2_percent_pwr;

    private Transform saved_robot_pos;
    private Transform robot_vector;

    private boolean going_to_pt;

    private DigitalChannel limit_switch_front;
    private DigitalChannel limit_switch_back;

    private double positional_offset;

    private Boolean x_down_gp2;
    private Boolean x_down_gp1;
    private Boolean y_down;

    private String capstone_arm_loc;
    private String foundation_mover_loc;
    private String front_foundation_movers_loc;

    @Override
    public void init() {
        leftFront                 = hardwareMap.get(DcMotor.class, "left_front");
        rightFront                = hardwareMap.get(DcMotor.class, "right_front");
        leftBack                  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack                 = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender       = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender         = hardwareMap.get(DcMotor.class, "vertical_ext");

        collector_arm             = hardwareMap.get(Servo.class, "collector_arm");
        foundation_mover          = hardwareMap.get(Servo.class, "Foundation_mover");
        left_foundation_mover     = hardwareMap.get(Servo.class, "front_foundation_left");
        right_foundation_mover     = hardwareMap.get(Servo.class, "front_foundation_right");
        right_stone_collector_arm = hardwareMap.get(Servo.class, "right_stone_collector_arm");
        left_stone_collector_arm  = hardwareMap.get(Servo.class, "left_stone_collector_arm");
        right_stone_collector     = hardwareMap.get(Servo.class, "right_stone_collector");
        left_stone_collector      = hardwareMap.get(Servo.class, "left_stone_collector");
        capstone_arm              = hardwareMap.get(Servo.class, "Capstone_Arm");

        outer_collector           = hardwareMap.get(CRServo.class, "outer_collector");
        inner_collector           = hardwareMap.get(CRServo.class, "inner_collector");

        limit_switch_back         = hardwareMap.get(DigitalChannel.class, "limit_switch1");
        limit_switch_front        = hardwareMap.get(DigitalChannel.class, "limit_switch2");

        positional_offset         = 0;

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

        rowboat     = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control     = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat);

        going_to_pt = false;

        collector_arm.setPosition(0.72);
        foundation_mover.setPosition(0.05);
        //capstone_arm.setPosition(0);

        x_down_gp1 = Boolean.FALSE;
        x_down_gp2 = Boolean.FALSE;
        y_down = Boolean.FALSE;
        capstone_arm_loc = "up";
        foundation_mover_loc = "up";
        front_foundation_movers_loc = "up";
        left_foundation_mover.setPosition(0.72);
        right_foundation_mover.setPosition(0.26);
        right_stone_collector_arm.setPosition(0.1);
        left_stone_collector_arm.setPosition(0.9);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        right_stone_collector.setPosition(0.98);
        left_stone_collector.setPosition(0.01);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        right_stone_collector_arm.setPosition(0);
        left_stone_collector_arm.setPosition(1);
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
        TelemetryPacket p = new TelemetryPacket();
        p.put("Left",leftBack.getCurrentPosition());
        p.put("Right",rightBack.getCurrentPosition());
        p.put("Side",rightFront.getCurrentPosition());

        control.dashboard.sendTelemetryPacket(p);

        robot_vector = new Transform(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.left_bumper) gp1_percent_pwr = 0.25;
        else if (gamepad1.right_bumper) gp1_percent_pwr = 0.35;
        else gp1_percent_pwr = 1;

        if (gamepad2.left_trigger > 0.8) gp2_percent_pwr = 0.25;
        else if (gamepad2.right_trigger > 0.8) gp2_percent_pwr = 0.4;
        else gp2_percent_pwr = 1;

        /*if (gamepad1.y) {
            saved_robot_pos = rowboat.pos.clone();
            saved_robot_pos.r = (saved_robot_pos.r % (2 * Math.PI)) % -(2 * Math.PI);
        }

        if (gamepad1.x && saved_robot_pos != null && !going_to_pt) {
            going_to_pt = true;
            control.gotoPoint(saved_robot_pos, false, 0.1,0.7,20, (Object obj) -> {
                going_to_pt = false;
                return 0;
            });
        }*/

        if(gamepad1.x && foundation_mover_loc == "up" && !x_down_gp1) {
            foundation_mover.setPosition(0.57);
            foundation_mover_loc = "down";
            x_down_gp1 = Boolean.TRUE;
        } else if(gamepad1.x && foundation_mover_loc == "down" && !x_down_gp1) {
            foundation_mover.setPosition(0.05);
            foundation_mover_loc = "up";
            x_down_gp1 = Boolean.TRUE;
        } else if(!gamepad1.x && x_down_gp1) {
            x_down_gp1 = Boolean.FALSE;
        }

        if(gamepad1.y && front_foundation_movers_loc == "up" && !y_down) {
            left_foundation_mover.setPosition(0.14);
            right_foundation_mover.setPosition(0.86);
            front_foundation_movers_loc = "down";
            y_down = Boolean.TRUE;
        } else if(gamepad1.y && front_foundation_movers_loc == "down" && !y_down) {
            left_foundation_mover.setPosition(0.73);
            right_foundation_mover.setPosition(0.25);
            front_foundation_movers_loc = "up";
            y_down = Boolean.TRUE;
        } else if(!gamepad1.y && y_down) {
            y_down = Boolean.FALSE;
        }

        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
            going_to_pt = false;
            control.clearGoto();
        }

        robot_vector.rotate(new Transform(0, 0, 0), positional_offset - rowboat.pos.r);

        if (!going_to_pt) control.setVec(robot_vector, gp1_percent_pwr);

        if      (gamepad2.dpad_down && limit_switch_back.getState())  horizontal_extender.setPower(-gp2_percent_pwr * 0.5);
        else if (gamepad2.dpad_up   && limit_switch_front.getState()) horizontal_extender.setPower(gp2_percent_pwr * 0.5);
        else                                                          horizontal_extender.setPower(0);

        if(gamepad2.left_stick_y > 0)      vertical_extender.setPower(-gamepad2.left_stick_y * gp2_percent_pwr * 0.75);
        else if(gamepad2.left_stick_y < 0) vertical_extender.setPower(-gamepad2.left_stick_y * gp2_percent_pwr);
        else                               vertical_extender.setPower(0);

        if(gamepad2.y) {
            collector_arm.setPosition(0.72);
            inner_collector.setPower(1);
            outer_collector.setPower(1);
        } else if(gamepad2.a) {
            collector_arm.setPosition(0.403);
            inner_collector.setPower(-1);
            outer_collector.setPower(-1);
        } else {
            inner_collector.setPower(0);
            outer_collector.setPower(0);
        }

        if(gamepad2.x && capstone_arm_loc == "up" && !x_down_gp2) {
            capstone_arm.setPosition(0.54);
            capstone_arm_loc = "down";
            x_down_gp2 = Boolean.TRUE;
        } else if(gamepad2.x && capstone_arm_loc == "down" && !x_down_gp2) {
            capstone_arm.setPosition(0.82);
            capstone_arm_loc = "up";
            x_down_gp2 = Boolean.TRUE;
        } else if(!gamepad2.x && x_down_gp2) {
            x_down_gp2 = Boolean.FALSE;
        }

        if(gamepad1.left_trigger > 0.9) {
            left_foundation_mover.setPosition(0.27);
            right_foundation_mover.setPosition(0.73);
        }

        if(gamepad1.right_trigger > 0.9) {
            positional_offset = rowboat.pos.r;
        }

        telemetry.addData("X Position", rowboat.pos.x);
        telemetry.addData("Y Position", rowboat.pos.y);
        telemetry.addData("Rotation", rowboat.pos.r);

        if(saved_robot_pos != null)
        {
            telemetry.addLine();
            telemetry.addData("Saved X Position", saved_robot_pos.x);
            telemetry.addData("Saved Y Position", saved_robot_pos.y);
            telemetry.addData("Saved Rotation", saved_robot_pos.r);

        }
        telemetry.addData("Z Lift Encoder", vertical_extender.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
