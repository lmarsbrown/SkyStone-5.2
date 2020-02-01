package org.firstinspires.ftc.teamcode.OldOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Position Finder", group="Iterative Opmode")
@Disabled
@Deprecated
public class ServoPositionFinder extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo stone_collector;
    private Servo stone_collector_arm;

    private Boolean x_down;
    private Boolean x_down2;
    private Boolean a_down;
    private Boolean a_down2;
    private Boolean b_down;
    private Boolean b_down2;
    private Boolean y_down;
    private Boolean y_down2;

    private double stone_collector_pos;
    private double stone_collector_arm_pos;

    @Override
    public void init() {
        stone_collector = hardwareMap.get(Servo.class, "front_foundation_right");
        stone_collector_arm = hardwareMap.get(Servo.class, "front_foundation_left");

        x_down = Boolean.FALSE;
        a_down = Boolean.FALSE;
        b_down = Boolean.FALSE;
        y_down = Boolean.FALSE;
        x_down2 = Boolean.FALSE;
        a_down2 = Boolean.FALSE;
        b_down2 = Boolean.FALSE;
        y_down2 = Boolean.FALSE;

        stone_collector_pos = 0.5;
        stone_collector_arm_pos = 0.5;
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
        if(gamepad2.a && !gamepad2.left_bumper && !a_down) {
            stone_collector_pos += 0.1;
            stone_collector.setPosition(stone_collector_pos);
            a_down = Boolean.TRUE;
        } else if (!gamepad2.a && a_down) {
            a_down = Boolean.FALSE;
        }
        if(gamepad2.x && !gamepad2.left_bumper && !x_down) {
            stone_collector_pos -= 0.1;
            stone_collector.setPosition(stone_collector_pos);
            x_down = Boolean.TRUE;
        } else if (!gamepad2.x && x_down) {
            x_down = Boolean.FALSE;
        }
        if(gamepad2.a && gamepad2.left_bumper && !a_down2) {
            stone_collector_arm_pos += 0.1;
            stone_collector_arm.setPosition(stone_collector_arm_pos);
            a_down2 = Boolean.TRUE;
        } else if (!gamepad2.a && a_down2) {
            a_down2 = Boolean.FALSE;
        }
        if(gamepad2.x && gamepad2.left_bumper && !x_down2) {
            stone_collector_arm_pos -= 0.1;
            stone_collector_arm.setPosition(stone_collector_arm_pos);
            x_down2 = Boolean.TRUE;
        } else if (!gamepad2.x && x_down2) {
            x_down2 = Boolean.FALSE;
        }

        if(gamepad2.b && !gamepad2.left_bumper && !b_down) {
            stone_collector_pos += 0.01;
            stone_collector.setPosition(stone_collector_pos);
            b_down = Boolean.TRUE;
        } else if (!gamepad2.b && b_down) {
            b_down = Boolean.FALSE;
        }
        if(gamepad2.y && !gamepad2.left_bumper && !y_down) {
            stone_collector_pos -= 0.01;
            stone_collector.setPosition(stone_collector_pos);
            y_down = Boolean.TRUE;
        } else if (!gamepad2.y && y_down) {
            y_down = Boolean.FALSE;
        }
        if(gamepad2.b && gamepad2.left_bumper && !b_down2) {
            stone_collector_arm_pos += 0.01;
            stone_collector_arm.setPosition(stone_collector_arm_pos);
            b_down2 = Boolean.TRUE;
        } else if (!gamepad2.b && b_down2) {
            b_down2 = Boolean.FALSE;
        }
        if(gamepad2.y && gamepad2.left_bumper && !y_down2) {
            stone_collector_arm_pos -= 0.01;
            stone_collector_arm.setPosition(stone_collector_arm_pos);
            y_down2 = Boolean.TRUE;
        } else if (!gamepad2.y && y_down2) {
            y_down2 = Boolean.FALSE;
        }
        telemetry.addData("Front-Right Foundation Mover", stone_collector_pos);
        telemetry.addData("Front-Left Foundation Mover", stone_collector_arm_pos);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
