package org.firstinspires.ftc.teamcode.OldOpModes;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Utils.Interval;
import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.Transform;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;


@TeleOp(name="tesorflow", group="Iterative Opmode")
@Disabled
@Deprecated
public class Tensorflow extends OpMode {
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

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private Interval inter;

    private static final String VUFORIA_KEY =
            "AXD7Z8n/////AAABmeUpWxkr4UwCn1T5SeLkoYsYLhZbVtkUUiH3anbbVLB6LppfJSGm+AVOaffZudIRjtBpgZG1MjRa4sz1YZPRUf/Tv9x0HQrm2+GfkHn2fi/Zu1GRH873rFjxnFnUIOar2q48nPytFs6n4/P4tkUMwBSmlffeJxcxhBSMnFgH5AXrTL7F+WAerdDGlFVGlHJgnbkMJWyFwsSrhkSm2TD2vnsiZ2PdnKhUPL3FLxHPTUh+b39PTlmW4Yzws1jDA+Xfp4lvn+E7p4g+fY/eAA3gzcRQP4XyhBYjACJaXOtatxclSNxBU5xyGN+L1cM5hQ/6d5UJBYQeQdV5GFzv0hd5xEYMCKcZplda+0y1f6+QG2Z6";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void init() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

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

        if (tfod != null) {
            tfod.activate();
        }
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
        detect((Object pos)->{
            telemetry.addData("out",pos);
            telemetry.update();

            return 0;
        });
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        rowboat.relocalize();


        /*telemetry.addData("x",rowboat.pos.x);
        telemetry.addData("y",rowboat.pos.y);
        telemetry.addData("r",rowboat.pos.r);
        telemetry.update();*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    public void detect(Lambda callback)
    {
        int out;
        inter = new Interval((Object obj)->{
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions != null)
            {
                if(updatedRecognitions.size()>=2)
                {
                    Comparator<Recognition> comparator = (e1, e2) -> Math.round(e2.getLeft()-e1.getLeft());
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                        updatedRecognitions.sort(comparator);
                    }
                    else
                    {
                        try {
                            throw(new Throwable("REEEEEEEEEEEE"));
                        } catch (Throwable throwable) {
                            throwable.printStackTrace();
                        }
                    }
                    String l1 = updatedRecognitions.get(0).getLabel();

                    String l2 = updatedRecognitions.get(1).getLabel();

                    if(l1=="Stone")
                    {
                        if(l2 == "Stone"){callback.call(2);return  1;}
                        else {callback.call(1);return  1;}
                    }
                    else {callback.call(0);return  1;}
                }

            }


            return 0;
        },250);
        inter.start();
    }

}
