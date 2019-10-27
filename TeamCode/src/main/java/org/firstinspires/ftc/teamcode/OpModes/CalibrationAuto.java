package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Odmetry.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Utils.Transform;


@TeleOp(name="CalibrationAuto", group="Iterative Opmode")
//@Disabled
public class CalibrationAuto extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor center = null;
    private DcMotor right = null;
    private DcMotor left = null;
    private DcMotor other = null;
    private Robot_Localizer rowboat;    // The IMU sensor object
    private boolean calibratedThisLoop = false;
    private double gyroPreval = 0;
    private double gyroVal = 0;
    BNO055IMU imu;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        center = hardwareMap.get(DcMotor.class,"right_back");
        right = hardwareMap.get(DcMotor.class,"right_front");
        left = hardwareMap.get(DcMotor.class,"left_back");
        other = hardwareMap.get(DcMotor.class,"left_front");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
        center.setDirection(DcMotorSimple.Direction.REVERSE);
        rowboat = new Robot_Localizer(left,right,center,1);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
        rowboat.startCalibration();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        getGyroRote();
        rowboat.relocalize();
        if(Math.abs(gyroVal)%(Math.PI*0.5) < 0.1&&!calibratedThisLoop)
        {
            calibratedThisLoop = true;
            rowboat.addCalibationStep(Math.abs(gyroVal));
        }
        else if(Math.abs(gyroVal)%(Math.PI*0.5) > 0.1) calibratedThisLoop = false;
        if(gamepad1.a)
        {
            center.setPower(0);
            right.setPower(0);
            left.setPower(0);
            other.setPower(0);
            rowboat.endCalibration();
            try {
                telemetry.addData("Calibration Constant",rowboat.getCalibrationConst());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        else
        {
            center.setPower(-0.3);
            right.setPower(-0.3);
            left.setPower(0.3);
            other.setPower(0.3);
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    private double getGyroRote()
    {
        double cVal = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        double step = Math.abs(gyroPreval-cVal);
        if(step<1)gyroVal += step;
        else gyroVal += step-Math.PI*2;
        gyroPreval = cVal;
        return gyroVal;
    }
}
