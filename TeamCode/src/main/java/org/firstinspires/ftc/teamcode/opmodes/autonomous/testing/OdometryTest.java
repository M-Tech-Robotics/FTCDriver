package org.firstinspires.ftc.teamcode.opmodes.autonomous.testing;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "Calibration", group = "Testing")
public class OdometryTest extends LinearOpMode {
    private final static double PIVOT_SPEED = .25;
    private final static double COUNTS_PER_INCH = 8192;

    private final static String rfName = "rightFront";
    private final static String rbName = "rightRear";
    private final static String lfName = "leftFront";
    private final static String lbName = "leftRear";
    private final static File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private final static File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    private final static String verticalLeftEncoderName = "leftFront";
    private final static String verticalRightEncoderName = "rightFront";
    private final static String horizontalEncoderName = "leftRear";
    private final ElapsedTime timer = new ElapsedTime();
    //Drive motors
    private MotorEx right_front, right_back, left_front, left_back;
    //Odometry Wheels
    private Encoder verticalLeft, verticalRight, horizontal;
    //IMU Sensor
    private BNO055IMU imu;
    public static final double WHEEL_DIAMETER = 1.3779528;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.update();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();


        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        double frPower = PIVOT_SPEED;
        double flPower = PIVOT_SPEED;
        double brPower = PIVOT_SPEED;
        double blPower = PIVOT_SPEED;

        //MUST TURN 90 DEGREES EXACTLY
        //PLAY WITH SPEED AND DEGREE OF TURN TO GET THE ROBOT TO END UP AS CLOSE TO 90 DEGREES AS POSSIBLE
        while (getZAngle() < 89.5 && opModeIsActive()) {
            right_front.motorEx.setPower(frPower);
            right_back.motorEx.setPower(brPower);
            left_front.motorEx.setPower(flPower);
            left_back.motorEx.setPower(blPower);
            if (getZAngle() < 60) {
                right_front.motorEx.setPower(frPower);
                right_back.motorEx.setPower(brPower);
                left_front.motorEx.setPower(flPower);
                left_back.motorEx.setPower(blPower);
            } else {
                right_front.motorEx.setPower(frPower / 2.0);
                right_back.motorEx.setPower(brPower / 2.0);
                left_front.motorEx.setPower(flPower / 2.0);
                left_back.motorEx.setPower(blPower / 2.0);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0,0,0,0);

        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive()) {
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();



        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */

        double encoderDifference = Math.abs(-verticalLeft.getPosition()) + (Math.abs(verticalRight.getPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);

        double horizontalTickOffset = horizontal.getPosition() / Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));


        while (opModeIsActive()) {

            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -verticalLeft.getPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getPosition());
            telemetry.addData("Horizontal Position", horizontal.getPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);


            telemetry.addData("Vertical Left Position", verticalLeft.getPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getPosition());
            telemetry.addData("Horizontal Position", horizontal.getPosition());

            //Update values
            telemetry.update();
        }

    }

    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        left_front = new MotorEx(hardwareMap, lfName);
        right_front = new MotorEx(hardwareMap, rfName);
        left_back = new MotorEx(hardwareMap, lbName);
        right_back = new MotorEx(hardwareMap, rbName);

        verticalLeft = left_front.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        verticalRight = right_front.encoder.setDistancePerPulse(DISTANCE_PER_PULSE); //backLeft.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        horizontal = left_back.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        right_front.resetEncoder();
        right_back.resetEncoder();
        left_front.resetEncoder();
        left_back.resetEncoder();

        right_front.setRunMode(Motor.RunMode.RawPower);
        right_back.setRunMode(Motor.RunMode.RawPower);
        left_front.setRunMode(Motor.RunMode.RawPower);
        left_back.setRunMode(Motor.RunMode.RawPower);

        verticalLeft.reset();
        verticalRight.reset();
        horizontal.reset();


        right_front.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */

        horizontal.setDirection(Motor.Direction.REVERSE);

        left_front.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }

    private void setPowerAll(double rf, double rb, double lf, double lb) {
        right_front.motorEx.setPower(rf);
        right_back.motorEx.setPower(rb);
        left_front.motorEx.setPower(lf);
        left_back.motorEx.setPower(lb);
    }
}
