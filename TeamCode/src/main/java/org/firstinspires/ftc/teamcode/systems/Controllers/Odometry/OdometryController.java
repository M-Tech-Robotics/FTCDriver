package org.firstinspires.ftc.teamcode.systems.Controllers.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.modules.Executable;
import org.firstinspires.ftc.teamcode.systems.modules.subsystems.ThreadController;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryController extends ThreadController {

    // Encoder Motors //
    private final DcMotor leftEncoder;
    private final DcMotor rightEncoder;
    private final DcMotor horizontalEncoder;


    // Variables //
    private final double ticksPerInches = 8192;
    private final double robotEncoderWheelDistance = 7.125;
    private final double horizontalEncoderTickPerDegreeOffset = 4.001;

    private final Lock lock = new ReentrantLock();

    private volatile double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;

    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    private int verticalLeftEncoderPositionMultiplier = 1;

    private int verticalRightEncoderPositionMultiplier = 1;

    private int normalEncoderPositionMultiplier = 1;

    private String[] EncoderNames = {
            "IntakeMotor",
            "leftSlide",
            "rightSlide"
    };


    public OdometryController(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, int threadSleepDelay, Executable<Boolean> _opModeIsActive, Executable<Boolean> _isStopRequested) {
        super(_opModeIsActive, _isStopRequested);
        this.leftEncoder = verticalEncoderLeft;
        this.rightEncoder = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

    }

    public int[] getPorts() {
        int[] ports = new int[3];
        ports[0] = rightEncoder.getPortNumber();
        ports[1] = leftEncoder.getPortNumber();
        ports[2] = horizontalEncoder.getPortNumber();
        return ports;
    }

    public double getTicksPerInches() {
        return ticksPerInches;
    }

    public int returnRightEncoderPosition() {
        return verticalRightEncoderPositionMultiplier * rightEncoder.getCurrentPosition();
    }

    public int returnLeftEncoderPosition() {
        return verticalLeftEncoderPositionMultiplier * leftEncoder.getCurrentPosition();
    }

    public int returnHorizontalEncoderPosition() {
        return normalEncoderPositionMultiplier * horizontalEncoder.getCurrentPosition();
    }

    public int[] returnRaw() {
        int[] positions = new int[3];
        positions[0] = rightEncoder.getCurrentPosition();
        positions[1] = leftEncoder.getCurrentPosition();
        positions[2] = horizontalEncoder.getCurrentPosition();
        return positions;
    }

    public void setOrientation(double angle) throws InterruptedException {

        lock.lockInterruptibly();
        try {

            robotOrientationRadians = Math.toRadians(angle);

        } finally {
            lock.unlock();
        }
    }

    public void setPos(double X, double Y, double rot) throws InterruptedException {

        lock.lockInterruptibly();
        try {
            robotGlobalXCoordinatePosition = X * ticksPerInches;
            robotGlobalYCoordinatePosition = Y * ticksPerInches;
            robotOrientationRadians = Math.toRadians(rot);

        } finally {
            lock.unlock();
        }

    }


    public double getRot() {
        return Math.toDegrees(robotOrientationRadians) % 360;
    }


    public double getX() {

        return robotGlobalXCoordinatePosition / ticksPerInches;
    }


    public double getY() {

        return robotGlobalYCoordinatePosition / ticksPerInches;
    }

    public double[] getPos() {
        return new double[]{this.getX(), this.getY(), this.getRot()};
    }

    public void threadMain() throws InterruptedException {
        lock.lockInterruptibly();
        try {

            //Get Current Positions
            double leftChange = leftEncoder.getCurrentPosition() - previousVerticalLeftEncoderWheelPosition;
            double rightChange = rightEncoder.getCurrentPosition() - previousVerticalRightEncoderWheelPosition;

            //Calculate Angle
            double changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
            robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

            //Get the components of the motion
            double normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition() * normalEncoderPositionMultiplier);
            double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
            double horizontalChange = rawHorizontalChange - (changeInRobotOrientation * horizontalEncoderTickPerDegreeOffset);

            double p = ((rightChange + leftChange) / 2);

            //Calculate and update the position values
            robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p * Math.sin(robotOrientationRadians) + horizontalChange * Math.cos(robotOrientationRadians));
            robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p * Math.cos(robotOrientationRadians) - horizontalChange * Math.sin(robotOrientationRadians));

            previousVerticalLeftEncoderWheelPosition = leftEncoder.getCurrentPosition();
            previousVerticalRightEncoderWheelPosition = rightEncoder.getCurrentPosition();
            prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        } finally {
            lock.unlock();
        }
    }


    public void reverseLeftEncoder() {
        if (verticalLeftEncoderPositionMultiplier == 1) {
            verticalLeftEncoderPositionMultiplier = -1;
        } else {
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder() {
        if (verticalRightEncoderPositionMultiplier == 1) {
            verticalRightEncoderPositionMultiplier = -1;
        } else {
            verticalRightEncoderPositionMultiplier = 1;
        }
    }


    public void reverseHorizontalEncoder() {
        if (normalEncoderPositionMultiplier == 1) {
            normalEncoderPositionMultiplier = -1;
        } else {
            normalEncoderPositionMultiplier = 1;
        }
    }

    @Override
    protected void onEnd() {
    }
}
