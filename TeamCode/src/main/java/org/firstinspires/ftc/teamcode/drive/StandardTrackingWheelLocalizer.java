package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.64370079 ; // in; distance between the left and right wheels 9.92519692
    public static double FORWARD_OFFSET = -5.81791355; //-6.01; // in; offset of the lateral wheel

//    public static double X_MULTIPLIER = 0.9855055030192697; // Multiplier in the X direction prev:1.00816614578
//    public static double Y_MULTIPLIER = 1.0048089; // Multiplier in the Y direction


    //0.9969846009 0.9968006152 0.9973564719374416
    public static double X_MULTIPLIER = 0.9973564719374416; // Multiplier in the X direction prev:0.9973564719374416
    public static double Y_MULTIPLIER = 1.0055238128260873; // Multiplier in the Y direction 1.0055238128260873

    public final String leftName = "leftFront";
    public final String rightName = "rightFront";
    public final String frontName = "leftRear";

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private DcMotorEx leftMotor, rightMotor, backMotor;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, leftName));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, rightName));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, frontName));

        frontEncoder.setDirection(Encoder.Direction.REVERSE);


//        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
