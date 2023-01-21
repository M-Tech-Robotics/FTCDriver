package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
    /*
     * Dark Matter Robot Specifications:
     * Width: 14.15 in
     * Length: 14.4 in
     * Height: 14 in
     *
     * Wheel Diameter:
     * Wheel Radius: 
     *
     * Track Width = 10 in
     *
     * Motors:
     *
     *
     *
     *
     * HD HEX Motor Specifications:
     * RPM: 300
     * Encoder Ticks: 28
     * Output Shaft Ticks: 560
     */

//    IPv4 Address. . . . . . . . . . . : 192.168.43.83(Preferred)
//    Subnet Mask . . . . . . . . . . . : 255.255.255.0
//    Lease Obtained. . . . . . . . . . : Thursday, January 19, 2023 4:40:08 PM
//    Lease Expires . . . . . . . . . . : Thursday, January 19, 2023 5:40:08 PM
//    Default Gateway . . . . . . . . . : 192.168.43.1
//    DHCP Server . . . . . . . . . . . : 192.168.43.1



    /*
     * These are motor constants that should be listed online for your motors.
     *
     * HD HEX Motor Specifications:
     * RPM: 300
     * Encoder Ticks: 28
     * Output Shaft Ticks: 560
     */
    public static final double TICKS_PER_REV = 537.7    ; //560 is Output Shaft
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
//    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
//            14.426575317778232); //getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

//    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(21, 0, 3,
//            15); //getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV)


    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));


//    kV = 0.01776, kStatic = 0.12010 (R^2 = 0.96)
//    kA = 0.00018 (R^2 = 0.10)



    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 7.90; //10

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
//    public static double kV = 0.01978;
//    public static double kA = 0.00020;
//    public static double kStatic = 0.11900;

//    public static double kV = 0.01793;
//    public static double kA = 0.00015;
//    public static double kStatic = 0.11220;

    //kV = 0.01813, kStatic = 0.10998 (R^2 = 0.97)
    //
    public static double kV = 0.01813;
    public static double kA = 0.00001;
    public static double kStatic = 0.10998;


    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */


//    Max Velocity: 52.335779374726876
//    Voltage Compensated kF: 14.147060881910702





    public static double MAX_VEL = 51.34267351154658;
    public static double MAX_ACCEL = 20;

//    Max Angular Velocity (deg): 33.10422960504306
//    Max Angular Velocity (rad): 0.5777778029441833


    public static double MAX_ANG_VEL = 5.816545437207842;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
