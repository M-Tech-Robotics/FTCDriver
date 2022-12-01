package org.firstinspires.ftc.teamcode.systems.Controllers.linearSlide.NoEncoder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide.SlideHeight;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;


/**
 * Linear Slide Controller
 * A class to control linear leftSlide by controlling the positions
 */

public class LinearSlide1 {
    // Motor Objects //
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    RevBulkData bulkData;
    ExpansionHubEx expansionHub1, expansionHub2;

    long lastLoopTime = System.nanoTime();
    // Slide Info //
    public boolean updateSlideLength = true;
    public double slideTickToInch = 25.1372713591;
    public int loops = 0;
    public double loopSpeed = 0;

    public static double kPSlides = 0.2, kISlides = 0.05, slidesI = 0;

    // Slide Data //
    public double slideExtensionLength = 0;
    public double targetSlideExtensionLength = 0;

    public double currentTargetSlidesPose = 0;

    public double slidesPower;

    public double slidesOffset;

    public double currentSlidesSpeed = 0, slidesSpeed = 0;
    double targetSlidesPose = 0;

    private final ElapsedTime slideResetTimer = new ElapsedTime();

    // Slide Data //
    private SlideHeight currentLevel = SlideHeight.Floor;
    private final FtcDashboard dashboard;


    public LinearSlide1(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public void ResetSlides(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void getEncoders(){
        bulkData = expansionHub1.getBulkInputData(); //expansionHub2.getBulkInputData();

        if (bulkData != null) {
            slideExtensionLength = bulkData.getMotorCurrentPosition(rightSlide) / slideTickToInch;
            currentSlidesSpeed = bulkData.getMotorVelocity(rightSlide) / slideTickToInch;
        }
    }

    public void updateSlidesLength(){
        double maxSlideSpeed = 42/0.5; //42 inches per second
        double dif = (targetSlidesPose) - currentTargetSlidesPose;

        currentTargetSlidesPose += Math.signum(dif) * Math.min(slidesSpeed * loopSpeed * maxSlideSpeed,1.0);

        if (Math.abs(dif) <= 1){
            currentTargetSlidesPose = targetSlidesPose;
        }

        double p = (targetSlidesPose - slideExtensionLength) * kPSlides;
        double kStatic = Math.signum(targetSlidesPose - slideExtensionLength) * slidesSpeed/2.0;

        if (Math.abs(targetSlidesPose - slideExtensionLength) <= 3){ // we are within 3 from the end
            p /= 2;
            if (targetSlidesPose - slideExtensionLength >= 0.5){ // the target is greater than the current by 0.5 we engage the PID to get it to 0
                kStatic = 0.175 + slideExtensionLength * 0.002;
                if (loops >= 2) {
                    slidesI += (targetSlidesPose - slideExtensionLength) * loopSpeed * kISlides;
                }
            }
            else if (currentTargetSlidesPose - slideExtensionLength <= -0.5) { // the target is less than the current by 0.5 we slowly extend it back
                kStatic = -0.15 - (1.0/currentTargetSlidesPose) * 0.07;
                slidesI = 0;
                p /= 2.0;
            }
            else{// We are within +- 0.5 which means we use a holding power
                p = 0.085;
                kStatic = 0;
                slidesI = 0;
            }
        }

        else if (targetSlidesPose - slideExtensionLength < -3){ // We are more than 3 away so we go backward slowly
            kStatic = -0.1145; //-0.3 => 0.2 "ur bad" ~ @HudsonKim => 0.15
            slidesI = 0;
            p /= 4.0; // making it go back slower
        }

        else if (targetSlidesPose - slideExtensionLength > 3){ // We are more than 3 away so we go forward at max speed
            kStatic = slidesSpeed;
            p = 0;
            slidesI = 0;
        }

        slidesPower = kStatic + p + slidesI;
        leftSlide.setPower(slidesPower);
        rightSlide.setPower(slidesPower);
    }


    public void setSlidesLength(double inches){
        targetSlidesPose = inches;
        slidesSpeed = 1;
    }

    public void setSlidesLength(double inches, double speed){
        targetSlidesPose = inches;
        slidesSpeed = speed;
    }


    public void Update() {
        loops ++;
        getEncoders();

        long currentTime = System.nanoTime();
        if (loops == 1){
            lastLoopTime = currentTime;
        }

        loopSpeed = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime;

        if (updateSlideLength) {
            updateSlidesLength();
        }
        else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Slide Mode", "Teleop");
        packet.put("Slide Speed", currentSlidesSpeed);
        packet.put("Slides Length", slideExtensionLength);
        packet.put("Slides Power", slidesPower);
        packet.put("Target Slide Length 1", currentTargetSlidesPose);
        packet.put("Target Slide Length", targetSlidesPose);
        packet.put("Loop Speed", loopSpeed);
        dashboard.sendTelemetryPacket(packet);
    }


}
