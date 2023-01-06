package org.firstinspires.ftc.teamcode.systems.Controllers.navigation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.modules.interfaces.ThreadedInterface;

public interface Odometry extends ThreadedInterface {

    int[] getPorts();


    void showPosition(Telemetry telemetry);

    double getCOUNTS_PER_INCH();

    int returnRightEncoderPosition();

    int returnLeftEncoderPosition();

    int returnHorizontalEncoderPosition();

    int[] returnRaw();

    void setOrientation(double angle) throws InterruptedException;

    void threadMain() throws InterruptedException;

    void reverseLeftEncoder();

    void reverseRightEncoder();

    void reverseHorizontalEncoder();


}
