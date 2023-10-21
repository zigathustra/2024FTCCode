package org.firstinspires.ftc.teamcode;

public class RevUltra20DcMotorData extends MotorData {

    public RevUltra20DcMotorData() {
        countsPerMotorRev = 28 * 4;
        gearRatio = 5.23 * 2.89;
        wheelDiameterInches = 75.0 / 25.4;
        maxMotorRpm = 5900;
        maxMotorRps = maxMotorRpm / 60.0;
        maxCountsPerSec = maxMotorRps * countsPerMotorRev;
    }
}
