package org.firstinspires.ftc.teamcode;
public class GoBilda312DcMotorData extends MotorData {

    public GoBilda312DcMotorData() {
        countsPerMotorRev = 28 * 4;
        gearRatio = 19.2;
        wheelDiameterInches = 0;
        maxMotorRpm = 5900;
        maxMotorRps = maxMotorRpm / 60.0;
        maxCountsPerSec = maxMotorRps * countsPerMotorRev;
    }
}
