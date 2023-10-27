package org.firstinspires.ftc.teamcode;
public class GoBilda312DcMotorData {
    public static double countsPerMotorRev = 28;
    public static double gearRatio = 19.2;
    public static double wheelDiameterInches = 44.0/25.4;
    public static double countsPerGearboxRev = gearRatio * countsPerMotorRev;
    public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
    public static double countsPerInch = countsPerGearboxRev/wheelCircumferenceInches;
    public static double maxMotorRpm = 5900;
    public static double maxMotorRps = maxMotorRpm / 60.0;
    public static double maxCountsPerSec = maxMotorRps * countsPerMotorRev;

}
