package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "AutoFarRed528", group = "Linear OpMode")
public class AutoFarRed528 extends AutoMaster {

public AutoFarRed528(){    
    super(1);
    }
    public void runOpMode() {
        bot = new AutoBot(this);
        if (alliance == 1) {
            allianceDirection = -1;
        } else {
            allianceDirection = 1;
        }

        waitForStart();

        // Raise lift, raise wrist, close grabber
        setToCruisingPosition();

        // Move to the center of the spike mark square
        moveToCenterOfSquare();

        // Determine prop position, and place the purple pixel on the spike mark
        dsPlacePurplePixel();

        if (alliance == 1)
        {
            if (propPosition == Position.NEAR){targetAprilTagNumber = 1;}

            if (propPosition == Position.MIDDLE){targetAprilTagNumber = 2;}

            if (propPosition == Position.FAR){targetAprilTagNumber = 3;}
        }
        else if (alliance == 2)
        {
            if (propPosition == Position.FAR){targetAprilTagNumber = 4;}

            if (propPosition == Position.MIDDLE){targetAprilTagNumber = 5;}

            if (propPosition == Position.NEAR){targetAprilTagNumber = 6;}
        }
    }    
        
        
        
        
        
        
        
        
        
        
        
    
}