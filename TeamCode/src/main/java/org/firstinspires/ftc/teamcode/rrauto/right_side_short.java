package org.firstinspires.ftc.teamcode.rrauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "right_side (med. pole)", group = "Pushbot")
//@Disabled
public class right_side_short extends rrAutoComp3 {

    // Constructor
    public right_side_short() {
        position = "right_side";
        skipPolesConf = 1;
    }
}