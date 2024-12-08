package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="EmptyAuto", group="Autonomous")
public class EmptyAuto extends AutoMethods{
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
}
