package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "24-25 January IntoTheDeep TeleOp (Harrisonburg)")
public class TeleOpMode extends TeleOpMethods {
    boolean libCode = false;
    @Override
    public void loop() {
        runMecanumDrive();
        runClawIntake();
        runClawOuttake();
        runLeadScrew();
    }
}






