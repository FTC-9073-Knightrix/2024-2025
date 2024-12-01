package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IntoTheDeep TeleOp 24-25")
public class TeleOpMode extends TeleOpMethods {
    boolean libCode = false;
    @Override
    public void loop() {
        runMecanumDrive();
        intakeOuttakeSystem();
        basketSystem();
        clawSystem();
        verticalSlideSystem();
        horizonalSlideSystem();
        updateAttachments();
        addTelemetryToDriverStation();
    }
}
