package org.firstinspires.ftc.teamcode.teleOp.decemberComp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IntoTheDeep TeleOp 24-25 (Gathersburg)")
@Disabled
public class OldTeleOpMode extends OldTeleOpMethods {
    boolean libCode = false;
    @Override
    public void loop() {
        runMecanumDrive();
        intakeOuttakeSystem();
        basketSystem();
        clawSystem();
        verticalSlideSystem();
        horizonalSlideSystem();
        leadScrewSystem();
        updateAttachments();
        addTelemetryToDriverStation();
    }
}






