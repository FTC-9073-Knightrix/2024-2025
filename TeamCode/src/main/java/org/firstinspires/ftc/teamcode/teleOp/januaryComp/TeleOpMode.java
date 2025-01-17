package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "1: IntoTheDeep TeleOp")
public class TeleOpMode extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void loop() {
        runMecanumDrive();

        runIntakeOuttake();
        runSpecimens();
        verticalSlideSystem();
        horizontalSlideSystem();
        switchGameMode();
        endGame();

        updateAttachments();
        addTelemetryToDriverStation();
    }
}






