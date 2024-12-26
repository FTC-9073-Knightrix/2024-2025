package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "1: RED ALLIANCE IntoTheDeep TeleOp")
public class TeleOpModeRED extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void init() {
        super.init();
        allianceColor = GameColors.RED;
    }
    @Override
    public void loop() {
        runMecanumDrive();

        getColors();
        runClawIntake();
        runClawOuttake();
        runLeadScrew();
        horizontalSlideSystem();
        switchGameMode();
        endGame();

        updateAttachments();
        addTelemetryToDriverStation();
    }
}






