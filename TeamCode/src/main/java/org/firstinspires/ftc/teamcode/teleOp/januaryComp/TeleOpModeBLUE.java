package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "2: BLUE ALLIANCE IntoTheDeep TeleOp")
public class TeleOpModeBLUE extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void init() {
        super.init();
        allianceColor = GameColors.BLUE;
    }
    @Override
    public void loop() {
        runMecanumDrive();

        getColors();
        runIntakeOuttake();
        runSpecimens();
        runLeadScrew();
        horizontalSlideSystem();
        switchGameMode();
        endGame();

        updateAttachments();
        addTelemetryToDriverStation();
    }
}






