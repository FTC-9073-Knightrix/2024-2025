package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "1: RED ALLIANCE IntoTheDeep TeleOp")
public class TeleOpModeRED extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void init() {
        super.init();
        allianceColor = GameColors.RED
        ;
    }
    @Override
    public void loop() {
        if (gamepad2.dpad_left) {
            outtakeClawServoRot = incrementServoRot(outtakeClawServoRot, -0.01, 0, 1);
        }
        if (gamepad2.dpad_right) {
            outtakeClawServoRot = incrementServoRot(outtakeClawServoRot, 0.01, 0, 1);

        }
        runMecanumDrive();

        getColors();
        runIntakeOuttake();
        runSpecimens();
        horizontalSlideSystem();
        switchGameMode();
        endGame();

        updateAttachments();
        addTelemetryToDriverStation();
    }
}






