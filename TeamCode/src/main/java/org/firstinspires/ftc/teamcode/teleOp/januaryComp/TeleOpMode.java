package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "1: IntoTheDeep TeleOp")
public class TeleOpMode extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void loop() {
//        if (gamepad2.dpad_up) {
//            outtakeClawServoRot = incrementServoRot(outtakeClawServoRot, 0.01, 0, 1);
//        }
//        if (gamepad2.dpad_down) {
//            outtakeClawServoRot = incrementServoRot(outtakeClawServoRot, -0.01, 0, 1);
//        }
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






