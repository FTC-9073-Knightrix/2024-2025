package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "2: IntoTheDeep TeleOp GOBILDA PINPOINT")
public class TeleOpModePinpoint extends TeleOpMethods {
    boolean libCode = false;

    @Override
    public void runMecanumDrive() {
        // Only update the heading because that is all you need in Teleop
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

        //Setting boolean hold
        if(gamepad1.right_bumper) {
            //Slowmode
            finalSlowMode = slowSpeed;

        } else if (gamepad1.left_bumper) {
            //Fastmode
            finalSlowMode = fastSpeed;
        } else {
            //Regular
            finalSlowMode = driveSpeed;
        }


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * .8;

        if (gamepad1.y) {
//            imu.resetYaw();
            pinpoint.resetPosAndIMU();
//            pinpoint.recalibrateIMU();
        }

        orientation = imu.getRobotYawPitchRollAngles();

        // IF U WANT TO CHANGE BETWEEN PP AND IMU U CHANGE BOTHEADING VARIABLE
//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = pinpoint.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.set(frontLeftPower * finalSlowMode);
        leftBack.set(backLeftPower * finalSlowMode);
        rightFront.set(frontRightPower * finalSlowMode);
        rightBack.set(backRightPower * finalSlowMode);

        robotCentric = false;
    }

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






