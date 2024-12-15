package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    // ENUMS & VARIABLES FOR FINITE STATE MACHINE
    public enum IntakeState {
        INTAKE_START,
        INTAKE_PICKUP,
        INTAKE_RETRACT
    }
    IntakeState intakeState = IntakeState.INTAKE_START;
    ElapsedTime intakeTimer = new ElapsedTime();
    public enum OuttakeState {
        OUTTAKE_START,
        OUTTAKE_PICKUP,
        OUTTAKE_LIFT,
        OUTTAKE_DUMP,
        OUTTAKE_RETRACT
    }
    OuttakeState outtakeState = OuttakeState.OUTTAKE_START;
    ElapsedTime outtakeTimer = new ElapsedTime();

    // ------------------------------------ TELEOP VARIABLES ------------------------------------
    //Drive train speeds
    final double driveSpeed = 0.66;
    final double fastSpeed = 1.0;
    final double slowSpeed = 0.25;
    double finalSlowMode = 0.0;

    //Servo variables
    double armDownPos = 0.68;
    double armDefaultPos = 0.5;
    double armAtBasketPos = 0.05;

    //Motor variables
    int liftPosHoriz = 0;
    int liftPosAdjHoriz = 0;
    double horizLinearPower = 0.0;

    int liftPosVert = 0;
    int liftPosAdjVert = 0;
    final double maximumVertExtend = 3600;
    double vertLinearPower = 0.0;
    boolean robotCentric = false;

    public void runMecanumDrive(){
        // ---------------------------------------MECANUM DRIVE ---------------------------------------
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
            imu.resetYaw();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

    public void runClawIntake() {
        // --------------------------------------- CLAW INTAKE ---------------------------------------
        // TODO FILL OUT OUTTAKE STATE MACHINE
        switch (intakeState) {
            case INTAKE_START:
                break;
            case INTAKE_PICKUP:
                break;
            case INTAKE_RETRACT:
                break;
            default:
                // should never be reached, as intakeState should never be null
                intakeState = IntakeState.INTAKE_START;
        }
    }

    public void runClawOuttake() {
        // --------------------------------------- CLAW OUTTAKE ---------------------------------------
        // TODO FILL OUT OUTTAKE STATE MACHINE
        switch (outtakeState) {
            case OUTTAKE_START:
                break;
            case OUTTAKE_PICKUP:
                break;
            case OUTTAKE_LIFT:
                break;
            case OUTTAKE_DUMP:
                break;
            case OUTTAKE_RETRACT:
                break;
            default:
                // should never be reached, as outtakeStart should never be null
                outtakeState = OuttakeState.OUTTAKE_START;
        }
    }

    public void runLeadScrew() {
        // --------------------------------------- LEAD SCREW ---------------------------------------
        // TODO FILL OUT LEAD SCREW CODE

    }

    public void updateAttachments() {
        // ----------------------------------- UPDATE ATTACHMENTS -----------------------------------

    }

    @SuppressLint("DefaultLocale")
    public void addTelemetryToDriverStation() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("g2LStickY, g2RStickY", gamepad2.left_stick_y + ", " + gamepad2.right_stick_y );
        telemetry.addData("Gyro: ", "Yaw: " + String.format("%.2f", orientation.getYaw(AngleUnit.DEGREES))
                + "Roll: " + String.format("%.2f", orientation.getRoll(AngleUnit.DEGREES))
                + "Pitch: " + String.format("%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("Slowmode: ", finalSlowMode);
        telemetry.update();
    }
}
