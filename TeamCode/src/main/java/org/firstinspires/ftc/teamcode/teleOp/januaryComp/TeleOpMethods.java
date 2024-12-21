package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    // Intake Finite State Machine
    protected class IntakeFSM {
        private IntakeState state;
        public ElapsedTime timer;

        public enum IntakeState {
            INTAKE_START,
            INTAKE_PICKUP,
            INTAKE_RETRACT
        }

        public IntakeFSM() {
            state = IntakeState.INTAKE_START;
            timer = new ElapsedTime();
        }

        public IntakeState getState () {return state;}
        public void setState (IntakeState state) {this.state = state;}
    }
    IntakeFSM intakeFSM = new IntakeFSM();

    // Outtake Finite State Machine
    protected class OuttakeFSM {
        private OuttakeState state;
        public ElapsedTime timer;

        public enum OuttakeState {
            DEFAULT,
            PICKUP,
            LIFT,
            DUMP,
            FLIP_BACK,
            DESCENT
        }

        public OuttakeFSM() {
            state = OuttakeState.DEFAULT;
            timer = new ElapsedTime();
        }

        public OuttakeState getState() {return state;}
        public void setState(OuttakeState state) {this.state = state;}
    }
    OuttakeFSM outtakeFSM = new OuttakeFSM();

    protected class SpecimenFSM {
        private SpecimenState state;

        public enum SpecimenState {
            GRAB
        }
    }
    // ------------------------------------ TELEOP VARIABLES ------------------------------------
    // Drive train speeds
    final double driveSpeed = 0.66;
    final double fastSpeed = 1.0;
    final double slowSpeed = 0.25;
    double finalSlowMode = 0.0;

    // TODO Change variables and servo program
    // Intake variables
    double intakeArmServoRot = 0.5;
    double intakeClawServoRot = 0.0;
    double intakeTwistServoRot = 0.0;

    final double INTAKE_ARM_DOWN_POS = 0.0;
    final double INTAKE_ARM_DEFAULT_POS = 0.5;
    final double INTKAE_ARM_TRANSFER = 1.0;

    // Outtake variables
    double outtakeArmServoRot = 0.5;
    double outtakeClawServoRot = 0.0;
    double outtakeTwistServoRot = 0.0;

    final double OUTTAKE_ARM_BACK_POS = 0.0;
    final double OUTTAKE_ARM_HOOK_POS = 0.5;
    final double OUTTAKE_ARM_DEFAULT_POS = 0.75;
    final double OUTTAKE_ARM_TRANSFER = 1.0;

    // Horiz Lift
    int liftPosHoriz = 0;
    int liftPosAdjHoriz = 0;
    double horizLinearPower = 0.0;
    final int HORIZ_MAX = 2000;

    // Vert Lift
    int liftPosVert = 0;
    int liftPosAdjVert = 0;
    double vertLinearPower = 0.0;
    final int VERT_MAX = 3600;

    // Hanger
    double hangerPower = 0.0;

    // Intake Color Sensor
    float gain = 22.0F;

    /* Once per loop, we will update this hsvValues array. The first element (0) will contain the
       hue, the second element (1) will contain the saturation, and the third element (2) will
       contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
       for an explanation of HSV color. */
    final float[] hsvValues = new float[3];
    NormalizedRGBA colors;
    // Mecanum
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
        switch (intakeFSM.state) {
            case INTAKE_START:
                intakeFSM.timer.reset();
                break;
            case INTAKE_PICKUP:
                intakeFSM.timer.reset();
                break;
            case INTAKE_RETRACT:
                intakeFSM.timer.reset();
                break;
            default:
                // should never be reached, as intakeState should never be null
                intakeFSM.setState(IntakeFSM.IntakeState.INTAKE_START);
        }
    }

    public void runClawSampleOuttake() {
        // --------------------------------------- CLAW SAMPLE OUTTAKE ---------------------------------------
        // TODO FILL OUT OUTTAKE STATE MACHINE
        switch (outtakeFSM.state) {
            case DEFAULT:
                outtakeFSM.timer.reset();
                break;
            case PICKUP:
                outtakeFSM.timer.reset();
                break;
            case LIFT:
                outtakeFSM.timer.reset();
                break;
            case DUMP:
                outtakeFSM.timer.reset();
                break;
            case FLIP_BACK:
                break;
            case DESCENT:
                outtakeFSM.timer.reset();
                break;
            default:
                // should never be reached, as outtakeStart should never be null
                outtakeFSM.setState(OuttakeFSM.OuttakeState.DEFAULT);
        }
    }

    public void runClawSpecimens() {
        // --------------------------------------- CLAW SPECIMENS ---------------------------------------
    }
    public void horizontalSlideSystem() {
        // If the outtake claw extends behind the robot,
        if (outtakeFSM.state == OuttakeFSM.OuttakeState.DUMP) {
            if (!horizSlideSensor.isPressed()) {horizLinearPower = -0.75;}
            else {horizLinearPower = 0.0;}
            return;
        }

        // Stop overextension and over retraction of horizontal linear motor
        liftPosHoriz = Math.abs(horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz);
        // if slide sensor touched or manual adjustment button pressed
        if (horizSlideSensor.isPressed() || gamepad2.dpad_left) {
            liftPosAdjHoriz = Math.abs(horizLinearMotor.getCurrentPosition());
        }

        if (gamepad2.right_stick_y > 0) {
            horizLinearPower = -gamepad2.right_stick_y * 0.5;
        } else if (gamepad2.right_stick_y < 0.0 && liftPosHoriz < HORIZ_MAX) {
            horizLinearPower = -gamepad2.right_stick_y * 0.5;
        } else { horizLinearPower = 0.0;}
    }
    public void runLeadScrew() {
        // --------------------------------------- LEAD SCREW ---------------------------------------
        // TODO FILL OUT LEAD SCREW CODE

    }

    public void getColors() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
    }
    public void updateAttachments() {
        // ----------------------------------- UPDATE ATTACHMENTS -----------------------------------
        intakeArmServoRot = Range.clip(intakeArmServoRot, 0.0, 1.0);
        intakeClawServoRot = Range.clip(intakeClawServoRot, 0.0, 1.0);
        intakeTwistServoRot = Range.clip(intakeTwistServoRot, 0.0, 1.0);

        outtakeArmServoRot = Range.clip(outtakeArmServoRot, 0.0, 1.0);
        outtakeClawServoRot = Range.clip(outtakeClawServoRot, 0.0, 1.0);
        outtakeTwistServoRot = Range.clip(outtakeTwistServoRot, 0.0, 1.0);

        intakeArmServo.setPosition(intakeArmServoRot);
        intakeClawServo.setPosition(intakeClawServoRot);
        intakeTwistServo.setPosition(intakeTwistServoRot);

        outtakeArmServo.setPosition(outtakeArmServoRot);
        outtakeClawServo.setPosition(outtakeClawServoRot);
        outtakeTwistServo.setPosition(outtakeTwistServoRot);

        vertLinearMotor.setPower(vertLinearPower);
        horizLinearMotor.setPower(horizLinearPower);
        hangerMotor.setPower(hangerPower);
    }

    public void addTelemetryToDriverStation() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("g2LStickY, g2RStickY", gamepad2.left_stick_y + ", " + gamepad2.right_stick_y );
        telemetry.addData("Gyro: ", "Yaw: " + String.format(Locale.US, "%.2f", orientation.getYaw(AngleUnit.DEGREES))
                                                + "Roll: " + String.format(Locale.US, "%.2f", orientation.getRoll(AngleUnit.DEGREES))
                                                + "Pitch: " + String.format(Locale.US, "%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("Slowmode: ", finalSlowMode);
        telemetry.addData("Intake State:", intakeFSM.getState());
        telemetry.addData("Outtake State:", outtakeFSM.getState());
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addData("Distance", "%.3f", intakeDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    public double incrementServoRot(double currentRot, double amount, double min, double max) {
        if (max > min) throw new IllegalArgumentException("Min must be less than max");
        return Range.clip(currentRot, min, max) + amount;
    }

    public void runLiftToPosition(DcMotor motor, int target, double power) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(target);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

