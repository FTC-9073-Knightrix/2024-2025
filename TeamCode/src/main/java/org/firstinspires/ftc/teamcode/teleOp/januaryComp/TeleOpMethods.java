package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

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
    @Override
    public void init() {
        super.init();
    }
    // Intake Finite State Machine
    protected class IntakeFSM {
        private IntakeState state;
        public ElapsedTime timer;
        public boolean justSwitched;

        public enum IntakeState {
            DEFAULT,
            PICKUP,
            TRANSFER
        }

        public IntakeFSM() {
            state = IntakeState.DEFAULT;
            timer = new ElapsedTime();
            justSwitched = true;
        }

        public IntakeState getState () {return state;}
        public void setState (IntakeState state) {this.state = state;}
    }
    IntakeFSM intakeFSM = new IntakeFSM();

    // Outtake Finite State Machine
    protected class OuttakeFSM {
        private OuttakeState state;
        public ElapsedTime timer;
        public boolean justSwitched;

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
            justSwitched = true;
        }

        public OuttakeState getState() {return state;}
        public void setState(OuttakeState state) {this.state = state;}
    }
    OuttakeFSM outtakeFSM = new OuttakeFSM();

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

    final double INTAKE_CLAW_CLOSE = 0.6;
    final double INTAKE_CLOW_OPEN = 1.0;

    final double INTAKE_ARM_DOWN = 0.0;
    final double INTAKE_ARM_DEFAULT = 0.5;
    final double INTKAE_ARM_TRANSFER = 1.0;

    // Outtake variables
    double outtakeArmServoRot = 0.5;
    double outtakeClawServoRot = 0.0;
    double outtakeTwistServoRot = 0.0;

    final double OUTTAKE_ARM_BACK = 0.0;
    final double OUTTAKE_ARM_HOOK = 0.5;
    final double OUTTAKE_ARM_DEFAULT = 0.75;
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
    final float minIntensity = 0.4F;
    NormalizedRGBA colors;
    enum GameColors {RED, YELLOW, BLUE, NONE}
    GameColors colorMatch = GameColors.NONE;
    GameColors allianceColor; // MUST be initialized in Op Mode as only RED or BLUE

    // Mecanum
    boolean robotCentric = false;

    // TODO ---------------------------------------MECANUM DRIVE ---------------------------------------
    public void runMecanumDrive(){
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

    // --------------------------------------- CLAW INTAKE ---------------------------------------
    public void runClawIntake() {
        // TODO FILL OUT OUTTAKE STATE MACHINE
        switch (intakeFSM.state) {
            case DEFAULT:
                if (intakeFSM.justSwitched) {
                    intakeFSM.timer.reset();
                    intakeClawServoRot = INTAKE_CLOW_OPEN;
                    intakeFSM.justSwitched = false;
                }
                runIntakeTwist();
                runIntakeGrab();
                if (gamepad2.a) {
                    intakeFSM.setState(IntakeFSM.IntakeState.PICKUP);
                    intakeFSM.justSwitched = true;
                }
                break;
            case PICKUP:
                if (intakeFSM.justSwitched) {
                    intakeFSM.timer.reset();
                    intakeFSM.justSwitched = false;
                    if (!sampleColorIsAcceptable()) {
                        intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT);
                    }
                }
                break;
            case TRANSFER:
                if (intakeFSM.justSwitched) {
                    intakeFSM.timer.reset();
                    intakeFSM.justSwitched = false;
                }
            default:
                // should never be reached, as intakeState should never be null
                intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT);
                intakeFSM.justSwitched = true;
        }
    }

    // --------------------------------------- CLAW OUTTAKE ---------------------------------------
    public void runClawOuttake() {
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
        // Color sensor get the RGB values
        colors = colorSensor.getNormalizedColors();
        float redGreenDifference = Math.abs(colors.red-colors.green);

        // Determine which color the robot sees

        // (Red & Green -> Yellow)
        // Checks that intense red & green are detected, and that they are about the same value -> Yellow color
        if ((colors.red > minIntensity && colors.green > minIntensity) && redGreenDifference < 0.05) {
            colorMatch = GameColors.YELLOW;
        }
        // Checks that intense red is detected, and that red value is greater than blue value -> Red color
        else if ((colors.red > minIntensity && Math.max(colors.red, colors.blue) == colors.red)) {
            colorMatch = GameColors.RED;
        }
        // Checks that intense blue is detected, and that blue value is greater than red value -> Blue color
        else if ((colors.blue > minIntensity) && Math.max(colors.red, colors.blue) == colors.blue) {
            colorMatch = GameColors.BLUE;
        }
        else {colorMatch = GameColors.NONE;}
    }

    public boolean sampleColorIsAcceptable() {
        boolean colorIsNotAnAllianceColor = (colorMatch != GameColors.RED && colorMatch != GameColors.BLUE);
        boolean colorMatchesAllianceColor = (colorMatch == allianceColor);
        return (colorIsNotAnAllianceColor || colorMatchesAllianceColor);
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

    public void runIntakeTwist() {
        if (gamepad2.left_bumper) {
            intakeTwistServoRot = incrementServoRot(intakeTwistServoRot, -0.01, 0.0, 1.0);
        }
        else if (gamepad2.right_bumper) {
            intakeTwistServoRot = incrementServoRot(intakeTwistServoRot, 0.01, 0.0, 1.0);
        }
    }

    public void runIntakeGrab() {
        if (gamepad2.right_trigger >= 0.75) intakeClawServoRot = INTAKE_CLAW_CLOSE;
        else if (gamepad2.left_trigger >= 0.75) intakeClawServoRot = INTAKE_CLOW_OPEN;
    }

    public void runAutoIntakeTransfer() {
        horizLinearPower = 0.0;
    }

    public double incrementServoRot(double currentRot, double amount, double min, double max) {
        if (max > min) throw new IllegalArgumentException("Min must be less than max");
        return Range.clip(currentRot, min, max) + amount;
    }

    public void runLiftToPosition(DcMotor motor, int target, double power) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(target);
        motor.setPower(power);
    }

    public void addTelemetryToDriverStation() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("g2LStickY, g2RStickY", gamepad2.left_stick_y + ", " + gamepad2.right_stick_y );
        telemetry.addData("Gyro: ", "Yaw: " + String.format(Locale.US, "%.2f", orientation.getYaw(AngleUnit.DEGREES))
                                                + "Roll: " + String.format(Locale.US, "%.2f", orientation.getRoll(AngleUnit.DEGREES))
                                                + "Pitch: " + String.format(Locale.US, "%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("Slowmode: ", finalSlowMode);

        telemetry.addData("Intake State:", intakeFSM.getState());
        telemetry.addData("Intake Timer", intakeFSM.timer.time());

        telemetry.addData("Outtake State:", outtakeFSM.getState());
        telemetry.addData("Outtake Timer", outtakeFSM.timer.time());

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Color Match", colorMatch);
        telemetry.addData("Distance", "%.3f", intakeDistanceSensor.getDistance(DistanceUnit.CM));

        telemetry.update();
    }
}

