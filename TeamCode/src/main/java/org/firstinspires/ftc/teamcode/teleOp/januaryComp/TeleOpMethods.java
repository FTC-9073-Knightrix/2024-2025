package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
        ElapsedTime timer;
        boolean slideWasJustRetracted;
        boolean clawClosedInitiated;
        private boolean justSwitched;

        public enum IntakeState {
            DEFAULT,
            PICKUP,
            SAMPLE_RETRACT,
            SPECIMEN_RETRACT,
            TRANSFER
        }

        public IntakeFSM() {
            state = IntakeState.DEFAULT;
            timer = new ElapsedTime();
            justSwitched = true;
            slideWasJustRetracted = false;
            clawClosedInitiated = false;
        }

        public IntakeState getState () {return state;}
        public void setState (IntakeState state) {this.state = state;}
        public boolean justSwitched() {return justSwitched;}
        public void setJustSwitched(boolean b) {justSwitched = b;}
    }
    IntakeFSM intakeFSM = new IntakeFSM();

    // Outtake Finite State Machine
    protected class OuttakeFSM {
        private OuttakeState state;
        public ElapsedTime timer;
        public boolean justSwitched;

        public enum OuttakeState {
            DEFAULT,

            // STATES FOR SAMPLE TRANSFER
            READY_TO_PICKUP,
            PICKUP,
            LIFT,
            DUMP,
            FLIP_BACK,
            DESCENT,

            // STATES FOR SPECIMEN TRANSFER
            READY_TO_GRAB_SPEC,
            GRAB_AND_FLIP_SPEC,
            SPECIMEN_HANG,
            RETURN_TO_GRABBING
        }

        public OuttakeFSM() {
            state = OuttakeState.DEFAULT;
            timer = new ElapsedTime();
            justSwitched = true;
        }

        public OuttakeState getState() {return state;}
        public void setState(OuttakeState state) {this.state = state;}
        public boolean justSwitched() {return justSwitched;}
        public void setJustSwitched(boolean b) {justSwitched = b;}
    }
    OuttakeFSM outtakeFSM = new OuttakeFSM();

    enum GameMode {
        SAMPLE,
        SPECIMEN
    }
    GameMode gameMode = GameMode.SAMPLE;
    // ------------------------------------ TELEOP VARIABLES ------------------------------------
    // Game variables
    boolean initiatedEndGame = false;
    boolean allPressed = (g2RightTriggerPressed && g2LeftTriggerPressed && gamepad2.left_bumper && gamepad2.right_bumper);
    double gameModeCurrent = Double.MAX_VALUE;

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

    final double INTAKE_CLAW_CLOSE = 0.52;
    final double INTAKE_CLAW_OPEN = 1.0;

    final double INTAKE_TWIST_STRAIGHT = 0.5;

    final double INTAKE_ARM_DOWN = 0.0;
    final double INTAKE_ARM_DEFAULT = 0.5;
    final double INTAKE_ARM_TRANSFER = 1.0;

    // Outtake variables
    double outtakeArmServoRot = 0.5;
    double outtakeClawServoRot = 0.0;
    double outtakeTwistServoRot = 0.0;

    final double OUTTAKE_ARM_BACK = 1.0;
    final double OUTTAKE_ARM_HOOK = 0.75;
    final double OUTTAKE_ARM_DEFAULT = 0.5;
    final double OUTTAKE_ARM_TRANSFER = 0.0;

    // Horiz Lift
    int liftPosHoriz = 0;
    int liftPosAdjHoriz = 0;
    double horizLinearPower = 0.0;
    final int HORIZ_MAX = 2000;
    final int TRANSFER_TARGET = 100;

    // Vert Lift
    int liftPosVert = 0;
    int liftPosAdjVert = 0;
    double vertLinearPower = 0.0;
    final int VERT_MAX = 3600;

    // Hanger
    double hangerPower = 0.0;

    // Intake Color Sensor
    final float minColorIntensity = 0.2F;
    NormalizedRGBA colors = new NormalizedRGBA();
    boolean useColorSensor = true;  // Choose whether to let the color sensor make judgements on the block to automatically reject it
    enum GameColors {RED, BLUE, NONE}
    GameColors colorMatch = GameColors.NONE;
    GameColors allianceColor; // MUST be initialized in Op Mode as only RED or BLUE

    // Mecanum
    boolean robotCentric = false;

    // TODO ---------------------------------------MECANUM DRIVE ---------------------------------------
    public void runMecanumDrive(){
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
            pinpoint.recalibrateIMU();
        }

//        orientation = imu.getRobotYawPitchRollAngles();
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

    // --------------------------------------- CLAW INTAKE ---------------------------------------
    public void runClawIntake() {
        switch (intakeFSM.getState()) {
            // --------------------------------------- DEFAULT ---------------------------------------
            case DEFAULT:
                if (intakeFSM.justSwitched()) {
                    intakeFSM.timer.reset();
                    intakeArmServoRot = INTAKE_ARM_DEFAULT;
                    intakeFSM.setJustSwitched(false);
                }

                runIntakeTwist();
                if (runIntakeGrabbing().equals("GRAB")) {
                    intakeFSM.setState(IntakeFSM.IntakeState.PICKUP);
                    intakeFSM.setJustSwitched(true);
                }
                break;
            // --------------------------------------- PICKUP ---------------------------------------
            case PICKUP:
                if (intakeFSM.justSwitched()) {
                    intakeFSM.timer.reset();
                    intakeFSM.setJustSwitched(false);
                }

                // Allow some time to let block get picked up
                if (intakeFSM.timer.seconds() > 0.25) {
                    intakeArmServoRot = INTAKE_ARM_DEFAULT;

                    if (useColorSensor) {
                        boolean rejectBlock = (!sampleColorIsAcceptable() || intakeDistanceSensor.getDistance(DistanceUnit.CM) > 1.5);
                        if (rejectBlock) {
                            intakeClawServoRot = INTAKE_CLAW_OPEN;
                            intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT); // Return the claw back to default
                            intakeFSM.setJustSwitched(true);
                            break;
                        }
                    }
                    if (runIntakeGrabbing().equals("RELEASE")) {
                        intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT);
                        intakeFSM.setJustSwitched(true);
                        break;
                    }
                    if (gamepad2.a) {
                        if (gameMode == GameMode.SAMPLE) {
                            intakeFSM.setState(IntakeFSM.IntakeState.SAMPLE_RETRACT);
                        }
                        else if (gameMode == GameMode.SPECIMEN) {
                            intakeFSM.setState(IntakeFSM.IntakeState.SPECIMEN_RETRACT);
                        }
                        intakeFSM.setJustSwitched(true);
                    }
                }
                break;

            // --------------------------------------- SAMPLE RETRACT ---------------------------------------
            case SAMPLE_RETRACT:
                // Initiate the auto retraction of the slide and servos
                if (intakeFSM.justSwitched()) {
                    intakeFSM.timer.reset();
                    pullIntakeBack();
                    intakeFSM.setJustSwitched(false);
                }
                // Retract slide back to magnet sensor to reset encoder, then send it to the transfer position
                if (horizSlideSensor.isPressed()) {
                    intakeFSM.slideWasJustRetracted = true;
                    horizLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runLiftToPosition(horizLinearMotor, TRANSFER_TARGET, 0.5);
                }
                if (!horizLinearMotor.isBusy() && intakeFSM.slideWasJustRetracted) {
                    horizLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (gamepad2.x) {
                        intakeFSM.setState(IntakeFSM.IntakeState.TRANSFER);
                        intakeFSM.setJustSwitched(true);
                    } else if (gamepad2.b) {
                        intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT);
                        intakeFSM.setJustSwitched(true);
                    }
                    intakeFSM.slideWasJustRetracted = false;
                }
                break;
            case TRANSFER:
                if (intakeFSM.justSwitched) {
                    intakeFSM.timer.reset();
                    intakeFSM.justSwitched = false;
                }

            // --------------------------------------- SPECIMEN RETRACT ---------------------------------------
            case SPECIMEN_RETRACT:
                // Initiate the auto retraction of the slide and servos
                if (intakeFSM.justSwitched()) {
                    intakeFSM.timer.reset();
                    pullIntakeBack();
                    intakeFSM.setJustSwitched(false);
                }
                if (horizSlideSensor.isPressed()) {
                    horizLinearPower = 0.0;
                    horizLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT);
                    intakeFSM.setJustSwitched(true);
                }
                break;
            default:
                // should never be reached, as intakeState should never be null
                intakeFSM.setState(IntakeFSM.IntakeState.DEFAULT);
                intakeFSM.justSwitched = true;
        }
    }

    // --------------------------------------- CLAW OUTTAKE ---------------------------------------
    public void runClawOuttake() {
        switch (outtakeFSM.state) {
            case DEFAULT:
                outtakeFSM.timer.reset();
                break;
            case READY_TO_PICKUP:
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

    public void horizontalSlideSystem() {
        // Positive power extends, negative power retracts

        // MAY NEED THIS TO PASS INSPECTION
        // If the outtake claw extends behind the robot, bring slide back
        if (outtakeArmServoRot > 0.5 && outtakeArmServoRot  < 0.75) { //TODO Change servo positions here
            if (!horizSlideSensor.isPressed()) {horizLinearPower = -0.9;}
            else {horizLinearPower = 0.0;}
            return;
        }
        // Dont let driver mess with the slide while auto retracting
        if (intakeFSM.getState() == IntakeFSM.IntakeState.SAMPLE_RETRACT) {
            return;
        }

        // Stop overextension and over retraction of horizontal linear motor
        liftPosHoriz = Math.abs(horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz);
        // if slide sensor touched or manual adjustment button pressed
        if (horizSlideSensor.isPressed() || gamepad2.dpad_left) {
            liftPosAdjHoriz = Math.abs(horizLinearMotor.getCurrentPosition());
        }

        // Prevent smashing into outtake arm
        // TODO Reverse inequality if outtake servo positions get inverted
        if (liftPosHoriz < 800 & outtakeArmServoRot < OUTTAKE_ARM_DEFAULT) {
            outtakeFSM.setState(OuttakeFSM.OuttakeState.DEFAULT);
            outtakeFSM.setJustSwitched(true);
        }
        if (gamepad2.right_stick_y > 0 && !horizSlideSensor.isPressed()) {
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

        // Determine which color the robot sees
        // Color Match Red
        if ((colors.red > minColorIntensity) && (colors.red > colors.green) && (colors.red > colors.blue)) {
            colorMatch = GameColors.RED;
        }
        // Color Match Blue
        else if ((colors.blue > minColorIntensity) && (colors.blue > colors.green) && (colors.blue > colors.red)) {
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
//        hangerMotor.setPower(hangerPower);
    }

    public void runIntakeTwist() {
        if (gamepad2.left_bumper) {
            intakeTwistServoRot = incrementServoRot(intakeTwistServoRot, -0.01, 0.0, 1.0);
        }
        else if (gamepad2.right_bumper) {
            intakeTwistServoRot = incrementServoRot(intakeTwistServoRot, 0.01, 0.0, 1.0);
        }
    }

    public String runIntakeGrabbing() {
        // Allow 0.15 seconds for arm to slam down and grab the block
        if (intakeFSM.clawClosedInitiated && intakeFSM.timer.seconds() > 0.15) {
            intakeClawServoRot = INTAKE_CLAW_CLOSE;
            intakeFSM.clawClosedInitiated = false;
            return "GRAB";
        }
        // If rt pressed swing the arm down and initiate claw closing
        else if (g2RightTriggerPressed && !intakeFSM.clawClosedInitiated) {
            intakeArmServoRot = INTAKE_ARM_DOWN;
            intakeFSM.timer.reset();
            intakeFSM.clawClosedInitiated = true;
            return "NONE";
        }
        // Release the claw
        else if (g2LeftTriggerPressed && intakeClawServoRot == INTAKE_CLAW_CLOSE) {
            intakeClawServoRot = INTAKE_CLAW_OPEN;
            return "RELEASE";
        }
        // Driver isn't pressing triggers
        else {return "NONE";}
    }

    public void pullIntakeBack() {
        intakeTwistServoRot = INTAKE_TWIST_STRAIGHT;
        if (gameMode == GameMode.SAMPLE) {
            intakeArmServoRot = INTAKE_ARM_TRANSFER;
        }
        else if (gameMode == GameMode.SPECIMEN) {
            intakeArmServoRot = INTAKE_ARM_DEFAULT;
        }
        horizLinearPower = -0.9;

        // MOVE THE OUTTAKE SERVOS TO DEFAULT POSITION TO PREVENT SMASHING INTO IT
        // TODO Reverse inequality if outtake servo positions get inverted
        if (outtakeArmServoRot < OUTTAKE_ARM_DEFAULT) {
            outtakeFSM.setState(OuttakeFSM.OuttakeState.DEFAULT);
            outtakeFSM.setJustSwitched(true);
        }
    }

    public void controlRumble() {
        if (!initiatedEndGame && getRuntime() > 90 ) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            initiatedEndGame = true;
        }

        if (getRuntime() > 110) {
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
        }
    }

    public void switchGameMode() {
        if (allPressed && getRuntime() > gameModeCurrent + 1) {
            gameModeCurrent = getRuntime();
            if (gameMode == GameMode.SAMPLE) {
                gameMode = GameMode.SPECIMEN;
            }
            else if (gameMode == GameMode.SPECIMEN) {
                gameMode = GameMode.SAMPLE;
            }
        }
    }
    public double incrementServoRot(double currentRot, double amount, double min, double max) {
        if (max < min) throw new IllegalArgumentException("Min must be less than max");
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
//        telemetry.addData("Gyro: ", "Yaw: " + String.format(Locale.US, "%.2f", orientation.getYaw(AngleUnit.DEGREES))
//                                                + "Roll: " + String.format(Locale.US, "%.2f", orientation.getRoll(AngleUnit.DEGREES))
//                                                + "Pitch: " + String.format(Locale.US, "%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("Heading: ", String.format(Locale.US, "%.2f", pinpoint.getHeading()));
        telemetry.addData("Slowmode: ", finalSlowMode);

        telemetry.addData("Intake Arm Servo", intakeArmServo.getPosition());
        telemetry.addData("Intake Claw Servo", intakeClawServo.getPosition());
        telemetry.addData("Intake Twist Servo", intakeTwistServo.getPosition());

        telemetry.addData("Outtake Arm Servo", outtakeArmServo.getPosition());
        telemetry.addData("Outtake Claw Servo", outtakeClawServo.getPosition());
        telemetry.addData("Outtake Twist Servo", outtakeTwistServo.getPosition());

        telemetry.addData("Intake State:", intakeFSM.getState());
        telemetry.addData("Intake Timer", intakeFSM.timer.time());

        telemetry.addData("Outtake State:", outtakeFSM.getState());
        telemetry.addData("Outtake Timer", outtakeFSM.timer.time());

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Color Match", colorMatch);
        telemetry.addData("Intake Distance", "%.3f", intakeDistanceSensor.getDistance(DistanceUnit.CM));

        telemetry.update();
    }
}

