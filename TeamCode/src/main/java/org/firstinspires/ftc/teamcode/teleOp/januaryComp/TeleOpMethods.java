package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Config
public abstract class TeleOpMethods extends TeleOpHardwareMap {
    @Override
    public void init() {
        super.init();
    // ------------------------------------ FINITE STATE MACHINES ------------------------------------
    }
    // Intake Outtake Transfer Finite State Machine
    class IntakeOuttakeFSM {
        private IntakeOuttakeState state;
        ElapsedTime timer;
        boolean slideWasJustRetracted;
        boolean clawClosedInitiated;
        private boolean justSwitched;

        public enum IntakeOuttakeState {
            DEFAULT,
            PICKUP,
            SAMPLE_RETRACT,
            SPECIMEN_RETRACT,
            READY_TO_TRANSFER,
            TRANSFER,
            LIFT,
            DUMP,
            FLIP_BACK,
            DESCENT,
        }

        public IntakeOuttakeFSM() {
            state = IntakeOuttakeState.DEFAULT;
            timer = new ElapsedTime();
            justSwitched = true;
            slideWasJustRetracted = false;
            clawClosedInitiated = false;
        }

        public IntakeOuttakeState getState () {return state;}
        public void setState (IntakeOuttakeState state) {this.state = state;}
        public boolean justSwitched() {return justSwitched;}
        public void setJustSwitched(boolean b) {justSwitched = b;}
    }
    IntakeOuttakeFSM intakeOuttakeFSM = new IntakeOuttakeFSM();

    // SpecimenFinite State Machine
    class SpecimenFSM {
        private SpecimenState state;
        public ElapsedTime timer;
        public boolean justSwitched;

        public enum SpecimenState {
            READY_TO_GRAB,
            GRAB_AND_FLIP,
            SPECIMEN_HANG
        }

        public SpecimenFSM() {
            state = SpecimenState.READY_TO_GRAB;
            timer = new ElapsedTime();
            justSwitched = true;
        }

        public SpecimenState getState() {return state;}
        public void setState(SpecimenState state) {this.state = state;}
        public boolean justSwitched() {return justSwitched;}
        public void setJustSwitched(boolean b) {justSwitched = b;}
    }
    SpecimenFSM specimenFSM = new SpecimenFSM();

    // ------------------------------------ TELEOP VARIABLES ------------------------------------
    // Game variables
    enum GameMode {
        SAMPLE,
        SPECIMEN
    }
    GameMode gameMode = GameMode.SPECIMEN;

    boolean initiatedEndGame = false;
    boolean allPreviouslyPressed = false;

    // Drive train speeds
    final double driveSpeed = 0.66;
    final double fastSpeed = 1.0;
    final double slowSpeed = 0.25;
    double finalSlowMode = 0.0;

    // TODO Change variables and servo program
    // Intake variables
    final double INTAKE_CLAW_CLOSE = 0.52;
    final double INTAKE_CLAW_OPEN = 1.0;

    final double INTAKE_TWIST_STRAIGHT = 0.5;

    final double INTAKE_ARM_DOWN = 0.9;
    final double INTAKE_ARM_DEFAULT = 0.6;
    final double INTAKE_ARM_TRANSFER = 1.0;

    double intakeArmServoRot = INTAKE_ARM_DEFAULT;
    double intakeClawServoRot = INTAKE_CLAW_OPEN;
    double intakeTwistServoRot = INTAKE_TWIST_STRAIGHT;


    // Outtake variables
    final double OUTTAKE_ARM_BACK = 1.0;
    final double OUTTAKE_ARM_HOOK = 0.4;
    final double OUTTAKE_ARM_DEFAULT = 0.25;
    final double OUTTAKE_ARM_TRANSFER = 0.0;

    final double OUTTAKE_CLAW_CLOSE = 0.0;
    final double OUTTAKE_CLAW_OPEN = 1.0;

    // THE MEANING OF BEARINGS POINTING UP AND DOWN ARE RELATIVE TO THEIR ORIENTATION WHEN THE
    // ARM IS AT THE BACK OF THE ROBOT
    final double OUTTAKE_TWIST_BEARINGS_POINTING_UP = 1.0;
    final double OUTTAKE_TWIST_BEARINGS_POINTING_DOWN = 0.0;

    double outtakeArmServoRot = (gameMode == GameMode.SPECIMEN) ? OUTTAKE_ARM_BACK : OUTTAKE_ARM_DEFAULT;
    double outtakeClawServoRot = OUTTAKE_CLAW_OPEN;
    double outtakeTwistServoRot = OUTTAKE_TWIST_BEARINGS_POINTING_UP;

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
    final int TRANSFER_TARGET = 100;

    // Hanger
    double hangerPower = 0.0;

    // Intake Color Sensor
    final float minColorIntensity = 0.2F;
    NormalizedRGBA colors = new NormalizedRGBA();
    boolean useColorSensor = false;  // Choose whether to let the color sensor make judgements on the block to automatically reject it
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
            pinpoint.resetPosAndIMU();
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

    // --------------------------------------- INTAKE AND OUTTAKE (SAMPLES) ---------------------------------------
    public void runIntakeOuttake() {
        switch (intakeOuttakeFSM.getState()) {
            // --------------------------------------- DEFAULT ---------------------------------------
            case DEFAULT:
                if (intakeOuttakeFSM.justSwitched()) {
                    intakeOuttakeFSM.timer.reset();
                    intakeArmServoRot = INTAKE_ARM_DEFAULT;
                    intakeOuttakeFSM.setJustSwitched(false);
                }

                runIntakeTwist();
                if (runIntakeGrabbing().equals("GRAB")) {
                    intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.PICKUP);
                    intakeOuttakeFSM.setJustSwitched(true);
                }
                break;
            // --------------------------------------- PICKUP ---------------------------------------
            case PICKUP:
                if (intakeOuttakeFSM.justSwitched()) {
                    intakeOuttakeFSM.timer.reset();
                    intakeOuttakeFSM.setJustSwitched(false);
                }

                // Allow some time to let block get picked up
                if (intakeOuttakeFSM.timer.seconds() > 0.25) {
                    intakeArmServoRot = INTAKE_ARM_DEFAULT;

                    if (useColorSensor) {
                        boolean rejectBlock = (!sampleColorIsAcceptable() || intakeDistanceSensor.getDistance(DistanceUnit.CM) > 1.5);
                        if (rejectBlock) {
                            intakeClawServoRot = INTAKE_CLAW_OPEN;
                            intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.DEFAULT); // Return the claw back to default
                            intakeOuttakeFSM.setJustSwitched(true);
                            break;
                        }
                    }
                    if (runIntakeGrabbing().equals("RELEASE")) {
                        intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.DEFAULT);
                        intakeOuttakeFSM.setJustSwitched(true);
                        break;
                    }
                    if (gamepad2.a) {
                        if (gameMode == GameMode.SAMPLE) {
                            intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.SAMPLE_RETRACT);
                        }
                        else if (gameMode == GameMode.SPECIMEN) {
                            intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.SPECIMEN_RETRACT);
                        }
                        intakeOuttakeFSM.setJustSwitched(true);
                    }
                }
                break;

            // --------------------------------------- SAMPLE RETRACT ---------------------------------------
            case SAMPLE_RETRACT:
                // Initiate the auto retraction of the slide and servos
                if (intakeOuttakeFSM.justSwitched()) {
                    intakeOuttakeFSM.timer.reset();
                    pullIntakeBack();
                    intakeOuttakeFSM.setJustSwitched(false);
                }
                break;
                // TODO
            case TRANSFER:
                if (intakeOuttakeFSM.justSwitched) {
                    intakeOuttakeFSM.timer.reset();
                    intakeOuttakeFSM.justSwitched = false;
                }

            // --------------------------------------- SPECIMEN RETRACT ---------------------------------------
            case SPECIMEN_RETRACT:
                // Initiate the auto retraction of the slide and servos
                if (intakeOuttakeFSM.justSwitched()) {
                    intakeOuttakeFSM.timer.reset();
                    pullIntakeBack();
                    intakeOuttakeFSM.setJustSwitched(false);
                }
                if (horizSlideSensor.isPressed()) {
                    intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.DEFAULT);
                    intakeOuttakeFSM.setJustSwitched(true);
                }
                break;
            default:
                // should never be reached, as intakeState should never be null
                intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.DEFAULT);
                intakeOuttakeFSM.justSwitched = true;
        }
    }

    // --------------------------------------- SPECIMENS ---------------------------------------
    public void runSpecimens() {
        // Specimen cycling ONLY runs in specimen gamemode on the controller
        if (gameMode == GameMode.SPECIMEN) return;
        switch (specimenFSM.getState()) {
            case READY_TO_GRAB:
                if (specimenFSM.justSwitched()) {
                    specimenFSM.timer.reset();
                    outtakeArmServoRot = OUTTAKE_ARM_BACK;
                    outtakeClawServoRot = OUTTAKE_CLAW_OPEN;
                    outtakeTwistServoRot = OUTTAKE_TWIST_BEARINGS_POINTING_UP;
                    specimenFSM.setJustSwitched(false);
                }
                if (gamepad2.x) {
                    specimenFSM.setState(SpecimenFSM.SpecimenState.GRAB_AND_FLIP);
                    specimenFSM.setJustSwitched(true);
                }
                break;
            case GRAB_AND_FLIP:
                if (specimenFSM.justSwitched()) {
                    specimenFSM.timer.reset();
                    outtakeClawServoRot = OUTTAKE_CLAW_CLOSE;
                    specimenFSM.setJustSwitched(false);
                }
                else if (specimenFSM.timer.seconds() > 0.10) {
                    outtakeArmServoRot = OUTTAKE_ARM_HOOK;
                    outtakeTwistServoRot = OUTTAKE_TWIST_BEARINGS_POINTING_DOWN;
                }
                break;
            case SPECIMEN_HANG:
                if (specimenFSM.justSwitched()) {
                    specimenFSM.timer.reset();
                    specimenFSM.setJustSwitched(false);
                }
                break;
            default:
                // should never be reached, as specimenState should never be null
                specimenFSM.setState(SpecimenFSM.SpecimenState.READY_TO_GRAB);
                specimenFSM.justSwitched = true;
        }
    }

    public void horizontalSlideSystem() {
        // Positive power extends, negative power retracts

        // MAY NEED THIS TO PASS INSPECTION
        // If the outtake claw extends behind the robot, bring slide back
        if (outtakeArmServoRot > 0.5 && outtakeArmServoRot  < 0.75 && liftPosHoriz > 1500) { //TODO Change servo positions here
            if (!horizSlideSensor.isPressed()) {horizLinearPower = -0.9;}
            else {horizLinearPower = 0.0;}
            return;
        }

        liftPosHoriz = Math.abs(horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz);
        // if slide sensor touched or manual adjustment button pressed
        if (horizSlideSensor.isPressed() || gamepad2.dpad_left) {
            liftPosAdjHoriz = Math.abs(horizLinearMotor.getCurrentPosition());
        }

        // TODO Prevent smashing into outtake claw

        if (intakeOuttakeFSM.getState() == IntakeOuttakeFSM.IntakeOuttakeState.SAMPLE_RETRACT
        || intakeOuttakeFSM.getState() == IntakeOuttakeFSM.IntakeOuttakeState.SPECIMEN_RETRACT) {
            return;
        }
        // Stop overextension and over retraction of horizontal linear motor
        if (gamepad2.right_stick_y > 0 && !horizSlideSensor.isPressed()) {
            horizLinearPower = gamepad2.right_stick_y * 0.5;
        } else if (gamepad2.right_stick_y < 0.0 && liftPosHoriz < HORIZ_MAX) {
            horizLinearPower = gamepad2.right_stick_y * 0.5;
        } else { horizLinearPower = 0.0;}
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
            intakeTwistServoRot = incrementServoRot(intakeTwistServoRot, -0.03, 0.0, 1.0);
        }
        else if (gamepad2.right_bumper) {
            intakeTwistServoRot = incrementServoRot(intakeTwistServoRot, 0.03, 0.0, 1.0);
        }
    }

    public String runIntakeGrabbing() {
        boolean g2RightTriggerPressed = gamepad2.right_trigger > 0.4;
        boolean g2LeftTriggerPressed = gamepad2.left_trigger > 0.4;
        // Allow 0.15 seconds for arm to slam down and grab the block
        if (intakeOuttakeFSM.clawClosedInitiated && intakeOuttakeFSM.timer.seconds() > 0.15) {
            intakeClawServoRot = INTAKE_CLAW_CLOSE;
            intakeOuttakeFSM.clawClosedInitiated = false;
            return "GRAB";
        }
        // If rt pressed swing the arm down and initiate claw closing
        else if (g2RightTriggerPressed && !intakeOuttakeFSM.clawClosedInitiated) {
            intakeArmServoRot = INTAKE_ARM_DOWN;
            intakeOuttakeFSM.timer.reset();
            intakeOuttakeFSM.clawClosedInitiated = true;
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

        // TODO Prevent smashing into the outtake claw
    }

    public void endGame() {
        if (!initiatedEndGame && getRuntime() > 90 ) {
            rumbleGamePads(1_000, 0.5);
            initiatedEndGame = true;
        }

        if (getRuntime() > 110) {
            rumbleGamePads(10_000, 0.5);
        }
    }

    public void switchGameMode() {
        // require full press of triggers to initiate
        boolean g2RightTriggerPressed = gamepad2.right_trigger >= 0.99;
        boolean g2LeftTriggerPressed = gamepad2.right_trigger >= 0.99;
        boolean allPressed = (g2RightTriggerPressed && g2LeftTriggerPressed && gamepad2.left_bumper && gamepad2.right_bumper);

        // Check if all buttons are pressed and this is a new state
        if (allPressed && !allPreviouslyPressed) {
            allPreviouslyPressed = true; // Mark the state as handled

            // Toggle game mode and provide feedback
            if (gameMode == GameMode.SAMPLE) {
                gameMode = GameMode.SPECIMEN;
                specimenFSM.setState(SpecimenFSM.SpecimenState.READY_TO_GRAB);
                intakeOuttakeFSM.setState(IntakeOuttakeFSM.IntakeOuttakeState.DEFAULT);
                specimenFSM.setJustSwitched(true);
                intakeOuttakeFSM.setJustSwitched(true);
                rumbleGamePads(1500, 0.5); // Longer, softer rumble
            } else if (gameMode == GameMode.SPECIMEN) {
                gameMode = GameMode.SAMPLE;
                rumbleGamePads(500, 1.0); // Shorter, stronger rumble
            }
        }

        // Reset the state when buttons are released
        if (!allPressed && allPreviouslyPressed) {
            allPreviouslyPressed = false;
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

    public void rumbleGamePads(int ms, double power) {
        gamepad1.rumble(power, power, ms);
        gamepad2.rumble(power, power, ms);
    }

    public void addTelemetryToDriverStation() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Heading: ", String.format(Locale.US, "%.2f", Math.toDegrees(pinpoint.getHeading())));
        telemetry.addData("Slowmode: ", finalSlowMode);
        telemetry.addData("Gamemode:", gameMode);


        telemetry.addData("Intake Arm Servo", intakeArmServo.getPosition());
        telemetry.addData("Intake Claw Servo", intakeClawServo.getPosition());
        telemetry.addData("Intake Twist Servo", intakeTwistServo.getPosition());

        telemetry.addData("Outtake Arm Servo", outtakeArmServo.getPosition());
        telemetry.addData("Outtake Claw Servo", outtakeClawServo.getPosition());
        telemetry.addData("Outtake Twist Servo", outtakeTwistServo.getPosition());

        telemetry.addData("Intake State:", intakeOuttakeFSM.getState());
        telemetry.addData("Intake Timer", intakeOuttakeFSM.timer.time());

        telemetry.addData("Specimen State:", specimenFSM.getState());
        telemetry.addData("Outtake Timer", specimenFSM.timer.time());

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Color Match", colorMatch);
        telemetry.addData("Intake Distance", "%.3f", intakeDistanceSensor.getDistance(DistanceUnit.CM));

        telemetry.update();
    }
}

