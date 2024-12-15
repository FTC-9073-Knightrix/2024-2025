package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Config
public abstract class FiveSpecimenActions extends LinearOpMode {
    final public int liftUpHeight = -2200;
    final public int liftOffWallHeight = -500;
    final public int hookHeight = -2300;
    final public double clawOpenPosition = 0.55; // TODO CHANGE VALUES ACCORDINGLY
    final public double clawClosePosition = 0.2;

    final public double clawArmForwardPosition = 0.0;
    final public double clawArmBackPosition = 1.0;

    public TouchSensor liftSensor = hardwareMap.touchSensor.get("liftSensor");

    // --------------------------------- VERT LIFT ----------------------------------
    public class VertLift {
        private final DcMotorEx liftMotor;

        public VertLift(HardwareMap hardwareMap) {
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftMotor.setTargetPosition(liftUpHeight);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (liftMotor.isBusy() && !isStopRequested()) {
                    return true;
                } else {
                    liftMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUpToChamber() {
            return new LiftUp();
        }

        public class HookOnBar implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftMotor.setTargetPosition(hookHeight);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (liftMotor.isBusy() && !isStopRequested()) {
                    return true;
                } else {
                    liftMotor.setPower(0);
                    return false;
                }
            }

        }
        public Action hookOnBar() {
            return new HookOnBar();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftMotor.setPower(-0.8);
                    initialized = true;
                }
                packet.put("liftPos", liftMotor.getCurrentPosition());
                if (!liftSensor.isPressed() && !isStopRequested()) {
                    return true;
                } else {
                    liftMotor.setPower(0);
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftOffWall implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftMotor.setPower(-0.8);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (liftMotor.isBusy() && !isStopRequested()) {
                    return true;
                } else {
                    liftMotor.setPower(0);
                    return false;
                }
            }
        }
        public Action liftOffWall() {
            return new LiftOffWall();
        }
    }


    // ----------------------------------- CLAW -----------------------------------
    public class Claw {
        private final Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }

        public void setPos(double pos) {
            clawServo.setPosition(Range.clip(pos, 0, 1));
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(clawOpenPosition);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(clawClosePosition);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
    }


    // --------------------------------- CLAW ARM ---------------------------------
    public class ClawArm {
        final private Servo clawArmServo;

        public ClawArm(HardwareMap hardwareMap) {
            clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");
            clawArmServo.setPosition(clawArmForwardPosition);
        }

        public void setPos(double pos) {
            clawArmServo.setPosition(Range.clip(pos, 0.0, 1.0));
        }

        public class ClawArmForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawArmServo.setPosition(clawArmForwardPosition);
                return false;
            }
        }
        public Action clawArmForward() {
            return new ClawArmForward();
        }

        public class ClawArmBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawArmServo.setPosition(clawArmBackPosition);
                return false;
            }
        }
        public Action clawArmBack() {
            return new ClawArmBack();
        }
    }
}
