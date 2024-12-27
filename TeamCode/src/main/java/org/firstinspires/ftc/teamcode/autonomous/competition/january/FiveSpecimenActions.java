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

    public static double heading = 0;

    public TouchSensor liftSensor = hardwareMap.touchSensor.get("liftSensor");

//    public int cioqlnwqduhqw

    // --------------------------------- VERT LIFT ----------------------------------
    public class VertLinearMotor {
        private final DcMotorEx vertLinearMotor;

        public VertLinearMotor(HardwareMap hardwareMap) {
            vertLinearMotor = hardwareMap.get(DcMotorEx.class, "vertLinearMotor");
            vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    vertLinearMotor.setTargetPosition(liftUpHeight);
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (vertLinearMotor.isBusy() && !isStopRequested()) {
//                    vertLinearMotor.setPower();
                    return true;
                } else {
                    vertLinearMotor.setPower(0);
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
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    vertLinearMotor.setTargetPosition(hookHeight);
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (vertLinearMotor.isBusy() && !isStopRequested()) {
                    return true;
                } else {
                    vertLinearMotor.setPower(0);
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
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    vertLinearMotor.setPower(-0.8);
                    initialized = true;
                }
                packet.put("liftPos", vertLinearMotor.getCurrentPosition());
                if (!liftSensor.isPressed() && !isStopRequested()) {
                    return true;
                } else {
                    vertLinearMotor.setPower(0);
                    vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    vertLinearMotor.setPower(-0.8);
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (vertLinearMotor.isBusy() && !isStopRequested()) {
                    return true;
                } else {
                    vertLinearMotor.setPower(0);
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
