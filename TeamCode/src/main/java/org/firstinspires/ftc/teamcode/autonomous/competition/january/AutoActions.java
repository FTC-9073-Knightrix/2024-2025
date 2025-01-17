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
public abstract class AutoActions extends LinearOpMode {
    final public int liftUpHeight = -725;
//    final public int liftOffWallHeight = -500;
//    final public int hookHeight = -2300;
    final public double clawOpenPosition = 0.66; // TODO CHANGE VALUES ACCORDINGLY
    final public double clawClosePosition = 0.25;

    final public double clawArmStartPosition = 0.06;
    final public double clawArmForwardPosition = 0.275;
    final public double clawArmBackPosition = 1.0;

    final double clawTwistBearingUp = 1.0;
    final double clawTwistBearingDown = 0.0;

    public static double heading = 0;



    // --------------------------------- VERT LIFT ----------------------------------
    public class VertLinearMotor {
        private final DcMotorEx vertLinearMotor;
        private final TouchSensor vertSlideSensor;

        public VertLinearMotor(HardwareMap hardwareMap) {
            vertLinearMotor = hardwareMap.get(DcMotorEx.class, "vertLinearMotor");
            vertSlideSensor = hardwareMap.get(TouchSensor.class, "vertSlideSensor");
            vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class LiftUpToChamber implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vertLinearMotor.setTargetPosition(liftUpHeight);
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initialized = true;
                }
                if (vertLinearMotor.isBusy() && !isStopRequested()) {
                    vertLinearMotor.setPower(-0.7);
                    return true;
                } else {
                    vertLinearMotor.setPower(-0.12);
                    return false;
                }
            }
        }
        public Action liftUpToChamber() {
            return new LiftUpToChamber();
        }

        public class HookOnBar implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    initialized = true;
                }
                if (!isStopRequested() && !vertSlideSensor.isPressed()) {
                    vertLinearMotor.setPower(1.0);
                    return true;
                } else {
                    vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    vertLinearMotor.setPower(0);
                    return false;
                }
            }

        }
        public Action hookOnBar() {
            return new HookOnBar();
        }
    }


    // ----------------------------------- CLAW -----------------------------------
    public class OuttakeClawServo {
        private final Servo outtakeClawServo;

        public OuttakeClawServo(HardwareMap hardwareMap) {
            outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
            outtakeClawServo.setPosition(clawClosePosition);
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(clawOpenPosition);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(clawClosePosition);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
    }


    // --------------------------------- CLAW ARM ---------------------------------
    public class OuttakeArmServo {

        final private Servo outtakeArmServo;

        public OuttakeArmServo(HardwareMap hardwareMap) {
            outtakeArmServo = hardwareMap.get(Servo.class, "outtakeArmServo");
            outtakeArmServo.setPosition(clawArmStartPosition);
        }

        public void setPos(double pos) {outtakeArmServo.setPosition(Range.clip(pos, 0.0, 1.0));}

        public class ClawArmStart implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeArmServo.setPosition(clawArmStartPosition);
                return false;
            }
        }
        public Action clawArmStart() {
            return new ClawArmStart();
        }

        public class ClawArmForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeArmServo.setPosition(clawArmForwardPosition);
                return false;
            }
        }
        public Action clawArmForward() {
            return new ClawArmForward();
        }

        public class ClawArmBack implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeArmServo.setPosition(clawArmBackPosition);
                return false;
            }
        }
        public Action clawArmBack() {
            return new ClawArmBack();
        }
    }
    // --------------------------------- TWIST ---------------------------------
    public class OuttakeTwistServo {
        final private Servo outtakeTwistServo;

        public OuttakeTwistServo(HardwareMap hardwareMap) {
            outtakeTwistServo = hardwareMap.get(Servo.class, "outtakeTwistServo");
            outtakeTwistServo.setPosition(clawTwistBearingDown);
        }

        public void setPos(double pos) {outtakeTwistServo.setPosition(Range.clip(pos, 0.0, 1.0));}

        public class TwistToBearingsUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeTwistServo.setPosition(clawTwistBearingUp);
                return false;
            }
        }
        public Action twistToBearingsUp() {
            return new TwistToBearingsUp();
        }

        public class TwistToBearingsDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeTwistServo.setPosition(clawTwistBearingDown);
                return false;
            }
        }
        public Action twistToBearingsDown() {return new TwistToBearingsDown();}
    }
}
