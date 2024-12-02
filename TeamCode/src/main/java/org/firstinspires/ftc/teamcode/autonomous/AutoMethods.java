package org.firstinspires.ftc.teamcode.autonomous;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;

public abstract class AutoMethods extends LinearOpMode {
    public class LatchServo {
        private Servo latchServo;

        public LatchServo(HardwareMap hardwareMap) {
            latchServo = hardwareMap.get(Servo.class, "latchServo");
        }
        public class LatchOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                latchServo.setPosition(1);
                return false;
            }
        }
        public Action openLatch() {
            return new LatchOpen();
        }

        public class LatchClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                latchServo.setPosition(0);
                return false;
            }
        }
        public Action closeLatch() {
            return new LatchClose();
        }
    }

    public class ArmServo {
        private Servo armServo;

        public ArmServo(HardwareMap hardwareMap) {
            armServo = hardwareMap.get(Servo.class, "armServo");
        }
        public class ArmOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(1);
                return false;
            }
        }
        public Action openArm() {
            return new ArmOpen();
        }

        public class ArmClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(0);
                return false;
            }
        }
        public Action closeArm() {
            return new ArmClose();
        }
    }

    public class ClawServo {
        private Servo clawServo;

        public ClawServo(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
        }
        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(1);
                return false;
            }
        }
        public Action openClaw() {
            return new ClawOpen();
        }

        public class ClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0);
                return false;
            }
        }
        public Action closeClaw() {
            return new ClawClose();
        }
    }

    public class BasketServo {
        private Servo basketServo;

        public BasketServo(HardwareMap hardwareMap) {
            basketServo = hardwareMap.get(Servo.class, "basketServo");
        }
        public class BasketOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basketServo.setPosition(1);
                return false;
            }
        }
        public Action openBasket() {
            return new BasketOpen();
        }

        public class BasketClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                basketServo.setPosition(0);
                return false;
            }
        }
        public Action closeBasket() {
            return new BasketClose();
        }
    }

    public class VertLift {
        private DcMotorEx vertLift;

        public VertLift(HardwareMap hardwareMap) {
            vertLift = hardwareMap.get(DcMotorEx.class, "vertLinearMotor");
            vertLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertLift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class VertLiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vertLift.setPower(1);
                    initialized = true;
                }

                double pos = vertLift.getCurrentPosition();
                telemetry.addData("Vertical Position: ", pos);
                if (pos < 3000.0) { //TODO Find true encoder position
                    return true;
                } else {
                    vertLift.setPower(0);
                    return false;
                }
            }
        }
        public Action vertLiftUp() {
            return new VertLiftUp();
        }

        public class VertLiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vertLift.setPower(-1);
                    initialized = true;
                }

                double pos = vertLift.getCurrentPosition();
                telemetry.addData("Vertical Position: ", pos);
                if (!vertSlideLimit.isPressed()) {
                    return true;
                } else {
                    vertLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    vertLift.setPower(0);
                    return false;
                }
            }
        }
        public Action vertLiftDown(){
            return new VertLiftDown();
        }
    }

    public class HorizLinearMotor {
        private DcMotorEx horizLinearMotor;

        public HorizLinearMotor(HardwareMap hardwareMap) {
            horizLinearMotor = hardwareMap.get(DcMotorEx.class, "horizLinearMotor");
            horizLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            horizLinearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }


    public class IntakeMotor {
        private DcMotorEx intakeMotor;

        public IntakeMotor(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust if needed
        }


        public class IntakeIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(1);
                    initialized = true;
                }

                telemetry.addData("Intake Motor", "Running Forward");
                return false;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        // Action to outtake (move the motor backward)
        public class IntakeOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(-1);
                    initialized = true;
                }

                telemetry.addData("Intake Motor", "Running Backward");
                return false;
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public class IntakeStop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeMotor.setPower(0);
                    initialized = true;
                }

                telemetry.addData("Intake Motor", "Stopped");
                return true;
            }
        }

        public Action intakeStop() {
            return new IntakeStop();
        }
    }

    TouchSensor vertSlideLimit;
    TouchSensor horizSlideLimit;

    MecanumDrive mecanumDrive;

    public IMU imu;
    public ElapsedTime runtime;

    public void initRobot() {
        telemetry.addData("Initialization", "Starting...");
        telemetry.addData("Initialization", "Done!");

        vertSlideLimit = hardwareMap.touchSensor.get("vertSlideSensor");
        horizSlideLimit = hardwareMap.touchSensor.get("horizSlideSensor");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}