package org.firstinspires.ftc.teamcode.autonomous.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public  class ParallelActionsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        TestMotor motor = new TestMotor(hardwareMap);
        TestServo servo = new TestServo(hardwareMap);

        if (opModeIsActive()) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(
                            motor.drive(),
                            new SequentialAction(
                                servo.open(),
                                servo.close(),
                                servo.open()
                            )
                    )
            );
        }
    }

    // TEST MOTOR
    public class TestMotor {
        private final DcMotorEx motor;

        public TestMotor(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "motor");
        }

        public class Drive implements Action {
            private boolean initialized = false;
            private final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setTargetPosition(2000);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    initialized = true;
                }
                if (timer.time() < 5 && !isStopRequested()) {
                    motor.setPower(0.5);
                    return true;
                } else {
                    motor.setPower(0);
                    return false;
                }
            }
        }
        public Action drive() {
            return new Drive();
        }
    }

    // TEST SERVO
    public class TestServo {
        private final Servo servo;

        public TestServo(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "servo");
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(0.0);
                return false;
            }
        }
        public Action open() {
            return new Open();
        }
        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo.setPosition(1.0);
                return false;
            }
        }
        public Action close() {
            return new Close();
        }
    }
}
