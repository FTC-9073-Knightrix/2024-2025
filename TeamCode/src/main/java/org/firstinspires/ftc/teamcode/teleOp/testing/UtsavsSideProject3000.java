package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Utsav's Advanced TeleOp Control")
public class UtsavsSideProject3000 extends OpMode {
    public DcMotor intake;
    public static final double NORMAL_SPEED = 0.7;
    public static final double BOOST_SPEED = 1.0;
    public static final double SLOW_SPEED = 0.4;
    private double currentSpeed = NORMAL_SPEED;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Speed Mode Control
        if (gamepad1.left_bumper) {
            currentSpeed = SLOW_SPEED;
        } else if (gamepad1.right_bumper) {
            currentSpeed = BOOST_SPEED;
        } else {
            currentSpeed = NORMAL_SPEED;
        }

        // Intake Power Control
        double intakePower = (gamepad1.left_trigger - gamepad1.right_trigger) * currentSpeed;
        intake.setPower(intakePower);

        // Directional Control
        double driveX = gamepad1.left_stick_x * currentSpeed;
        double driveY = -gamepad1.left_stick_y * currentSpeed; // inverted Y-axis for intuitive control
        double rotation = gamepad1.right_stick_x * currentSpeed;

        // Telemetry for Debugging and Monitoring
        telemetry.addData("Intake Power:", intakePower);
        telemetry.addData("Drive X:", driveX);
        telemetry.addData("Drive Y:", driveY);
        telemetry.addData("Rotation:", rotation);
        telemetry.addData("Current Speed Mode:", currentSpeed == BOOST_SPEED ? "Boost" : (currentSpeed == SLOW_SPEED ? "Slow" : "Normal"));
        telemetry.update();
    }
}
