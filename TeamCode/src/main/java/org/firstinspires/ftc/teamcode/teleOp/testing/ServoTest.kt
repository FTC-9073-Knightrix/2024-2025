import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range

@TeleOp(name="Servo Test")
class ServoTest : OpMode() {
    lateinit var servo: Servo
    var servoRot: Double = 0.0;
    override fun init() {
        servo = hardwareMap.get(Servo::class.java, "servo")
    }

    override fun loop() {
        if (gamepad1.right_bumper) {
            incrementServoRot(servoRot, 0.05);
        }
        if (gamepad1.left_bumper) {
            incrementServoRot(servoRot, -0.05);
        }

        if (gamepad1.x) servoRot = 0.0
        if (gamepad1.y) servoRot = 0.5
        if (gamepad1.b) servoRot = 1.0

        servo.position = servoRot
        telemetryInstructions()
    }

    fun incrementServoRot(currentRot: Double, amount: Double): Double {
        return (Range.clip(currentRot, 0.0, 1.0) + amount)
    }

    fun telemetryInstructions(): Unit {
        telemetry.addLine("G1.Right Bumper: Increase Servo Position")
        telemetry.addLine("G1.Left Bumper: Decrease Servo Position")

        telemetry.addLine("G1.X: Set Servo Position to 0")
        telemetry.addLine("G1.Y: Set Servo Position to 0.5")
        telemetry.addLine("G1.B: Set Servo Position to 1")

        telemetry.addData("Servo Position", servoRot)
        telemetry.update()
    }
}