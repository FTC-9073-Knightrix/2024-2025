package org.firstinspires.ftc.teamcode.teleOp.testing.onRobotTesting

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.teleOp.januaryComp.TeleOpHardwareMap

@TeleOp(name =  "C-HUB & EX-HUB Port Checker")
class PortCheck: TeleOpHardwareMap() {
    override fun loop() {
        // Display the port of every hardware device.
        telemetry.addLine("Port information:\n")
        telemetry.addData("vertLinearMotor", vertLinearMotor.portNumber)
        telemetry.addData("horizLinearMotor", horizLinearMotor.portNumber)
        telemetry.addData("intakeClawServo", intakeClawServo.portNumber)
        telemetry.addData("intakeTwistServo", intakeTwistServo.portNumber)
        telemetry.addData("intakeArmServo", intakeArmServo.portNumber)
        telemetry.addData("outtakeClawServo", outtakeClawServo.portNumber)
        telemetry.addData("outtakeArmServo", outtakeArmServo.portNumber)
        telemetry.addData("outtakeTwistServo", outtakeTwistServo.portNumber)
        telemetry.addData("vertSlideSensor", vertSlideSensor.connectionInfo)
        telemetry.addData("horizSlideSensor", horizSlideSensor.connectionInfo)
        telemetry.addData("colorSensor", colorSensor.connectionInfo)
        telemetry.addData("intakeDistanceSensor", intakeDistanceSensor.connectionInfo)
        telemetry.update()
    }

}