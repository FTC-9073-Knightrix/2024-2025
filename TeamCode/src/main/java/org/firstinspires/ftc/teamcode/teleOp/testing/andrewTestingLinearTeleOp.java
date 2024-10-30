package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="andrewTestingLinearTeleOp")
public class andrewTestingLinearTeleOp extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization code goes here
        telemetry.addData("Initialization:", "Successful");
        telemetry.update();

        waitForStart(); // Waits for the start button to be pressed

        runtime.reset(); // start counting runtime now

        while (opModeIsActive()) {
            telemetry.addData("Elapsed Time:", runtime.toString());
            idle();
        }
    }
}
