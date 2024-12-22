package org.firstinspires.ftc.teamcode.teleOp.testing;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.januaryComp.TeleOpMethods;

import java.util.Arrays;
import java.util.Collections;
import java.util.Locale;

@TeleOp
public class ColorSensorTest extends OpMode {

    private Servo servo;
    private NormalizedColorSensor color;
    private DistanceSensor distance;

    boolean clawClosed = false;
    float gain = 22;
    NormalizedRGBA colors;

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];
    enum GameColors {RED, BLUE, NONE}
    GameColors colorMatch = GameColors.NONE;
    GameColors allianceColor = GameColors.RED; // MUST be initialized in Op Mode as only RED or BLUE
    double current1 = Double.MAX_VALUE;
    @Override
    public void init() {
        // Initialize the hardware components
        servo = hardwareMap.get(Servo.class, "servo");
        color = hardwareMap.get(NormalizedColorSensor.class, "color_distance"); // use same i2c config for both
        distance = hardwareMap.get(DistanceSensor.class, "color_distance"); // use same i2c config
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {
        data();
        getColors();
        setAllianceColor();
        if (gamepad1.right_trigger >= 0.75 && !clawClosed) {
            clawClosed = true;
            servo.setPosition(0.55);
            current1 = getRuntime();
        }
        if (clawClosed) {
            if ((!sampleColorIsAcceptable() || distance.getDistance(DistanceUnit.CM) > 1) && getRuntime() > current1 + 0.25) {
                servo.setPosition(1.0);
                current1 = Double.MAX_VALUE;
                clawClosed = false;
            }
        }

        if (gamepad1.left_trigger >= 0.75) {
            servo.setPosition(1.0);
            clawClosed = false;
        }
        telemetry.update();
    }

    public void setAllianceColor() {
        if (gamepad1.right_bumper) {
            allianceColor = GameColors.RED;
        } else if (gamepad1.left_bumper) {
            allianceColor = GameColors.BLUE;
        }


        // Telemetry to tell user instructions for how to change alliance color
        telemetry.addLine("Press RB to set alliance color to RED");
        telemetry.addLine("Press LB to set alliance color to BLUE");
        telemetry.addData("Alliance Color", allianceColor);

    }
    public void data() {
        // Update the gain value if either of the A or B gamepad buttons is being held
        if (gamepad1.a) {
            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
            gain += 0.005F;
        } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
            gain -= 0.005F;
        }

        // Show the gain value via telemetry
        telemetry.addData("Gain:", gain);
        color.setGain(gain);


        telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Get the normalized colors from the sensor

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */
        // Update the hsvValues array by passing it to Color.colorToHSV()
//        Color.colorToHSV(colors.toColor(), hsvValues);
        colors = color.getNormalizedColors();

        // Output the sensor data to telemetry
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.addData("Color Match", colorMatch + "\n");
        if (sampleColorIsAcceptable()) {
            telemetry.addLine("Sample is acceptable with " + allianceColor.name() + " alliance color");
        } else {
            telemetry.addLine("Sample is NOT acceptable with " + allianceColor.name() + " alliance color");
        }
        telemetry.addData("Distance", "%.3f", distance.getDistance(DistanceUnit.CM));
        telemetry.addData("Current", current1);
    }

    public boolean sampleColorIsAcceptable() {
        boolean colorIsNotAnAllianceColor = (colorMatch != GameColors.RED && colorMatch != GameColors.BLUE);
        boolean colorMatchesAllianceColor = (colorMatch == allianceColor);
        return (colorIsNotAnAllianceColor || colorMatchesAllianceColor);
    }

    public void getColors(){
//        float[] colorsInOrder = {colors.red, colors.green, colors.blue};
//        Arrays.sort(colorsInOrder);
//        // Reverse the array
//        for (int i = 0; i < colorsInOrder.length / 2; i++) {
//            float temp = colorsInOrder[i];
//            colorsInOrder[i] = colorsInOrder[colorsInOrder.length - 1 - i];
//            colorsInOrder[colorsInOrder.length - 1 - i] = temp;
//        }

        if ((colors.red > 0.2 ) && (colors.red > colors.green) && (colors.red > colors.blue)) {
            colorMatch = GameColors.RED;
        }
        else if ((colors.blue > 0.2 ) && (colors.blue > colors.green) && (colors.blue > colors.red)) {
            colorMatch = GameColors.BLUE;
        }
        else {colorMatch = GameColors.NONE;}
    }
}
