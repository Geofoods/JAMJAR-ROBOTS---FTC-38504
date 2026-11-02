package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Reads gamepad1 and prints, live, which buttons are pressed, the D-pad direction,
 * the trigger values, and the X/Y position of both joysticks.
 *
 * Note: on an FTC gamepad, pushing a stick UP gives a NEGATIVE y value.
 *
 * Select "Controller Telemetry" on the Driver Station, press INIT, then START.
 */
@TeleOp(name = "Controller Telemetry", group = "Examples")
public class
ControllerTelemetry extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Controller Telemetry");
        telemetry.addLine("Use gamepad1, then press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Gamepad pad = gamepad1;

            telemetry.addLine("=== gamepad1 ===");
            telemetry.addData("Buttons", describeButtons(pad));
            telemetry.addData("D-pad", describeDpad(pad));
            telemetry.addData("Left stick", "x=%.3f  y=%.3f", pad.left_stick_x, pad.left_stick_y);
            telemetry.addData("Right stick", "x=%.3f  y=%.3f", pad.right_stick_x, pad.right_stick_y);
            telemetry.addData("Triggers", "left=%.3f  right=%.3f", pad.left_trigger, pad.right_trigger);
            telemetry.update();
        }
    }

    /** Returns a comma-separated list of the face/shoulder/system buttons currently held. */
    private String describeButtons(Gamepad pad) {
        StringBuilder sb = new StringBuilder();
        appendIfPressed(sb, pad.a, "A");
        appendIfPressed(sb, pad.b, "B");
        appendIfPressed(sb, pad.x, "X");
        appendIfPressed(sb, pad.y, "Y");
        appendIfPressed(sb, pad.left_bumper, "LB");
        appendIfPressed(sb, pad.right_bumper, "RB");
        appendIfPressed(sb, pad.left_stick_button, "L3");
        appendIfPressed(sb, pad.right_stick_button, "R3");
        appendIfPressed(sb, pad.back, "Back");
        appendIfPressed(sb, pad.start, "Start");
        appendIfPressed(sb, pad.guide, "Guide");
        return sb.length() == 0 ? "(none)" : sb.toString();
    }

    /** Returns the D-pad direction(s) currently held, or "(none)". */
    private String describeDpad(Gamepad pad) {
        StringBuilder sb = new StringBuilder();
        appendIfPressed(sb, pad.dpad_up, "Up");
        appendIfPressed(sb, pad.dpad_down, "Down");
        appendIfPressed(sb, pad.dpad_left, "Left");
        appendIfPressed(sb, pad.dpad_right, "Right");
        return sb.length() == 0 ? "(none)" : sb.toString();
    }

    private void appendIfPressed(StringBuilder sb, boolean pressed, String name) {
        if (pressed) {
            if (sb.length() > 0) {
                sb.append(", ");
            }
            sb.append(name);
        }
    }
}
