package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Sweeps a Studica Multi-Mode Smart Servo back and forth through 180 degrees.
 *
 *   One second toward POSITION_B, one second back toward POSITION_A, repeat.
 *
 * The servo must be named "servo" in the Driver Station robot configuration
 * (servo port 0).
 *
 * About the positions: the Multi-Mode Smart Servo in its default "standard"
 * mode has a 300-degree range (150 degrees each side of center) and accepts
 * pulse widths from 500 to 2500 microseconds. The code tells the hub to use
 * that full 500-2500 range, so setPosition() maps directly onto the 300
 * degrees:
 *
 *   position 0.0 = 500 us = 0 degrees
 *   position 0.5 = 1500 us = center
 *   position 1.0 = 2500 us = 300 degrees
 *
 * A 180-degree sweep centered on the servo is therefore 0.2 to 0.8:
 *   position 0.2 = 900 us = 60 degrees
 *   position 0.8 = 2100 us = 240 degrees
 *
 * The servo moves as fast as it can (about 0.2 s per 60 degrees), so the
 * one-second waits just give it time to reach each end. If your servo turns
 * the wrong way first, swap POSITION_A and POSITION_B.
 *
 * If you reprogrammed the servo to a custom 180-degree range with the Studica
 * programmer, change the positions back to 0.0 and 1.0.
 */
@TeleOp(name = "Servo Sweep", group = "Examples")
public class ServoSweep extends LinearOpMode {

    // This name MUST match the servo name in your Driver Station configuration.
    private static final String SERVO_NAME = "servo";

    // The two ends of the 180-degree sweep (60 and 240 degrees out of 300).
    // Swap them to reverse the direction.
    private static final double POSITION_A = 0.2;
    private static final double POSITION_B = 0.8;

    // How long to wait at each end, in milliseconds.
    private static final long MOVE_TIME_MS = 1000;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, SERVO_NAME);

        // Use the Studica servo's full 500-2500 us range. Without this the SDK
        // defaults to 600-2400 us, which would make the 0.2-0.8 sweep a little
        // less than 180 degrees. The cast is safe on REV hubs; the check keeps
        // the code working on other controllers.
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmRange(500, 2500));
        }

        // Start at position A so the first move is always the same.
        servo.setPosition(POSITION_A);

        telemetry.addLine("Servo Sweep ready");
        telemetry.addLine("The servo will sweep 180 degrees back and forth.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(POSITION_B);
            telemetry.addData("Direction", "Clockwise -> %.2f", POSITION_B);
            telemetry.update();
            sleep(MOVE_TIME_MS);

            servo.setPosition(POSITION_A);
            telemetry.addData("Direction", "Counterclockwise -> %.2f", POSITION_A);
            telemetry.update();
            sleep(MOVE_TIME_MS);
        }
    }
}
