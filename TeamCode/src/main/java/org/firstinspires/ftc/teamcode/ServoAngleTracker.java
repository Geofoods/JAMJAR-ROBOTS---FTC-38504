package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Reports the angle of servo1 and servo2 on the Driver Station.
 *
 *   Servo 1 and servo 2 must be named "servo1" and "servo2" in the robot
 *   configuration. They start at 0 degrees.
 *
 *   X : move both servos to 0 degrees
 *   Y : move both servos to 180 degrees
 *
 * IMPORTANT: a standard RC servo has no position feedback wire, so the SDK
 * cannot measure the physical angle of the horn. Servo.getPosition() returns
 * the angle the code LAST COMMANDED, not where the servo actually is. This
 * program therefore tracks/prints the commanded angle. If the servo is
 * unplugged, stalled, or blocked, the printed angle will still show the
 * command.
 */
@TeleOp(name = "Servo Angle Tracker", group = "Examples")
public class ServoAngleTracker extends LinearOpMode {

    // These names MUST match the servo names in the Driver Station configuration.
    private static final String SERVO_1_NAME = "servo1";
    private static final String SERVO_2_NAME = "servo2";

    // The Studica Multi-Mode Smart Servo in standard mode travels 300 degrees
    // over the 500-2500 us pulse range. Change this to 180.0 if your servos
    // were reprogrammed to a 180-degree range.
    private static final double SERVO_TRAVEL_DEGREES = 300.0;

    private static final double X_DEGREES = 0.0;
    private static final double Y_DEGREES = 180.0;

    // One servo is mounted upside down, so its direction is reversed in code.
    // If it is actually servo1 that is flipped, swap these two values.
    private static final boolean REVERSE_SERVO_1 = false;
    private static final boolean REVERSE_SERVO_2 = true;

    private Servo servo1;
    private Servo servo2;

    @Override
    public void runOpMode() {
        servo1 = initServo(SERVO_1_NAME, REVERSE_SERVO_1);
        servo2 = initServo(SERVO_2_NAME, REVERSE_SERVO_2);

        telemetry.addLine("Servo Angle Tracker ready");
        telemetry.addLine("X: 0 deg   Y: 180 deg");
        telemetry.addLine("Angles shown are commanded positions.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Commanding a position is what updates getPosition(), so these
            // buttons also let you watch the reported angle change.
            if (gamepad1.x) {
                setServoAngle(servo1, X_DEGREES);
                setServoAngle(servo2, X_DEGREES);
            } else if (gamepad1.y) {
                setServoAngle(servo1, Y_DEGREES);
                setServoAngle(servo2, Y_DEGREES);
            }

            double servo1Position = servo1.getPosition();
            double servo2Position = servo2.getPosition();

            telemetry.addData("Servo 1", "%.1f deg  (position %.3f)",
                    positionToDegrees(servo1Position), servo1Position);
            telemetry.addData("Servo 2", "%.1f deg  (position %.3f)",
                    positionToDegrees(servo2Position), servo2Position);
            telemetry.addData("Note", "Commanded angle, not measured");
            telemetry.update();
        }
    }

    /** Gets a servo, sets its range/direction, and moves it to 0 degrees. */
    private Servo initServo(String name, boolean reversed) {
        Servo servo = hardwareMap.get(Servo.class, name);
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmRange(500, 2500));
        }
        servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        setServoAngle(servo, X_DEGREES);
        return servo;
    }

    /** Commands the servo to an angle, converted to the SDK's 0.0-1.0 range. */
    private void setServoAngle(Servo servo, double degrees) {
        double position = degrees / SERVO_TRAVEL_DEGREES;
        servo.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    /** Converts an SDK position (0.0-1.0) back into degrees. */
    private double positionToDegrees(double position) {
        if (Double.isNaN(position)) {
            return Double.NaN;
        }
        return position * SERVO_TRAVEL_DEGREES;
    }
}
