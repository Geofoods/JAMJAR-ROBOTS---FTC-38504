package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Mecanum drive using gamepad1.
 *
 *   Left stick  : push forward/back to drive, push left/right to strafe
 *   Right stick : push left/right to rotate (yaw)
 *   A (press)   : start intakeMotor at 100% power and keep it running
 *   B (press)   : start flywheelMotor at 100% power and keep it running
 *   X (press)   : swing servo1 down to the test angle and hold
 *   Y (press)   : return servo1 to its start position
 *
 * Wheel layout (as configured on the Driver Station):
 *   motorFrontRight = motor port 0
 *   motorFrontLeft  = motor port 1
 *   motorBackRight  = motor port 2
 *   motorBackLeft   = motor port 3
 *
 * Servo1 is driven by X and stays limp until it is pressed, so it does not jump
 * when the OpMode initializes. Servo2 is mechanically linked to servo1 with
 * polycarbonate, so its PWM is disabled to leave it limp: it follows servo1 on
 * its own and is never commanded by the code.
 */
@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends LinearOpMode {

    // These names MUST match the names in your Driver Station robot configuration.
    private static final String FRONT_RIGHT_NAME = "motorFrontRight";
    private static final String FRONT_LEFT_NAME  = "motorFrontLeft";
    private static final String BACK_RIGHT_NAME  = "motorBackRight";
    private static final String BACK_LEFT_NAME   = "motorBackLeft";
    private static final String INTAKE_NAME      = "intakeMotor";
    private static final String FLYWHEEL_NAME    = "flywheelMotor";
    private static final String SERVO_1_NAME     = "servo1";
    private static final String SERVO_2_NAME     = "servo2";
    private static final String LAUNCHER_SERVO_NAME = "launcherServo";

    // Left trigger moves the launcher servo to this position; right trigger returns it to 0.
    private static final double LAUNCHER_TRIGGER_POSITION = 0.2;
    private static final double LAUNCHER_HOME_POSITION    = 0.0;
    private static final double TRIGGER_THRESHOLD         = 0.5;

    // How hard the intake and flywheel run while their button is held.
    private static final double INTAKE_POWER   = 1.0;
    private static final double FLYWHEEL_POWER = 1.0;

    // Joystick values below this are treated as zero so the robot does not creep.
    private static final double DEADBAND = 0.05;

    // Servo1 is a positional servo with 300 degrees of travel (Studica standard
    // mode). Positions are 0.0-1.0, so 90 degrees is 0.3 of the range. With the
    // test starting at 0.5, the largest possible down swing is 150 degrees.
    private static final double SERVO_TRAVEL_DEGREES = 300.0;
    private static final double TEST_SWING_DEGREES   = 150.0;
    private static final double TEST_SWING_POSITION  = TEST_SWING_DEGREES / SERVO_TRAVEL_DEGREES;

    // "Down" is a decreasing position on this robot. Change to +1.0 if X moves
    // the stick up instead.
    private static final double TEST_DOWN_DIRECTION = -1.0;

    // Set to true if servo1 is mounted upside down and needs its direction
    // reversed in code. Servo2 is unpowered, so its direction does not matter.
    private static final boolean REVERSE_SERVO_1 = false;

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor intakeMotor;
    private DcMotor flywheelMotor;

    private Servo servo1;
    private Servo servo2;
    private Servo launcherServo;

    // The hub's reported servo position is unreliable before the servo has been
    // commanded (it can read 0.0 even when the horn is elsewhere). Track the
    // position we last commanded so the test cannot get stuck against an end.
    private double servoPosition = 0.5;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        backRight  = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);
        backLeft   = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);

        // Mechanism motors. These keep the SDK's default FLOAT behavior so they
        // spin down on their own when the button is released.
        intakeMotor   = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        flywheelMotor = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);

        // Neither mechanism motor has an encoder, so they are driven open-loop
        // with setPower() only. RUN_WITHOUT_ENCODER is the SDK default; setting
        // it explicitly keeps them out of any encoder-based mode.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The intake is mounted so that positive power would pull inward/backward
        // on this robot, so reverse it here and keep positive power meaning
        // "intake" everywhere else in the code.
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Most mecanum robots need the motors on one side reversed so that positive
        // power drives the robot forward. If a wheel spins backward during testing,
        // flip that motor between FORWARD and REVERSE.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE stops the wheels quickly when the sticks are released.
        // Change to FLOAT if you prefer the robot to coast.
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Servo1 is set up (range/direction only) but deliberately NOT commanded a
        // position here. The hub leaves servo PWM disabled at init, so it stays limp
        // and does not jump when the OpMode initializes. It moves only after X.
        servo1 = initServo(SERVO_1_NAME, REVERSE_SERVO_1);

        // Servo2 is only a passive follower (linked to servo1 by polycarbonate),
        // so cut its PWM. With no power it has no holding torque and moves freely
        // whenever servo1 pushes it.
        servo2 = hardwareMap.get(Servo.class, SERVO_2_NAME);
        boolean servo2Disabled = disableServo(servo2);

        launcherServo = hardwareMap.get(Servo.class, LAUNCHER_SERVO_NAME);

        telemetry.addLine("Mecanum Drive ready");
        telemetry.addLine("Left stick: drive/strafe   Right stick: rotate");
        telemetry.addLine("A (press): intake 100% until stopped   B (press): flywheel 100% until stopped");
        telemetry.addLine(String.format("X (press): servo1 down %.0f deg   Y (press): return",
                TEST_SWING_DEGREES));
        telemetry.addLine("servo1: limp until X is pressed");
        telemetry.addLine(servo2Disabled
                ? "servo2: PWM disabled (free to follow servo1)"
                : "servo2: WARNING - could not disable PWM");
        telemetry.update();

        // X saves the current position, goes down by the test angle, and holds.
        // Y returns to the saved position. A fresh press is required so holding
        // a button does not repeat the move.
        boolean lastA = false;
        boolean intakeOn = false;
        boolean lastB = false;
        boolean flywheelOn = false;
        final double startPosition = 0.5;
        String servoState = "limp (press X)";

        waitForStart();

        while (opModeIsActive()) {
            // Pushing a stick forward reports a negative y, so negate it.
            double axial   = -applyDeadband(gamepad1.left_stick_y); // forward +
            double lateral =  applyDeadband(gamepad1.left_stick_x); // strafe right +
            double yaw     =  applyDeadband(gamepad1.right_stick_x); // turn clockwise +

            // Combine the three requests into a power for each wheel.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize so no wheel is asked for more than 100% power.
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // A and B start the mechanism motors on a fresh press and leave them running.
            if (gamepad1.a && !lastA) {
                intakeOn = true;
            }
            lastA = gamepad1.a;
            intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0.0);
            if (gamepad1.b && !lastB) {
                flywheelOn = true;
            }
            lastB = gamepad1.b;
            flywheelMotor.setPower(flywheelOn ? FLYWHEEL_POWER : 0.0);

            // X saves the current position and swings servo1 down to the test
            // angle, where it stays. Y brings it back to the saved position.
            // Button edges only, so holding a button does not repeat the move.
            // Each button always sends its own position, so neither depends on
            // the other having been pressed first.
            if (gamepad1.x) {
                setServoPosition(servo1, clampPosition(
                        startPosition + TEST_DOWN_DIRECTION * TEST_SWING_POSITION));
                servoState = "down (press Y)";
            } else if (gamepad1.y) {
                setServoPosition(servo1, startPosition);
                servoState = "at start (press X)";
            }

            if (gamepad1.left_bumper) {
                if (launcherServo instanceof ServoImplEx) {
                    ((ServoImplEx) launcherServo).setPwmEnable();
                }
                launcherServo.setPosition(LAUNCHER_TRIGGER_POSITION);
            } else if (gamepad1.right_bumper) {
                disableServo(launcherServo);
            }

            telemetry.addData("Buttons", "X=%b Y=%b LB=%b RB=%b LT=%.2f RT=%.2f",
                    gamepad1.x, gamepad1.y, gamepad1.left_bumper, gamepad1.right_bumper,
                    gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("Sticks", "drive=%.2f  strafe=%.2f  rotate=%.2f", axial, lateral, yaw);
            telemetry.addData("Front L/R", "%.2f  %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  L/R", "%.2f  %.2f", backLeftPower, backRightPower);
            telemetry.addData("Intake (A)", "%.0f%%", intakeOn ? INTAKE_POWER * 100 : 0.0);
            telemetry.addData("Flywheel (B)", "%.0f%%", flywheelOn ? FLYWHEEL_POWER * 100 : 0.0);
            telemetry.addData("Servo 1", "%.2f (commanded)  %s", servoPosition, servoState);
            telemetry.addData("Servo 2", "unpowered - follows servo1");
            telemetry.addData("Servo Note", "X = down, Y = return to start");
            telemetry.update();
        }
    }

    /**
     * Gets a servo and configures its PWM range and direction. It does NOT send a
     * position, so the servo stays limp (no jump) until it is explicitly commanded.
     */
    private Servo initServo(String name, boolean reversed) {
        Servo servo = hardwareMap.get(Servo.class, name);
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmRange(500, 2500));
        }
        servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        return servo;
    }

    /**
     * Cuts the PWM signal to a servo so it gets no power and no holding torque.
     * Returns true if the servo was disabled. Only servos that support PWM
     * control (ServoImplEx, e.g. on the Control Hub) can be disabled this way.
     */
    private boolean disableServo(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmDisable();
            return true;
        }
        return false;
    }

    /**
     * Commands a positional servo to a position in the SDK's 0.0-1.0 range and
     * remembers that position for the next test.
     */
    private void setServoPosition(Servo servo, double position) {
        double clamped = clampPosition(position);
        servoPosition = clamped;
        // Servos are left limp at init, so re-enable PWM before commanding a move.
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmEnable();
        }
        servo.setPosition(clamped);
    }

    /** Keeps a servo position inside the 0.0-1.0 range. */
    private double clampPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }

    /** Treats tiny stick values as zero, so stick drift does not move the robot. */
    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }
}
