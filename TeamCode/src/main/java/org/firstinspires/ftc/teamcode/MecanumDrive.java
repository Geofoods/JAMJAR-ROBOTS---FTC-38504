package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Mecanum drive using gamepad1.
 *
 *   Left stick  : push forward/back to drive, push left/right to strafe
 *   Right stick : push left/right to rotate (yaw)
 *
 * Wheel layout (as requested):
 *   motor 0 = front right
 *   motor 1 = front left
 *   motor 2 = back right
 *   motor 3 = back left
 */
@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends LinearOpMode {

    // These names MUST match the motor names in your Driver Station robot configuration.
    // "motor0"..."motor3" are the default config names; change them here if yours differ.
    private static final String FRONT_RIGHT_NAME = "motor0";
    private static final String FRONT_LEFT_NAME  = "motor1";
    private static final String BACK_RIGHT_NAME  = "motor2";
    private static final String BACK_LEFT_NAME   = "motor3";

    // Joystick values below this are treated as zero so the robot does not creep.
    private static final double DEADBAND = 0.05;

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        backRight  = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);
        backLeft   = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);

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

        telemetry.addLine("Mecanum Drive ready");
        telemetry.addLine("Left stick: drive/strafe   Right stick: rotate");
        telemetry.update();

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

            telemetry.addData("Sticks", "drive=%.2f  strafe=%.2f  rotate=%.2f", axial, lateral, yaw);
            telemetry.addData("Front L/R", "%.2f  %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  L/R", "%.2f  %.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }

    /** Treats tiny stick values as zero, so stick drift does not move the robot. */
    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }
}
