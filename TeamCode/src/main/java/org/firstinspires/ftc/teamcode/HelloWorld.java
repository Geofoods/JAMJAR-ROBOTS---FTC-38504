package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The simplest possible OpMode: says "Hello World" on the Driver Station telemetry.
 *
 * Select "Hello World" from the OpMode list on the Driver Station, then press INIT
 * followed by START. Rename it below if you want a different name in the list.
 */
@TeleOp(name = "Hello World", group = "Examples")
public class HelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Shown on the Driver Station while the OpMode is initialized.
        telemetry.addLine("Hello World");
        telemetry.addData("Status", "Initialized - press START");
        telemetry.update();

        waitForStart();

        // Stay running until the driver presses STOP.
        while (opModeIsActive()) {
            telemetry.addLine("Hello World");
            telemetry.addData("Status", "Running");
            telemetry.update();

            idle();
        }
    }
}
