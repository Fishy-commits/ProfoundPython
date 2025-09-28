package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Run at 50 RPM", group = "Examples")
public class RunAt50RPM extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() {
        // Initialize motor from configuration
        motor = hardwareMap.get(DcMotor.class, "motorName"); // Replace with your motor name

        // Set mode to use encoder for velocity control
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calculate target velocity in ticks per second
        double targetRPM = 50;
        double ticksPerRev = 560; // Example for REV HD Hex motor
        double targetTicksPerSecond = (targetRPM * ticksPerRev) / 60.0; // ≈ 466.67

        waitForStart();

        // Set motor velocity
        motor.setVelocity(targetTicksPerSecond);

        // Keep running while OpMode is active
        while (opModeIsActive()) {
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current Velocity (ticks/sec)", motor.getVelocity());
            telemetry.addData("Current RPM", (motor.getVelocity() * 60.0) / ticksPerRev);
            telemetry.update();
        }

        // Stop motor when OpMode ends
        motor.setVelocity(0);
    }
}
