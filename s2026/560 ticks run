package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Run 560 Ticks", group = "Examples")
public class Run560Ticks extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() {
        // Initialize motor from configuration
        motor = hardwareMap.get(DcMotor.class, "motorName"); // Replace with your motor name

        // Reset encoder and set mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target position (560 ticks)
        motor.setTargetPosition(560);

        // Set power level (0.0 to 1.0)
        motor.setPower(0.5); // Adjust as needed for speed

        waitForStart();

        // Start moving to target
        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Target", motor.getTargetPosition());
            telemetry.update();
        }

        // Stop motor once target is reached
        motor.setPower(0);
    }
}
