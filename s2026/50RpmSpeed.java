package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Wheels at 50 RPM", group = "Examples")
public class RpmSpeed extends LinearOpMode {
    DcMotor intakeWheelR;
    DcMotor intakeWheelL;
    double targetTicksPerSecond;

    @Override
    public void runOpMode() {
        // Initialize motors from configuration
        intakeWheelR = hardwareMap.get(DcMotor.class, "intakeWheelR"); // M2
        intakeWheelL = hardwareMap.get(DcMotor.class, "intakeWheelL"); // M3

        // Calculate target velocity in ticks per second
        double targetRPM = 50;
        double ticksPerRev = 560; // Example for REV HD Hex motor
        targetTicksPerSecond = (targetRPM * ticksPerRev) / 60.0; // ≈ 466.67

        // Set mode to use encoder for velocity control
        intakeWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.x) {
                // Run both motors at 50 RPM
                intakeWheelR.setVelocity(targetTicksPerSecond);
                intakeWheelL.setVelocity(targetTicksPerSecond);
            } else {
                // Stop motors when button is not pressed
                intakeWheelR.setVelocity(0);
                intakeWheelL.setVelocity(0);
            }
            telemetry.addData("Button Pressed", gamepad2.x);
            telemetry.addData("Right RPM", (intakeWheelR.getVelocity() * 60.0) / ticksPerRev);
            telemetry.addData("Left RPM", (intakeWheelL.getVelocity() * 60.0) / ticksPerRev);
            telemetry.update();
        }
    }
}
