package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "Allcode", group = "Opmode ProfoundPython")

public class Allcode extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    // Declare Hardware properties
    private Servo clawCenter = null;
	private int target0 = 0;
    @Override
    /*At the beginning, all motors and servos are set at the init status.
    * Make sure hardware definition matches the Control Station (Android phone) configuration file.
    * All Names are case sensitive.
    */
    public void init() {
        
        //3 Servos are connected to the Control Hub-Servo ports
        clawCenter = hardwareMap.get(Servo.class, "s0");
        clawCenter.setPosition(0.9);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        runtime.reset();

        //sleep(1000);
        move();
    }

    @Override
    public void start() {
        runtime.reset();
    }
}
