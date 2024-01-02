
package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "motor_test", group = "Opmode RamEaters")

public class motor_test extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    // Declare Hardware
    private DcMotor liftmotor0 = null;               //Lift Motor 0
    private DcMotor liftmotor1 = null;               //Lift Motor 1
    private DcMotor liftmotor2 = null;               //Lift Motor2
    private Servo clawCenter = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private Servo airplane = null;
    private int target0 = 0;
    private int target1 = 0;
    private int target2 = 0;
    private double tclawCenter = 0.50;
    
    @Override
    public void init() {

 
        liftmotor0 = hardwareMap.get(DcMotor.class, "Em0");
        liftmotor1 = hardwareMap.get(DcMotor.class, "Em1");
        liftmotor2 = hardwareMap.get(DcMotor.class, "Em2");
        liftmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawCenter = hardwareMap.get(Servo.class, "Es0");
        clawCenter.setPosition(tclawCenter);
        clawLeft = hardwareMap.get(Servo.class, "Es1");
        airplane = hardwareMap.get(Servo.class, "s5");
        airplane.setPosition(0.5);
        clawRight = hardwareMap.get(Servo.class, "Es2");
        
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

    private void move() {
        
        
        if (gamepad2.x) { //when x pressed, grab pixel position
            
            
            target2 = 1000;
            target0 = 0 - target2;
            target1 = 1050;
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor0.setPower(0.1);
            liftmotor1.setPower(0.1);
            liftmotor2.setPower(0.1);
            telemetry.addData("Status", "gamepad2.x");
            telemetry.update();
            
        }
        else if (gamepad2.y) {  // when y pressed, goes back to init position
            
            target0 = 0;
            target1 = 0;
            target2 = 0;
            
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.1);
            liftmotor2.setPower(0.15);
            
            clawCenter.setPosition(0.50);
            
            telemetry.addData("Status", "gamepad2.y");
            telemetry.update();
            
        } else if (gamepad2.a) { //when a pressed, moving to the backboard position
            
            target2 = 875;
            target0 = 0 - target2;
            target1 = 500;
            
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.1);
            liftmotor2.setPower(0.15);
            telemetry.addData("Status", "gamepad2.y");
            telemetry.update();
            
        } else if (gamepad2.b) {    //when b pressed, put pixel position
            
            target2 = 650;
            target0 = 0 - target2;
            target1 = 500;
            
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.1);
            liftmotor2.setPower(0.15);
            telemetry.addData("Status", "gamepad2.y");
            telemetry.update();
            
        }  
        else if (gamepad2.right_bumper) {
           
           clawLeft.setPosition(0.2);
           clawRight.setPosition(0);
        } else if (gamepad2.left_bumper) {
            
           clawLeft.setPosition(0);
           clawRight.setPosition(0.2);
           
        } else if (gamepad2.dpad_left) {
            
           airplane.setPosition(0);
            
        
        } else if (gamepad2.dpad_up) {
            
           tclawCenter = tclawCenter + 0.001;
           clawCenter.setPosition(tclawCenter);
           telemetry.addData("Status", "gamepad2.dpad_up");
           telemetry.update();
        
        } else if (gamepad2.dpad_down) {
            
          tclawCenter = tclawCenter - 0.001;
          clawCenter.setPosition(tclawCenter);
          telemetry.addData("Status", "gamepad2.dpad_down");
          telemetry.update();
        
        }
    }
}
