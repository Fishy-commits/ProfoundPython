
package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "lift_test", group = "Opmode RamEaters")

public class lift_test extends OpMode {

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
        clawCenter.setPosition(0.5);
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
            target1 = -250;
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
            
            clawCenter.setPosition(0.5);
            
            telemetry.addData("Status", "gamepad2.y");
            telemetry.update();
            
        } else if (gamepad2.a) { //when a pressed, moving to the backboard position
            
            target2 = 950;
            target0 = 0 - target2;
            target1 = -275;
            
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
            target1 = -200;
            
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
            
        
        } else if (gamepad1.dpad_up) {
            
           target2 = 500;
           target0 = 0 - target2;
           target1 = 400;
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
           clawLeft.setPosition(0.2);
           clawRight.setPosition(0); 
        } else if (gamepad1.dpad_down) {
            
            target2 = 500;
            target0 = 0 - target2;
            target1 = 3000;
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.2);
            liftmotor2.setPower(0.15);
           
            telemetry.addData("Status", "gamepad1.dpad_down");
            telemetry.update();
		} else if (gamepad1.dpad_left) {
			target2 = 700;
			target0 = 0 - target2;
			target1 = 3400;
			liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.2);
            liftmotor2.setPower(0.15);
        } else if (gamepad1.dpad_right) {
			target2 = 300;
			target0 = 0 - target2;
			target1 = 3400;
            liftmotor0.setTargetPosition(target0);
            liftmotor1.setTargetPosition(target1);
            liftmotor2.setTargetPosition(target2);
            liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            liftmotor0.setPower(0.15);
            liftmotor1.setPower(0.2);
            liftmotor2.setPower(0.15);
        }
    }
}
