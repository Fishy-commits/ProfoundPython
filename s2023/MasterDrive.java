
package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "MasterDrive", group = "Opmode RamEaters")

public class MasterDrive extends OpMode {

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
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
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
        
		leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");
		
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
        double drive;
        // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotateLeft;
        double rotateRight;// Power for rotating the robot
        //int intake;


        double drive2;
        double strafe2;

        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotateLeft = gamepad1.left_trigger;
        rotateRight = gamepad1.right_trigger;
        //intake = gamepad2.left_trigger;

        drive2 = -gamepad1.right_stick_y;
        strafe2 = gamepad1.right_stick_x;

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;
        // double powerIntake;
        //intakeWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeWheel1.setPower(1);
        //if full power on left stick
        
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
         
        
         
        } 
		else if (gamepad1.dpad_left) {
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
		if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            //powerIntake = intake;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF*0.5);
            leftWheelR.setPower(-powerLeftR*0.5);

            rightWheelF.setPower(powerRightF*0.5);
            rightWheelR.setPower(powerRightR*0.5);

            //intakeWheel1.setPower(powerIntake);

        } 
		else {
            // else half power
            powerLeftF = drive2 + strafe2 + rotateRight;
            powerLeftR = drive2 - strafe2 + rotateRight;

            powerRightF = drive2 - strafe2 - rotateLeft;
            powerRightR = drive2 + strafe2 - rotateLeft;

            leftWheelF.setPower(-powerLeftF*0.3);
            leftWheelR.setPower(-powerLeftR*0.3);

            rightWheelF.setPower(powerRightF*0.3);
            rightWheelR.setPower(powerRightR*0.3);
        } 
    }
}
