package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake Wheels at 50 RPM", group = "Examples")
public class RpmSpeed extends OpMode {
	
	private final ElapsedTime runtime = new ElapsedTime();
	private DcMotor leftWheelF = null;              //Left Wheel Front
    private DcMotor leftWheelR = null;              //Left Wheel Back
    private DcMotor rightWheelF = null;             //Right Wheel Front
    private DcMotor rightWheelR = null;             //Right Wheel Back
	private DcMotor intakeLiftL = null;
    private DcMotor intakeLiftR = null;
	private Servo rotateServo = null;
	
    DcMotorEx intakeWheelR;
    DcMotorEx intakeWheelL;
    double targetTicksPerSecondL = 10000;
    double targetTicksPerSecondR = -10000;
    double targetTicksPerSecondLO = -10000;
    double targetTicksPerSecondRO = 10000;
    
	private int targetL = 0;
	
    @Override
    public void init() {
        // Initialize motors from configuration
		leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");
		
		
		intakeLiftL = hardwareMap.get(DcMotorEx.class, "intakeLiftR"); //Em0
        intakeLiftR = hardwareMap.get(DcMotorEx.class, "intakeLiftR"); //Em1
        intakeWheelR = hardwareMap.get(DcMotorEx.class, "intakeWheelR"); // Em2
     	intakeWheelL = hardwareMap.get(DcMotorEx.class, "intakeWheelL"); // Em3
		rotateServo = hardwareMap.get (Servo.class, "rotateServo"); //s0
		
		
		initPosition(); 
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

	private void initPosition() {
        targetL = -50;
        intakeLiftL.setTargetPosition(targetL);
        intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeLiftL.setPower(0.15);
        
        telemetry.addData("Status: ", "Initial Position");
        telemetry.update();
    }

	private void move(){
        // Calculate target velocity in ticks per second
        double targetRPML = 600;
        double ticksPerRevL = 1000; // Example for REV HD Hex motor
        targetTicksPerSecondL = (targetRPML * ticksPerRevL) / 60.0; // ≈ 10000
        
        double targetRPMR = -600;
        double ticksPerRevR = 1000; // Example for REV HD Hex motor
        targetTicksPerSecondR = (targetRPMR * ticksPerRevR) / 60.0; // ≈ 466.67
        
        double targetRPMLO = -600;
        double ticksPerRevLO = 1000; // Example for REV HD Hex motor
        targetTicksPerSecondLO = (targetRPMLO * ticksPerRevLO) / 60.0; // ≈ 466.67
        
        double targetRPMRO = 600;
        double ticksPerRevRO = 1000; // Example for REV HD Hex motor
        targetTicksPerSecondRO = (targetRPMRO * ticksPerRevRO) / 60.0; // ≈ 466.67

		double drive;
		double strafe;
		double rotateLeft;            // Power for the robot counterclockwise rotation
        double rotateRight;            // Power for the robot clockwise rotation
		double drive2;
        double strafe2;
		
		
		drive = -gamepad1.left_stick_y;        // Negative because the gamepad is weird
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
        // Set mode to use encoder for velocity control
        intakeWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Inside the while loop
			if (gamepad2.right_trigger > 0.1) { // Threshold to detect trigger press
				intakeWheelR.setVelocity(targetTicksPerSecondR * gamepad2.right_trigger);// Scale velocity with trigger
				intakeWheelL.setVelocity(targetTicksPerSecondL * gamepad2.right_trigger);
			} else {
				intakeWheelR.setVelocity(0); // Stop if not pressed
				intakeWheelL.setVelocity(0); //stop if not pressed
			}
			
			if (gamepad2.left_trigger > 0.1) { // Threshold to detect trigger press
			intakeWheelR.setVelocity(targetTicksPerSecondRO * gamepad2.left_trigger);// Scale velocity with trigger
			intakeWheelL.setVelocity(targetTicksPerSecondLO * gamepad2.left_trigger);
			} else {
				intakeWheelL.setVelocity(0); // Stop if not pressed
				intakeWheelR.setVelocity(0); // Stop if not pressed
			}
			if (gamepad2.dpad_up) {
			targetL = 200;
			intakeLiftL.setTargetPosition(targetL);
            intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			intakeLiftL.setPower(0.4);
			}
			if (gamepad2.dpad_left){
				
				//blah bah blah
				
			}
			
			if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) { //when using the left joystick, all wheels receive 0.5 power
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            //powerIntake = intake;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF*0.7);
            leftWheelR.setPower(-powerLeftR*0.7);
            rightWheelF.setPower(powerRightF*0.7);
            rightWheelR.setPower(powerRightR*0.7);
			
		}
			else {
            //when using the right joystick, all wheels receive 0.3 power
            powerLeftF = drive2 + strafe2 + rotateRight;
            powerLeftR = drive2 - strafe2 + rotateRight;

            powerRightF = drive2 - strafe2 - rotateLeft;
            powerRightR = drive2 + strafe2 - rotateLeft;

            leftWheelF.setPower(-powerLeftF*0.5);
            leftWheelR.setPower(-powerLeftR*0.5);

            rightWheelF.setPower(powerRightF*0.5);
            rightWheelR.setPower(powerRightR*0.5);
        }
	}
}
