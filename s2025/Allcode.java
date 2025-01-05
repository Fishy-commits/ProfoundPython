package org.firstinspires.ftc;

import android.graphics.Color;
import android.util.Size;															//color sensor
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;								//distance sensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;


@TeleOp(name = "AutoTest", group = "Opmode Profound Pythons")
public class AutoTest extends LinearOpMode {



    private VisionPortal visionPortal;
	private String savedColorMatch = "NULL";		//
    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    
    private DcMotor liftmotor0 = null;              //Lift Motor 0 to control the primary single bar
    private DcMotor liftmotor1 = null;              //Lift Motor 1 to control the secondary singale bar
    private DcMotor liftmotor2 = null;
    private DcMotor slideMotor = null;
    private Servo clawCenter = null;                
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private final ElapsedTime runtime = new ElapsedTime();
    private double TURN_P = 0.010;
    
    private int target0 = 0;
    private int target2 = 0;
	private int slideMotorTarget = 0;
    private double tclawCenter = 0.45;                    //Claw Center Servo initial position
    private double planeTarget = 0.5;               //Airplane Servo initial position
    private int on_off1 = 0;                        //An indicator for left claw open/close status 
    private int on_off2 = 0;                        //An indicator for right claw open/close status
    private boolean findTargetColor = false;
	
    String test = "";
    
    @Override
    public void runOpMode() {

        initColorSensor();
		initDistanceSensor();
		initMotors();

        initPosition();

        waitForStart();

        if (opModeIsActive()) {
            if(detectColor("YELLOW"))
				MoveToTarget();

            //sleep(15000);
        }

    } 

private void MoveToTarget() {	

		
      

		// After exiting the vision loop...
		if (savedColorMatch == "RED")     {		//red
		// your code here: robot actions if the ROI was RED
        }

        telemetry.update();

    }

    //To open left claws
    private void openClaw() {
        clawLeft.setPosition(0.2);
        clawRight.setPosition(0);
        telemetry.addData("Status: ", "Open both claws");
        telemetry.update();
    }
    
    //To close left claws
    private void closeClaw() {
	    clawLeft.setPosition(0.1);
        clawRight.setPosition(0.1);
        telemetry.addData("Status: ", "Close both claws");
        telemetry.update();
    }
    
    //To grab the pixel, move both primary and secondary single bars to their positions
    private void grabPixel() {
        target2 = 975;
        target0 = 0 - target2;
        liftmotor0.setTargetPosition(target0);
        liftmotor2.setTargetPosition(target2);
        liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor0.setPower(0.1);
        liftmotor1.setPower(0.1);
        liftmotor2.setPower(0.1);
        clawCenter.setPosition(0.20);
        telemetry.addData("Status", "grabPixel");
        telemetry.update();
    } 
    
    //To move both primary and secondary single bars to their initial positions
    private void initPosition() {
		target0 = 0;
        target2 = 0;
        slideMotorTarget = 0;
        liftmotor0.setTargetPosition(target0);
        liftmotor2.setTargetPosition(target2);
        liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftmotor0.setPower(0.15);
        liftmotor2.setPower(0.15);
        
        clawCenter.setPosition(tclawCenter);
        
        slideMotor.setTargetPosition(slideMotorTarget);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);        
        
        telemetry.addData("Status: ", "Initial Position");
        telemetry.update();
    }

    //To move both primary and secondary single bars to their initial positions
    private void goBackToInit() {
        target2 = 300;
        target0 = 0 - target2;
        
        liftmotor0.setTargetPosition(target0);
        liftmotor2.setTargetPosition(target2);
        liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftmotor0.setPower(0.15);
        liftmotor1.setPower(0.1);
        liftmotor2.setPower(0.15);
        
        clawCenter.setPosition(0.45);
        telemetry.addData("Status", "Go Back to Init Position");
        telemetry.update();

    }
    
    //To move the robot to the board, move both primary and secondary single bars to their positions
    private void movePosition(){
        target2 = 875;
        target0 = 0 - target2;
        
        liftmotor0.setTargetPosition(target0);
        liftmotor2.setTargetPosition(target2);
        liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftmotor0.setPower(0.15);
        liftmotor1.setPower(0.1);
        liftmotor2.setPower(0.15);

    }
    
    //To put the pixel to the board, move both primary and secondary single bars to their positions
    private void putPixel(){
        target2 = 650;
        target0 = 0 - target2;
        
        liftmotor0.setTargetPosition(target0);
        liftmotor2.setTargetPosition(target2);
        liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftmotor0.setPower(0.15);
        liftmotor1.setPower(0.1);
        liftmotor2.setPower(0.15);
        
        clawCenter.setPosition(0);
        telemetry.addData("Status", "putPixel");
        telemetry.update();
    }
	private void initMotors();{
	
	    leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");

        leftWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
//3 Lift Motors are connected to the Expansion - Motor ports
        liftmotor0 = hardwareMap.get(DcMotor.class, "Em0");
        liftmotor1 = hardwareMap.get(DcMotor.class, "Em1");
        liftmotor2 = hardwareMap.get(DcMotor.class, "Em2");
        liftmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawLeft = hardwareMap.get(Servo.class, "Es1");
        clawRight = hardwareMap.get(Servo.class, "Es2");
        clawCenter = hardwareMap.get(Servo.class, "Es0");
	}
	private void initDistanceSensor(){
	
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
		
		waitForStart();
	}
	
    private void initColorSensor() {

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
            

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);  // Speed up telemetry updates, Just use for debugging.

    }   // end method initColorSensor()

    private boolean detectColor(String targetColor) {
        boolean find = false;

        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream\n");
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
			
            telemetry.addData("Best Match:", result.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            telemetry.update();

		if (result.closestSwatch == Swatch.RED)     {		//red
			savedColorMatch = "RED";
		}
		else if (result.closestSwatch == Swatch.BLUE){
			savedColorMatch = "BLUE";
		}
		else{
			savedColorMatch = "YELLOW";
		}
		
		if(savedColorMatch = targetColor){
			find = true;
			break;
		}else{
			sleep(20);
		}
        }
        
        return find;

    } 

    private void move(double drive,
                      double strafe,
                      double rotate, double power, int iSleep) {

        double powerLeftF;
        double powerRightF;
        double powerLeftR;
        double powerRightR;

        powerLeftF = drive + strafe + rotate;
        powerLeftR = drive - strafe + rotate;

        powerRightF = drive - strafe - rotate;
        powerRightR = drive + strafe - rotate;

        leftWheelF.setPower(power);
        leftWheelR.setPower(power);
        rightWheelF.setPower(power);
        rightWheelR.setPower(power);

        leftWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftWheelF.setTargetPosition(leftWheelF.getCurrentPosition() + (int) (-powerLeftF));
        leftWheelR.setTargetPosition(leftWheelR.getCurrentPosition() + (int) (-powerLeftR));

        rightWheelF.setTargetPosition(rightWheelF.getCurrentPosition() + (int) (powerRightF));
        rightWheelR.setTargetPosition(rightWheelR.getCurrentPosition() + (int) (powerRightR));

        leftWheelF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheelF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(iSleep);

        leftWheelF.setPower(0);
        leftWheelR.setPower(0);
        rightWheelF.setPower(0);
        rightWheelR.setPower(0);

    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void justTurn(double deg) {

        leftWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int i = 0;
        int iMAX = 200;

        double target_angle = getHeading() - deg;
        while (i < iMAX && opModeIsActive() && Math.abs((target_angle - getHeading()) % 360) > 3) {
            double error_degrees = (target_angle - getHeading()) % 360; //Compute Error
            double motor_output = Range.clip(error_degrees * TURN_P, -.6, .6); //Get Correction

            if (Math.abs(motor_output) < 0.020)
                i = 10001;
            // Send corresponding powers to the motors
            leftWheelF.setPower(1 * motor_output);
            leftWheelR.setPower(1 * motor_output);
            rightWheelF.setPower(1 * motor_output);
            rightWheelR.setPower(1 * motor_output);

            telemetry.addData("motor_output : ", motor_output);
            telemetry.addData("target_angle : ", target_angle);

            i++;
            telemetry.addData("i : ", i);
            telemetry.update();

        }
    }

    private void gyroTurn(double target_angle) {

        leftWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double currentHeading = getHeading();

        double delta = Math.abs((currentHeading - target_angle));

        int i = 0;
        int iMAX = 200;

        while (i < iMAX && opModeIsActive() && delta > 0.01)
        {

            double error_degrees = (target_angle - currentHeading) % 360.0;

            double motor_output = Range.clip(error_degrees * TURN_P, -0.6, 0.6);

            if (Math.abs(motor_output) < 0.020)
                i = 10001;

            leftWheelF.setPower(1 * motor_output);
            leftWheelR.setPower(1 * motor_output);
            rightWheelF.setPower(1 * motor_output);
            rightWheelR.setPower(1 * motor_output);


            currentHeading = getHeading();
            telemetry.addData("motor_output : ", motor_output);
            telemetry.addData("target_angle : ", target_angle);
            telemetry.addData("currentHeading : ", currentHeading);
            delta = Math.abs((currentHeading- target_angle));
            telemetry.addData("delta : ", delta);

            i++;
            telemetry.addData("i : ", i);
            telemetry.update();

        }

        //sleep(500);

        leftWheelF.setPower(0);
        leftWheelR.setPower(0);
        rightWheelF.setPower(0);
        rightWheelR.setPower(0);

    }
}
