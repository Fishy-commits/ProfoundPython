package org.firstinspires.ftc;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "BlueLeftAuto", group = "Opmode ProfoundPython")
public class BlueLeftAuto extends LinearOpMode {


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
    private IMU imu;
    private double TURN_P = 0.010;
    
    private int target0 = 0;
    private int target1 = 0;
    private int target2 = 0;
    private int slideMotorTarget = 0;
    private double tclawCenter = 0;                    //Claw Center Servo initial position
    private int on_off1 = 0;                        //An indicator for left claw open/close status 
    private int on_off2 = 0;                        //An indicator for right claw open/close status
    
    String test = "";
    
        private void caseLoc() {
            placeSample();
            move(5000,2000,0,0.5,500);
            sleep(500);
            gyroTurn(150);
            sleep(500);
            move(0,3000,0,0.5,500);
            move(0,3500,0,0.5,500);
            move(3000,0,0,0.5,500);
            move(3800,0,0,0.5,500);
            openClaw();
            sleep(1000);
            move(-3800,0,0,0.5,500);
            initPosition();
            sleep(1500);
            move(2000,0,0,0.5,500);
            sleep(10000);
            


        
        telemetry.update();

    }

    @Override
    public void runOpMode() {

        imuInit();

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
        clawRight = hardwareMap.get(Servo.class, "Es0");
        clawCenter = hardwareMap.get(Servo.class, "Es2");

        slideMotor = hardwareMap.get(DcMotor.class, "Em1");       
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        initPosition();

        waitForStart();
        caseLoc();

    } 
        private void closeClaw(){
        clawLeft.setPosition(0.1);
        clawRight.setPosition(0.1);
        telemetry.addData("Status: ", "Close both claws");
        telemetry.update();
    }
        private void openClaw() {
        clawLeft.setPosition(0.3);
        clawRight.setPosition(0);
        telemetry.addData("Status: ", "Open both claws");
        telemetry.update();
    }
    
    //To grab the pixel, move both primary and secondary single bars to their positions
    private void placeSample() {
        target2 = 1500;
           target0 = 0 - target2;
           liftmotor0.setTargetPosition(target0);
           liftmotor2.setTargetPosition(target2);
           liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           liftmotor0.setPower(0.5);
           liftmotor2.setPower(0.5);
           telemetry.addData("Status: ", "Press Gamepad1.dpad_up to raise both bars and be ready to hang");
           telemetry.update();
           
           clawLeft.setPosition(0.1);
           clawRight.setPosition(0.1); 
           clawCenter.setPosition(0);

           slideMotorTarget = -6500;
            slideMotor.setTargetPosition(slideMotorTarget);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(5); 
    } 
    
    //To move both primary and secondary single bars to their initial positions
    private void initPosition() {
        target2 = 0;
        target0 = 0 - target2;
        target1 = 0;
        
        liftmotor0.setTargetPosition(target0);
        liftmotor1.setTargetPosition(target1);
        liftmotor2.setTargetPosition(target2);
        liftmotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftmotor0.setPower(0.15);
        liftmotor1.setPower(0.1);
        liftmotor2.setPower(0.15);
        //clawCenter.setPosition(0);
        clawCenter.setPosition(tclawCenter);
        closeClaw();
        
        slideMotorTarget = 0;
            slideMotor.setTargetPosition(slideMotorTarget);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(5); 

        telemetry.addData("Status", "initPosition");
        telemetry.update();
    }

    //To move both primary and secondary single bars to their initial positions
    private void goBackToInit() {
        target2 = 300;
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
        
        clawCenter.setPosition(0);
        telemetry.addData("Status", "Go Back to Init Position");
        telemetry.update();

    }
    
    //To move the robot to the board, move both primary and secondary single bars to their positions
    private void movePosition(){
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

    }
    
    //To put the pixel to the board, move both primary and secondary single bars to their positions
    private void putPixel(){
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
        
        clawCenter.setPosition(0);
        telemetry.addData("Status", "putPixel");
        telemetry.update();
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

    private void imuInit() {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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
