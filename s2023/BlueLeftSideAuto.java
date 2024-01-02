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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.util.List;

@Autonomous(name = "BlueLeftSideAuto", group = "Opmode ProfoundPython")
public class BlueLeftSideAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    // For example: private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Teamprop.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "Redprop",
	   "Blueprop"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;    

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private DcMotor leftWheelF = null;               //Left Wheel Front
    private DcMotor leftWheelR = null;               //Left Wheel Back
    private DcMotor rightWheelF = null;              //Right Wheel Front
    private DcMotor rightWheelR = null;
    
    private DcMotor liftmotor0 = null;              //Lift Motor 0 to control the primary single bar
    private DcMotor liftmotor1 = null;              //Lift Motor 1 to control the secondary singale bar
    private DcMotor liftmotor2 = null;
    private Servo clawCenter = null;                
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private final ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private double TURN_P = 0.010;
    
    private int target0 = 0;
    private int target1 = 0;
    private int target2 = 0;
    private double tclawCenter = 0.45;                    //Claw Center Servo initial position
    private double planeTarget = 0.5;               //Airplane Servo initial position
    private int on_off1 = 0;                        //An indicator for left claw open/close status 
    private int on_off2 = 0;                        //An indicator for right claw open/close status
    
    String test = "";
    
        private void caseLoc(int loc) {

        gyroTurn(0);
      
        //sleep(500);

        if (loc == 1){            //left
            move(250,0,0,0.3,500);
            gyroTurn(40);
            grabPixel();
            sleep(3500);
            openLeftClaw();
            sleep(1000);
            initPosition();
            sleep(5000);
            gyroTurn(90);
            move(8000,0,0,0.7,500);
            move(4000,0,0,0.7,500);
        }
        else if (loc == 2)            //center
        {
            move(500,0,0,0.4,500);
            grabPixel();
            sleep(4000);
            openLeftClaw();
            sleep(1000);
            initPosition();
            sleep(5000);
            gyroTurn(90);
            move(6000,0,0,0.7,500);
            move(8000,0,0,0.7,500);
        }
        else if (loc == 3){                //right
            move(250,0,0,0.4,500);
            gyroTurn(-40);
            grabPixel();
            sleep(3500);
            openLeftClaw();
            sleep(1000);
            initPosition();
            sleep(5000);
            gyroTurn(90);
            move(6000,0,0,0.7,500);
            move(6000,0,0,0.7,500);
        }

        telemetry.update();

    }

    @Override
    public void runOpMode() {

        initTfod();
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
        clawRight = hardwareMap.get(Servo.class, "Es2");
        clawCenter = hardwareMap.get(Servo.class, "Es0");

        initPosition();

        waitForStart();

        if (opModeIsActive()) {
            int r = detecLocation();
            caseLoc(r);

            //sleep(15000);
        }

        if (tfod != null) {
            visionPortal.close();
            tfod.shutdown();
        }
    } 

    //To open left claws
    private void openLeftClaw() {
        clawLeft.setPosition(0.2);
        telemetry.addData("Status", "Open Left Claw");
        telemetry.update();
    }
    
    //To close left claws
    private void closeLeftClaw() {
        clawLeft.setPosition(0);
        telemetry.addData("Status", "Close Left Claw");
        telemetry.update();
    }
    
    //To open right claws
    private void openRightClaw() {
        clawRight.setPosition(0);
        telemetry.addData("Status", "Open Right Claw");
        telemetry.update();
    }
    
    //To close right claws
    private void closeRightClaw() {
        clawRight.setPosition(0.2);
        telemetry.addData("Status", "Close Right Claw");
        telemetry.update();
    }
    
    //To grab the pixel, move both primary and secondary single bars to their positions
    private void grabPixel() {
        target2 = 975;
        target0 = 0 - target2;
        target1 = 900;
        liftmotor0.setTargetPosition(target0);
        liftmotor1.setTargetPosition(target1);
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
        //clawCenter.setPosition(0.45);
        clawCenter.setPosition(tclawCenter);
        closeLeftClaw();
        closeRightClaw();

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
        
        clawCenter.setPosition(0.45);
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

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            .setIsModelTensorFlow2(true)
            .setIsModelQuantized(true)
            .setModelInputSize(300)
            .setModelAspectRatio(16.0 / 9.0)
            .build();
            
            tfod.setZoom(1.5);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.40f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    private int detecLocation() {
        
        int iTimeOut = 10;
        int j = 0;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
		telemetry.update();
		
        while (opModeIsActive() && j < iTimeOut) {
            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                //telemetry.addData(""," ");
                //telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                //telemetry.addData("- Position", "x=%.0f / y=%.0f", x, y);
                //telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                
                if (x < 182)
                {
                    telemetry.addData("I am at", "Left");
					telemetry.update();
                    return 1;
                }
                else if (x > 344)
                {
                    telemetry.addData("I am at", "Right");
					telemetry.update();
                    return 3;
                }
                else
                {
                    telemetry.addData("I am at", "Center");
					telemetry.update();
                    return 2;
                }
                
                
            }   // end for() loop
            
            sleep(500);
            j++;
        }
        
        telemetry.addData("I can't find team prop so choose default location", "Left");
		telemetry.update();
        return 1;

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
