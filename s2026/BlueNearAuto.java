package org.firstinspires.ftc;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BlueNearAuto", group = "Opmode ProfoundPython")
public class BlueNearAuto extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftWheelF = null;     // Left Front
    private DcMotor leftWheelR = null;     // Left Rear
    private DcMotor rightWheelF = null;    // Right Front
    private DcMotor rightWheelR = null;     // Right Rear
    private DcMotor intakeLiftL = null;
    private DcMotor intakeLiftR = null;
    private DcMotor intakeWheelL = null;
    private DcMotor intakeWheelR = null;
    private Servo rotateServo = null;
    private Servo centerServo = null;
    private Servo pushL = null;
    private Servo pushR = null;

    
    private IMU imu;
    private double TURN_P = 0.010;
    
    private WebcamName webcamName;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    private int targetL = 0;
    private int targetR = 0;
    
    private int servoStep = 0; // starts at position 0
    private double intakePower = 1.0; // default intake wheel power multiplier


    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
        move(9000,0,0,0.9,500);
        move(9000,0,0,0.7,500);
        move(9000,0,0,0.6,500);
        move(1000,0,0,0.7,500);
        gyroTurnRelative(90);
        gyroTurnRelative(98);
        intakeLiftUp();
        intakeOut();
        sleep(1000);
        blockerOut();
        sleep(900);
        pushOut();
        sleep(250);
        pushBack();
        sleep(500);
        rotateServoNext(); // move to second position
        sleep(700);
        pushOut();
        sleep(250);
        pushBack();
        sleep(500);
        rotateServoNext(); // move to third position
        sleep(700);
        pushOut();
        sleep(250);
        pushBack();
        sleep(500);
        rotateServoNext(); // move to third position
        sleep(700);
        pushOut();
        sleep(250);
        pushBack();
        sleep(500);
        intakeLiftDown();
        intakeIn();
        gyroTurnRelative(40);
        move(8000,0,0,0.9,500);
        move(8000,0,0,0.9,500);
        sleep(1000);
        move(-7000,0,0,1,500);
        sleep(1000);
        gyroTurnRelative(-50);
        intakeLiftUp();
        sleep(1000);
        intakeOut();
        sleep(1000);
        pushOut();
        sleep(250);
        pushBack();
        sleep(700);
        rotateServoNext(); // move to second position
        sleep(700);
        pushOut();
        sleep(250);
        pushBack();
        sleep(700);
        rotateServoNext(); // move to third position
        sleep(700);
        pushOut();
        sleep(250);
        pushBack();
        sleep(500);
        rotateServoNext(); // move to third position
        sleep(700);
        pushOut();
        sleep(250);
        pushBack();
        sleep(500);
        
        }
    }

    /** Initialize all hardware */
    private void initHardware() {
        imuInit(); 
        leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");

        intakeLiftL = hardwareMap.get(DcMotorEx.class, "intakeLiftL");
        intakeLiftR = hardwareMap.get(DcMotorEx.class, "intakeLiftR");
        intakeWheelL = hardwareMap.get(DcMotor.class, "intakeWheelL");
        intakeWheelR = hardwareMap.get(DcMotor.class, "intakeWheelR");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        centerServo = hardwareMap.get(Servo.class, "centerServo");     // Es0
        pushL = hardwareMap.get(Servo.class, "pushL");                   // s1
        pushR = hardwareMap.get(Servo.class, "pushR");                   // s2

        initPosition();
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

    private void initPosition() {
        // Set starting lift target positions
        targetL = -230;
        targetR = 260;

        intakeLiftL.setTargetPosition(targetL);
        intakeLiftR.setTargetPosition(targetR);

        intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Low power just to hold initial position
        intakeLiftL.setPower(0.15);
        intakeLiftR.setPower(0.15);
        centerServo.setPosition(0.1);
        rotateServo.setPosition(0.0);
        pushL.setPosition(0.7);
        pushR.setPosition(0);

        telemetry.addData("Status", "Initial Position Set");
        telemetry.update();
    }
    
    private void blockerIn() {
        centerServo.setPosition(0.1);
    }
    private void blockerOut() {
        centerServo.setPosition(1);
    }
    
    
    /** Initialize AprilTag detection */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam1"));
        } else {
            builder.setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    /** Reads AprilTag ID and performs path actions */
    /** Reads AprilTag ID and performs path actions */
private void caseLoc() {

    int tagID = getAprilTagID();  // Clear variable storing current detected ID

    boolean tag21 = false;
    boolean tag22 = false;
    boolean tag23 = false;

    // Logic to set the correct boolean true
    if (tagID == 21) {
        tag21 = true;
        telemetry.addLine("Performing path for Tag 21");
        gyroTurnRelative(65); 
        sleep(1000);
        intakeLiftUp();
        intakeOut();
        sleep(1500);
        move(-1000,500,0,0.6,500);
        move(500,500,0,0.6,500);
        move(400,0,0,0.5,500);
        sleep(1000);
        rotateServoPrev(); // move to first position
        sleep(1000);
        blockerOut();
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
        sleep(700);
        rotateServoPrev(); // move to second position
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
        sleep(700);
        rotateServoPrev(); // move to third position
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
    } else if (tagID == 22) {
        tag22 = true;
        telemetry.addLine("Performing path for Tag 22");
        gyroTurnRelative(65); 
        intakeLiftUp();
        intakeOut();
        sleep(1000);
        blockerOut();
        move(-1000,500,0,0.6,500);
        move(500,500,0,0.6,500);
        move(400,0,0,0.5,500);
        sleep(500);
        pushOut();
        sleep(1000);
        pushBack();
        //sleep(1000);
        //rotateServoPrev();// move to first position
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
        sleep(700);
        rotateServoPrev(); // move to second position
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
        sleep(700);
    } else if (tagID == 23) {
        tag23 = true;
        telemetry.addLine("Performing path for Tag 23");
        gyroTurnRelative(65); 
        sleep(1000);
        intakeLiftUp();
        intakeOut();
        sleep(2000);
        blockerOut();
        move(-1000,500,0,0.6,500);
        move(500,500,0,0.6,500);
        move(400,0,0,0.5,500);
        sleep(500);
        pushOut();
        sleep(1000);
        pushBack();
        rotateServoNext(); // move to first position
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
        sleep(700);
        rotateServoNext(); // move to second position
        sleep(1000);
        pushOut();
        sleep(1000);
        pushBack();
        sleep(700);
    }
     else {
        telemetry.addLine("No valid AprilTag detected.");
    }

    // Telemetry feedback
    telemetry.addData("Detected Tag ID", tagID);
    telemetry.addData("Tag21", tag21);
    telemetry.addData("Tag22", tag22);
    telemetry.addData("Tag23", tag23);

    telemetry.update();
    sleep(1000);  // small delay to prevent telemetry spam
}


/** Return AprilTag ID, or -1 if none */
private int getAprilTagID() {
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    telemetry.addData("# AprilTags Detected", currentDetections.size());

    int detectedID = -1;

    for (AprilTagDetection detection : currentDetections) {
        detectedID = detection.id;  // Store the ID of the detected tag

        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
        }
    }

    return detectedID;  // Returns -1 if none detected
}

    /** Movement helper method */
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
        /** Moves servo through 3-step sequence automatically */
    private void rotateServoNext() {
        servoStep++;
        if (servoStep > 2) servoStep = 0;

        switch (servoStep) {
            case 0:
                rotateServo.setPosition(0.0);
                telemetry.addData("Rotate Servo", "Position 0 (0.0)");
                break;
            case 1:
                rotateServo.setPosition(0.45);
                telemetry.addData("Rotate Servo", "Position 1 (0.43)");
                break;
            case 2:
                rotateServo.setPosition(0.94);
                telemetry.addData("Rotate Servo", "Position 2 (0.92)");
                break;
        }

        telemetry.update();
        sleep(500); // small delay to let servo move physically
    }
    
    private void rotateServoPrev() {
    servoStep--;
    if (servoStep < 0) servoStep = 2;

    switch (servoStep) {
        case 2:
            rotateServo.setPosition(0.94);
            telemetry.addData("Rotate Servo", "Position 2 (0.92)");
            break;
        case 1:
            rotateServo.setPosition(0.45);
            telemetry.addData("Rotate Servo", "Position 1 (0.43)");
            break;
        case 0:
            rotateServo.setPosition(0.0);
            telemetry.addData("Rotate Servo", "Position 0 (0.0)");
            break;
    }

        telemetry.update();
        sleep(500);
    }
    
        /** Spins intake wheels inward (to collect) */
    private void intakeIn() {
        intakeWheelL.setPower(-intakePower * 0.5);
        intakeWheelR.setPower(intakePower * 0.5);
    }

    /** Spins intake wheels outward (to eject) */
    private void intakeOut() {
        intakeWheelL.setPower(intakePower * 0.5);
        intakeWheelR.setPower(-intakePower * 0.5);
    }

    /** Lifts intake up */
    private void intakeLiftUp() {
        targetL = -280;
        targetR = 280;

        intakeLiftL.setTargetPosition(targetL);
        intakeLiftR.setTargetPosition(targetR);

        intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeLiftL.setPower(0.4);
        intakeLiftR.setPower(0.4);
    }

    /** Lowers intake down */
    private void intakeLiftDown() {
        targetL = -80;
        targetR = 60;

        intakeLiftL.setTargetPosition(targetL);
        intakeLiftR.setTargetPosition(targetR);

        intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeLiftL.setPower(0.2);
        intakeLiftR.setPower(0.2);
    }
    
    private void pushOut(){   
        pushL.setPosition(0.3);
        pushR.setPosition(0.5);
        telemetry.addData("Status: ", "Close both claws");
        telemetry.update();
    }
    
    private void pushBack(){   
        pushL.setPosition(0.8);
        pushR.setPosition(0);
        telemetry.addData("Status: ", "Open both claws");
        telemetry.update();
    }
    
    private void gyroTurnRelative(double relativeAngle) {
        double currentHeading = getHeading();

        double targetAngle = currentHeading + relativeAngle;

        if (targetAngle > 180)  targetAngle -= 360;
        if (targetAngle < -180) targetAngle += 360;

        gyroTurn(targetAngle);
    }


    
        public double getHeading() {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);
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

        while (i < iMAX && opModeIsActive() && delta > 1.0) // 1 deg tolerance
                                                            {
        double error_degrees = target_angle - currentHeading;

        // Normalize to shortest path
        if (error_degrees > 180)  error_degrees -= 360;
        if (error_degrees < -180) error_degrees += 360;

        double motor_output = Range.clip(error_degrees * TURN_P, -0.6, 0.6);

        leftWheelF.setPower(motor_output);
        leftWheelR.setPower(motor_output);
        rightWheelF.setPower(motor_output);
        rightWheelR.setPower(motor_output);

        currentHeading = getHeading();
        delta = Math.abs(error_degrees);

        telemetry.addData("Target", target_angle);
        telemetry.addData("Heading", currentHeading);
        telemetry.addData("Error", error_degrees);
        telemetry.addData("Power", motor_output);
        telemetry.update();

        i++;
    }


        //sleep(500);

        leftWheelF.setPower(0);
        leftWheelR.setPower(0);
        rightWheelF.setPower(0);
        rightWheelR.setPower(0);

    }
}
