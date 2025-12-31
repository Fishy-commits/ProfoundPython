package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Intake Wheels at 50 RPM", group = "Examples")
public class FullCode26 extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftWheelF = null;
    private DcMotor leftWheelR = null;
    private DcMotor rightWheelF = null;
    private DcMotor rightWheelR = null;
    private DcMotor intakeLiftL = null;
    private DcMotor intakeLiftR = null;
    private Servo rotateServo = null;
    private Servo centerServo = null;
    private DcMotor intakeWheelL = null;
    private DcMotor intakeWheelR = null;
    private Servo pushL = null;
    private Servo pushR = null;
    //private DistanceSensor sensorRange;


    private int targetL = 0;
    private int targetR = 0;

    private int servoStepLeft = 0;
    private int servoStepRight = 0;

    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    
    //private static final double TARGET_DISTANCE_IN = 24.0; // perfect shooting distance
    //private static final double DIST_TOLERANCE = 1.0;       // ±1 inch
    //private static final double ALIGN_SPEED = 0.25;         // slow & controlled


    @Override
    public void init() {
        leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");

        intakeLiftL = hardwareMap.get(DcMotorEx.class, "intakeLiftL"); // Em1
        intakeLiftR = hardwareMap.get(DcMotorEx.class, "intakeLiftR"); // Em0
        intakeWheelL = hardwareMap.get(DcMotor.class, "intakeWheelL"); // Em2
        intakeWheelR = hardwareMap.get(DcMotor.class, "intakeWheelR"); // Em3
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");     // S0
        pushL = hardwareMap.get(Servo.class, "pushL");                   // s1
        pushR = hardwareMap.get(Servo.class, "pushR");                   // s2
        centerServo = hardwareMap.get(Servo.class, "centerServo");     // Es0
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");


        initPosition();
    }

    @Override
    public void loop() {
        move();
    }

    @Override
    public void start() {
        runtime.reset();
    }
    private void blockerIn() {
        centerServo.setPosition(0.1);
    }
    private void blockerOut() {
        centerServo.setPosition(1);
    }
    
    private void pushOut(){
        pushL.setPosition(0.3);
        pushR.setPosition(0.4);
        telemetry.addData("Status: ", "Close both claws");
        telemetry.update();
    }
    

    private void pushBack() {
        pushL.setPosition(0.7);
        pushR.setPosition(0);
        telemetry.addData("Status: ", "Open both claws");
        telemetry.update();
    }
    /*    private void autoAlignToGoal() {
        double distance = sensorRange.getDistance(DistanceUnit.INCH);

        // SAFETY FIRST
        if (!Double.isFinite(distance)) {
            setDrivePower(0);
            telemetry.addData("AutoAlign", "Invalid distance");
            return;
        }

        if (distance > TARGET_DISTANCE_IN + DIST_TOLERANCE) {
            setDrivePower(ALIGN_SPEED);
        } 
        else if (distance < TARGET_DISTANCE_IN - DIST_TOLERANCE) {
            setDrivePower(-ALIGN_SPEED);
        } 
        else {
            setDrivePower(0);
        }

        telemetry.addData("AutoAlign Distance", distance);
    }
    private void setDrivePower(double power) {
        leftWheelF.setPower(-power);
        leftWheelR.setPower(-power);
        rightWheelF.setPower(power);
        rightWheelR.setPower(power);
} */



    private void initPosition() {
        targetL = -325;
        targetR = 325;
        intakeLiftL.setTargetPosition(targetL);
        intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLiftR.setTargetPosition(targetR);
        intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pushL.setPosition(0.7);
        pushR.setPosition(0);
        centerServo.setPosition(1.0);
        rotateServo.setPosition(0.0);

        intakeLiftL.setPower(0.1);
        intakeLiftR.setPower(0.1);

        telemetry.addData("Status", "Initial Position");
        telemetry.update();
    }

    private void move() {
        
        // AUTO ALIGN (highest priority)
     /*   if (gamepad1.x) {
            autoAlignToGoal();
            telemetry.update();
            return;
        }
*/
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotateLeft = gamepad1.left_trigger;
        double rotateRight = gamepad1.right_trigger;

        double drive2 = -gamepad1.right_stick_y;
        double strafe2 = gamepad1.right_stick_x;
        double intakePower = 20;

        // intake wheels control
        intakeWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double normalOuttakePower = intakePower * 0.0325;    //far shots
        double closeOuttakePower  = intakePower * 0.07;   // close shots
        double intakeInPower      = intakePower * 0.028;

        // Priority order matters
        if (gamepad2.right_trigger > 0.1) { //far shots
            // Normal outtake
            intakeWheelL.setPower(normalOuttakePower);
            intakeWheelR.setPower(-normalOuttakePower);

        } else if (gamepad2.left_trigger > 0.1) { //close shots
            // Close-range outtake
            intakeWheelL.setPower(closeOuttakePower);
            intakeWheelR.setPower(-closeOuttakePower);

        } else if (gamepad2.x) {
            // Intake (button instead of trigger)
            intakeWheelL.setPower(-intakeInPower);
            intakeWheelR.setPower(intakeInPower);

        } else {
            intakeWheelL.setPower(0);
            intakeWheelR.setPower(0);
        }


        // lift up
        if (gamepad2.dpad_up) {
            targetL = -320;
            targetR = 320;
            intakeLiftL.setTargetPosition(targetL);
            intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftR.setTargetPosition(targetR);
            intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftL.setPower(0.4);
            intakeLiftR.setPower(0.4);
        }

        // lift down
        if (gamepad2.dpad_down) {
            targetL = -100;
            targetR = 80;
            intakeLiftL.setTargetPosition(targetL);
            intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftR.setTargetPosition(targetR);
            intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftL.setPower(0.2);
            intakeLiftR.setPower(0.2);
        }
        
         else if (gamepad2.a) {
            blockerIn();
        }
         else if (gamepad2.b) {
            blockerOut();
        }

        
        else if (gamepad2.right_bumper) {    //when the right_bumper of gamepad 2 is pressed, claws are closed
            pushOut();

        }
        else if (gamepad2.left_bumper) {    //when the left_bumper of gamepad2 is pressed, claws are opened
            pushBack();
        
        }
        // === 3-step servo rotation (LEFT) ===
        boolean dpadLeftPressed = gamepad2.dpad_left;

        if (dpadLeftPressed && !dpadLeftPrev) {
            servoStepLeft++;
            if (servoStepLeft > 2) servoStepLeft = 0;

            switch (servoStepLeft) {
                case 0:
                    rotateServo.setPosition(0.0);
                    break;
                case 1:
                    rotateServo.setPosition(0.45);
                    break;
                case 2:
                    rotateServo.setPosition(0.94);
                    break;
            }
        }
        dpadLeftPrev = dpadLeftPressed;


        // === 3-step servo rotation (RIGHT – DIFFERENT POSITIONS) ===
        boolean dpadRightPressed = gamepad2.dpad_right;

        if (dpadRightPressed && !dpadRightPrev) {
            servoStepRight++;
            if (servoStepRight > 2) servoStepRight = 0;

            switch (servoStepRight) {
                case 0:
                    rotateServo.setPosition(0.14);
                    break;
                case 1:
                    rotateServo.setPosition(0.61);
                    break;
                case 2:
                    rotateServo.setPosition(1);
                    break;
            }
        }
        dpadRightPrev = dpadRightPressed;


        // drive system
        double powerLeftF, powerRightF, powerLeftR, powerRightR;

        if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF * 1);
            leftWheelR.setPower(-powerLeftR * 1);
            rightWheelF.setPower(powerRightF * 1);
            rightWheelR.setPower(powerRightR * 1);
        } else {
            powerLeftF = drive2 + strafe2 + rotateRight;
            powerLeftR = drive2 - strafe2 + rotateRight;
            powerRightF = drive2 - strafe2 - rotateLeft;
            powerRightR = drive2 + strafe2 - rotateLeft;

            leftWheelF.setPower(-powerLeftF * 0.6);
            leftWheelR.setPower(-powerLeftR * 0.6);
            rightWheelF.setPower(powerRightF * 0.6);
            rightWheelR.setPower(powerRightR * 0.6);
        }

        telemetry.addData("Servo Pos", rotateServo.getPosition());
        telemetry.update();
    }
}
