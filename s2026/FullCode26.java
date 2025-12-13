package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private int targetL = 0;
    private int targetR = 0;

    private int servoStep = 0;
    private boolean dpadLeftPrev = false; // tracks button press state

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

        if (gamepad2.right_trigger > 0.1) {
            intakeWheelL.setPower(intakePower * 0.032);
            intakeWheelR.setPower(-intakePower * 0.032);
        } else if (gamepad2.left_trigger > 0.1) {
            intakeWheelL.setPower(-intakePower * 0.028);
            intakeWheelR.setPower(intakePower * 0.028);
        } else {
            intakeWheelL.setPower(0);
            intakeWheelR.setPower(0);
        }

        // lift up
        if (gamepad2.dpad_up) {
            targetL = -300;
            targetR = 300;
            intakeLiftL.setTargetPosition(targetL);
            intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftR.setTargetPosition(targetR);
            intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftL.setPower(0.4);
            intakeLiftR.setPower(0.4);
        }

        // lift down
        if (gamepad2.dpad_down) {
            targetL = -70;
            targetR = 50;
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
        // === 3-step servo rotation ===
        boolean dpadLeftPressed = gamepad2.dpad_left || gamepad2.dpad_right;

        if (dpadLeftPressed && !dpadLeftPrev) {
            servoStep++; // move to next position
            if (servoStep > 2) servoStep = 0; // loop back after third

            // three tuned positions (adjust as needed)
            switch (servoStep) {
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

        // drive system
        double powerLeftF, powerRightF, powerLeftR, powerRightR;

        if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;

            leftWheelF.setPower(-powerLeftF * 0.7);
            leftWheelR.setPower(-powerLeftR * 0.7);
            rightWheelF.setPower(powerRightF * 0.7);
            rightWheelR.setPower(powerRightR * 0.7);
        } else {
            powerLeftF = drive2 + strafe2 + rotateRight;
            powerLeftR = drive2 - strafe2 + rotateRight;
            powerRightF = drive2 - strafe2 - rotateLeft;
            powerRightR = drive2 + strafe2 - rotateLeft;

            leftWheelF.setPower(-powerLeftF * 0.5);
            leftWheelR.setPower(-powerLeftR * 0.5);
            rightWheelF.setPower(powerRightF * 0.5);
            rightWheelR.setPower(powerRightR * 0.5);
        }

        telemetry.addData("Servo Pos", rotateServo.getPosition());
        telemetry.update();
    }
}
