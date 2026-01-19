package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Prism.Color;

@TeleOp(name = "FullCode26", group = "Examples")
public class FullCode26 extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // ===== Robot Hardware =====
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

    private int servoStepLeft = 0;
    private int servoStepRight = 0;

    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    // ===== Prism LED =====
    private GoBildaPrismDriver prism;
    private PrismAnimations.Solid solid;

    @Override
    public void init() {
        // --- Motors & Servos ---
        leftWheelF = hardwareMap.get(DcMotor.class, "M0");
        rightWheelF = hardwareMap.get(DcMotor.class, "M1");
        leftWheelR = hardwareMap.get(DcMotor.class, "M2");
        rightWheelR = hardwareMap.get(DcMotor.class, "M3");

        intakeLiftL = hardwareMap.get(DcMotor.class, "intakeLiftL");
        intakeLiftR = hardwareMap.get(DcMotor.class, "intakeLiftR");
        intakeWheelL = hardwareMap.get(DcMotor.class, "intakeWheelL");
        intakeWheelR = hardwareMap.get(DcMotor.class, "intakeWheelR");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        pushL = hardwareMap.get(Servo.class, "pushL");
        pushR = hardwareMap.get(Servo.class, "pushR");
        centerServo = hardwareMap.get(Servo.class, "centerServo");

        initPosition();

        // --- Prism LED setup (solid blue) ---
        prism = hardwareMap.get(GoBildaPrismDriver.class, "Prism"); // device name in robot config
        solid = new PrismAnimations.Solid();
        solid.setPrimaryColor(Color.BLUE);
        solid.setBrightness(100);       // 0-100%
        solid.setStartIndex(0);         // first LED
        solid.setStopIndex(11);         // last LED
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        move();  // robot controls
        // lights stay on as solid blue
    }

    @Override
    public void stop() {
        super.stop();

        // --- Turn off Prism LEDs ---
        if (prism != null) {
            PrismAnimations.Solid off = new PrismAnimations.Solid();
            off.setPrimaryColor(new Color (0,0,0));
            off.setBrightness(0);
            off.setStartIndex(0);
            off.setStopIndex(11);
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, off);
        }
    }

    // ===== Robot Methods =====
    private void initPosition() {
        targetL = -80;
        targetR = 60;
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
        // --- drive inputs ---
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotateLeft = gamepad1.left_trigger;
        double rotateRight = gamepad1.right_trigger;
        double drive2 = -gamepad1.right_stick_y;
        double strafe2 = gamepad1.right_stick_x;
        double intakePower = 20;

        // --- Intake wheels ---
        intakeWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double normalOuttakePower = intakePower * 0.0325;
        double farOuttakePower = intakePower * 0.035;
        double intakeInPower = intakePower * 0.04;

        if (gamepad2.right_trigger > 0.1) {
            intakeWheelL.setPower(normalOuttakePower);
            intakeWheelR.setPower(-normalOuttakePower);
        } else if (gamepad2.left_trigger > 0.1) {
            intakeWheelL.setPower(farOuttakePower);
            intakeWheelR.setPower(-farOuttakePower);
        } else if (gamepad2.x) {
            intakeWheelL.setPower(-intakeInPower);
            intakeWheelR.setPower(intakeInPower);
        } else {
            intakeWheelL.setPower(0);
            intakeWheelR.setPower(0);
        }

        // --- Lift controls ---
        if (gamepad2.dpad_up) {
            targetL = -240;
            targetR = 260;
            intakeLiftL.setTargetPosition(targetL);
            intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftR.setTargetPosition(targetR);
            intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftL.setPower(0.4);
            intakeLiftR.setPower(0.4);
        } else if (gamepad2.dpad_down) {
            targetL = -80;
            targetR = 60;
            intakeLiftL.setTargetPosition(targetL);
            intakeLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftR.setTargetPosition(targetR);
            intakeLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLiftL.setPower(0.2);
            intakeLiftR.setPower(0.2);
        } else if (gamepad2.a) {
            centerServo.setPosition(0.1);
        } else if (gamepad2.b) {
            centerServo.setPosition(1.0);
        } else if (gamepad2.right_bumper) {
            pushL.setPosition(0.3);
            pushR.setPosition(0.5);
        } else if (gamepad2.left_bumper) {
            pushL.setPosition(0.8);
            pushR.setPosition(0);
        }

        // --- 3-step servo rotation ---
        boolean dpadLeftPressed = gamepad2.dpad_left;
        if (dpadLeftPressed && !dpadLeftPrev) {
            servoStepLeft++;
            if (servoStepLeft > 2) servoStepLeft = 0;
            switch (servoStepLeft) {
                case 0: rotateServo.setPosition(0.0); break;
                case 1: rotateServo.setPosition(0.45); break;
                case 2: rotateServo.setPosition(0.94); break;
            }
        }
        dpadLeftPrev = dpadLeftPressed;

        boolean dpadRightPressed = gamepad2.dpad_right;
        if (dpadRightPressed && !dpadRightPrev) {
            servoStepRight++;
            if (servoStepRight > 2) servoStepRight = 0;
            switch (servoStepRight) {
                case 0: rotateServo.setPosition(0.14); break;
                case 1: rotateServo.setPosition(0.61); break;
                case 2: rotateServo.setPosition(1); break;
            }
        }
        dpadRightPrev = dpadRightPressed;

        // --- Drive system ---
        double powerLeftF, powerRightF, powerLeftR, powerRightR;
        if (drive != 0 || strafe != 0 || rotateRight != 0 || rotateLeft != 0) {
            powerLeftF = drive + strafe + rotateRight - rotateLeft;
            powerLeftR = drive - strafe + rotateRight - rotateLeft;
            powerRightF = drive - strafe - rotateRight + rotateLeft;
            powerRightR = drive + strafe - rotateRight + rotateLeft;
            leftWheelF.setPower(-powerLeftF);
            leftWheelR.setPower(-powerLeftR);
            rightWheelF.setPower(powerRightF);
            rightWheelR.setPower(powerRightR);
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
