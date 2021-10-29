package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;

public class SkystoneBot {
    private Telemetry telemetry = null;
    //Drive
    public DcMotor BLeft = null;
    public DcMotor BRight = null;
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor RIntake = null;
    public DcMotor LIntake = null;
    public DcMotor liftUp = null;
    public DcMotor liftOut = null;

    public Servo leftHook;
    public Servo rightHook;
    public Servo grabServo;
    public Servo spinServo;

    public DistanceSensor sensorRange;
    public ColorSensor colorSensor;

    private DcMotor.Direction direction = null;

    private final double STOP = 0;

    public float SERVO_LATCH_UP = (float) 1.0;
    public float SERVO_LATCH_DOWN = (float) 0.0;

    public void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction direction) {
        this.telemetry = telemetryIn;
        BLeft = hwmap.dcMotor.get("BLeft");
        BRight = hwmap.dcMotor.get("BRight");
        FLeft = hwmap.dcMotor.get("FLeft");
        FRight = hwmap.dcMotor.get("FRight");
        RIntake = hwmap.dcMotor.get("RIntake");
        LIntake = hwmap.dcMotor.get("LIntake");
        liftUp = hwmap.dcMotor.get("liftUp");
        liftOut = hwmap.dcMotor.get("liftOut");

        leftHook = hwmap.servo.get("leftHook");
        rightHook = hwmap.servo.get("rightHook");
        grabServo = hwmap.servo.get("grabServo");
        spinServo = hwmap.servo.get("spinServo");

        sensorRange = hwmap.get(DistanceSensor.class, "sensorRange");
        colorSensor = hwmap.get(ColorSensor.class, "colorSensor");

        leftHook.scaleRange(.1, .63);
        rightHook.scaleRange(.17, .69);
        grabServo.scaleRange(.36, .47);
        spinServo.scaleRange(.25, .73);

        BLeft.setPower(STOP);
        FLeft.setPower(STOP);
        BRight.setPower(STOP);
        FRight.setPower(STOP);
        RIntake.setPower(STOP);
        LIntake.setPower(STOP);
        BLeft.setDirection(REVERSE);
        BRight.setDirection(FORWARD);
        FLeft.setDirection(REVERSE);
        FRight.setDirection(FORWARD);
        RIntake.setDirection(FORWARD);
        LIntake.setDirection(REVERSE);
        liftUp.setDirection(REVERSE);
        liftOut.setDirection(FORWARD);

        grabServo.setPosition(SERVO_LATCH_UP);
        spinServo.setPosition(SERVO_LATCH_UP);

//        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {
//        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RobotStuff (float speed, float direction, float strafe) {
        float Magnitude = Math.abs(speed) + Math.abs(direction) + Math.abs(strafe);
        if (Magnitude < 1) {
            Magnitude = 1;
        }
        FLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -1, 1));
        FRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -1, 1));
        BLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -1, 1));
        BRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -1, 1));
    }

    public void suck() {
        RIntake.setPower(.5);
        LIntake.setPower(.5);
    }

    public void spit() {
        RIntake.setPower(-.5);
        LIntake.setPower(-.5);
    }

    public void stop() {
        BLeft.setPower(STOP);
        FLeft.setPower(STOP);
        BRight.setPower(STOP);
        FRight.setPower(STOP);
        RIntake.setPower(STOP);
        LIntake.setPower(STOP);
        liftUp.setPower(STOP);
        liftOut.setPower(STOP);
    }

    }