package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;

/**
 * Basically code that sets everything up
 */

public class RobotMain {

    private Telemetry telemetry = null;

    /*
    Newest Robot Motors & Servos

    public DcMotor BLeft = null;
    public DcMotor BRight = null;
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor RIntake = null; // right wheel
    public DcMotor LIntake = null; // left wheel
    public DcMotor liftUp = null;
    public DcMotor liftOut = null;

    public Servo leftHook;
    public Servo rightHook;
    public Servo grabServo;
    public Servo spinServo;*/

    public DcMotor BLeft = null;
    public DcMotor BRight = null;
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    public DcMotor Latch = null;
    public DcMotor SweeperExtends;
    public DcMotor MineralRaise;

    public CRServo Sweeper;
    public Servo SweeperUp;
    public Servo SweeperDown;
    public Servo MarkerDump;
    public Servo MineralDump;

    //Servo Variables
    public float SERVO_LATCH_UP = 1f;
    public float SERVO_LATCH_DOWN = 0f;

    private DcMotor.Direction direction = null;

    //BNO055IMU imu;


    public void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction direction) {
        this.telemetry = telemetryIn;
        BLeft = hwmap.dcMotor.get("BLeft");
        BRight = hwmap.dcMotor.get("BRight");
        FLeft = hwmap.dcMotor.get("FLeft");
        FRight = hwmap.dcMotor.get("FRight");
        Latch = hwmap.dcMotor.get("Latch");
        SweeperExtends = hwmap.dcMotor.get("SweeperExtends");
        MineralRaise = hwmap.dcMotor.get("MineralRaise");

        Sweeper = hwmap.crservo.get("Sweeper");
        SweeperUp = hwmap.servo.get("SweeperUp");
        SweeperDown = hwmap.servo.get("SweeperDown");
        MarkerDump = hwmap.servo.get("MarkerDump");
        MineralDump = hwmap.servo.get("MineralDump");

        SweeperUp.scaleRange(0, .6);
        SweeperDown.scaleRange(.4, 1);
        MarkerDump.scaleRange(.30, .83);
        MineralDump.scaleRange(.08 , .66);

        BLeft.setPower(0);
        FLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        Latch.setPower(0);
        SweeperExtends.setPower(0);
        MineralRaise.setPower(0);
        Sweeper.setPower(0);

        SweeperUp.setPosition(SERVO_LATCH_UP);
        SweeperDown.setPosition(-SERVO_LATCH_UP);
        MarkerDump.setPosition(SERVO_LATCH_UP);
        MineralDump.setPosition(SERVO_LATCH_DOWN);

        // What direction the motors operate in
        BLeft.setDirection(REVERSE);
        BRight.setDirection(FORWARD);
        FLeft.setDirection(REVERSE);
        FRight.setDirection(FORWARD);
        Latch.setDirection(FORWARD);
        SweeperExtends.setDirection(FORWARD);
        MineralRaise.setDirection(FORWARD);

        // When the motor has zero power, have it break
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Latch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SweeperExtends.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MineralRaise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


    public void stopAll() {
        BLeft.setPower(0);
        BRight.setPower(0);
        FLeft.setPower(0);
        FRight.setPower(0);
        Sweeper.setPower(0);
        Latch.setPower(0);
        SweeperExtends.setPower(0);
        MineralRaise.setPower(0);
    }
}
