package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;

public class EightNineRobot {
    private Telemetry telemetry = null;
    //Drive Motors
    public DcMotor BLeft = null;
    public DcMotor BRight = null;
    public DcMotor FLeft = null;
    public DcMotor FRight = null;
    //Latch Motors
    public DcMotor Latch = null;
    //Sweeper Power
    public DcMotor SweeperExtends;
    //Mineral Dump
    public DcMotor MineralRaise;

    //Sweep in servo
    public CRServo Sweeper;
    //Set Position servos for sweeper. Inverse up and down
    public Servo SweeperUp;
    public Servo SweeperDown;
    //Set Position Dump Servo
    public Servo MarkerDump;
    //Set Position Dump Minerals
    public Servo MineralDump;

    //Variables
    public float SERVO_LATCH_UP = (float) 1.0;
    public float SERVO_LATCH_DOWN = (float) 0.0;

    BNO055IMU imu;

    // State used for updating telemetry
    Acceleration gravity;

    Orientation angles;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;

    double directionGyro;

    Orientation roboterror;
    double targetangle;

    public EightNineRobot() {
    }

    public void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction direction) {
        this.telemetry = telemetryIn;
        //Motors
        BLeft = hwmap.dcMotor.get("BLeft");
        BRight = hwmap.dcMotor.get("BRight");
        FLeft = hwmap.dcMotor.get("FLeft");
        FRight = hwmap.dcMotor.get("FRight");

        Latch = hwmap.dcMotor.get("Latch");

        SweeperExtends = hwmap.dcMotor.get("SweeperExtends");

        MineralRaise = hwmap.dcMotor.get("MineralRaise");

        //Servos
        Sweeper = hwmap.crservo.get("Sweeper");

        SweeperUp = hwmap.servo.get("SweeperUp");
        SweeperDown = hwmap.servo.get("SweeperDown");

        MarkerDump = hwmap.servo.get("MarkerDump");

        MineralDump = hwmap.servo.get("MineralDump");

        SweeperUp.scaleRange(0, .6);
        SweeperDown.scaleRange(.4, 1);
        MarkerDump.scaleRange(.30, .83);
        MineralDump.scaleRange(.08 , .66);

        //Motors
        BLeft.setPower(0);
        FLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        Latch.setPower(0);
        SweeperExtends.setPower(0);
        MineralRaise.setPower(0);
        //Servos
        Sweeper.setPower(0);

        SweeperUp.setPosition(SERVO_LATCH_UP);
        SweeperDown.setPosition(-SERVO_LATCH_UP);
        MarkerDump.setPosition(SERVO_LATCH_UP);
        MineralDump.setPosition(SERVO_LATCH_DOWN);

        BLeft.setDirection(REVERSE);
        BRight.setDirection(FORWARD);
        FLeft.setDirection(REVERSE);
        FRight.setDirection(FORWARD);
        Latch.setDirection(FORWARD);
        SweeperExtends.setDirection(FORWARD);
        MineralRaise.setDirection(FORWARD);

        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Latch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SweeperExtends.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MineralRaise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Latch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Latch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SweeperExtends.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SweeperExtends.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MineralRaise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MineralRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        roboterror = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

    }

    public void loop() {}

    public void start() {
    }

    public void EightNineRobot (float speed, float direction, float strafe) {
        float Magnitude = Math.abs(speed) + Math.abs(direction) + Math.abs(strafe);
        if (Magnitude < 1) {
            Magnitude = 1;
        }
        FLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -1, 1));
        FRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -1, 1));
        BLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -1, 1));
        BRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -1, 1));
    }

    public void stop() {
        BLeft.setPower(0);
        FLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        Sweeper.setPower(0);
        Latch.setPower(0);
        SweeperExtends.setPower(0);
        MineralRaise.setPower(0);
   }

}

