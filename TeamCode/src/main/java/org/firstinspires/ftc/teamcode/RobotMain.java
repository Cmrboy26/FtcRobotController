package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;

public class RobotMain {
    private Telemetry telemetry = null;

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
    public Servo spinServo;

    private DcMotor.Direction direction = null;

    public void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction direction) {
        this.telemetry = telemetryIn;
        BLeft = hwmap.dcMotor.get("BLeft");
        BRight = hwmap.dcMotor.get("BRight");
        FLeft = hwmap.dcMotor.get("FLeft");
        FRight = hwmap.dcMotor.get("FRight");
        RIntake = hwmap.dcMotor.get("RIntake");
        LIntake = hwmap.dcMotor.get("LIntake");

        leftHook = hwmap.servo.get("leftHook");
        rightHook = hwmap.servo.get("rightHook");
        grabServo = hwmap.servo.get("grabServo");
        spinServo = hwmap.servo.get("spinServo");
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
        RIntake.setPower(0);
        LIntake.setPower(0);
    }
}
