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

    public void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction direction) {
        this.telemetry = telemetryIn;
        BLeft = hwmap.dcMotor.get("BLeft");
        BRight = hwmap.dcMotor.get("BRight");
        FLeft = hwmap.dcMotor.get("FLeft");
        FRight = hwmap.dcMotor.get("FRight");
        RIntake = hwmap.dcMotor.get("RIntake");
        LIntake = hwmap.dcMotor.get("LIntake");
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
