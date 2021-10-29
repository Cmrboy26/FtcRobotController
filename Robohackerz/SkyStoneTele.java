package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SkyStoneTele" , group = "TOComp")

public class SkyStoneTele extends OpMode {

    ImportantStuff robot = new ImportantStuff();

    static final double INCREMENT   = .3;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = 0;     // Maximum REV power applied to motor

    double  power   = 0;
    boolean rampUp  = true;

//    DcMotor liftUp;
//    DcMotor liftOut;

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry);

//        liftUp = hardwareMap.dcMotor.get("liftUp");
//        liftOut = hardwareMap.dcMotor.get("liftOut");

//        liftUp.setMode(RUN_WITHOUT_ENCODER);
//        liftOut.setMode(RUN_WITHOUT_ENCODER);

//        liftUp.setDirection(FORWARD);
//        liftOut.setDirection(FORWARD);
    }

    @Override
    public void loop() {
        float speed = -gamepad1.right_stick_y;
        float direction = gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;

        float Magnitude = Math.abs(speed) + Math.abs(direction) + Math.abs(strafe);
        if (Magnitude < 1) {
            Magnitude = 1;
        }
        power -= INCREMENT;

        robot.robotStuff.FLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -.5, .5));
        robot.robotStuff.FRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -.5, .5));
        robot.robotStuff.BLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -.5, .5));
        robot.robotStuff.BRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -.5, .5));

        if (gamepad1.right_trigger > .3){
            robot.robotStuff.FLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -.25, .25));
            robot.robotStuff.FRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -.25, .25));
            robot.robotStuff.BLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -.25, .25));
            robot.robotStuff.BRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -.25, .25));
        }


        if (gamepad1.left_bumper){
            robot.robotStuff.leftHook.setPosition(robot.robotStuff.SERVO_LATCH_DOWN); //vertical
            robot.robotStuff.rightHook.setPosition(robot.robotStuff.SERVO_LATCH_UP);
        }
        else if (gamepad1.right_bumper){
            robot.robotStuff.leftHook.setPosition(robot.robotStuff.SERVO_LATCH_UP); //horizontal
            robot.robotStuff.rightHook.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
        }

//        if (gamepad1.right_bumper && robot.robotStuff.leftHook.getPosition() == robot.robotStuff.SERVO_LATCH_DOWN){
//            robot.robotStuff.leftHook.setPosition(robot.robotStuff.SERVO_LATCH_UP);
//            robot.robotStuff.rightHook.setPosition(robot.robotStuff.SERVO_LATCH_UP);
//        }
//        else if (gamepad1.right_bumper && robot.robotStuff.rightHook.getPosition() == robot.robotStuff.SERVO_LATCH_UP){
//            robot.robotStuff.leftHook.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
//            robot.robotStuff.rightHook.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
//        }

        //DRIVER 2
             //INTAKE SYSTEM
        if (gamepad2.right_trigger > .1) {
            robot.robotStuff.suck();
        }
        else if (gamepad2.left_trigger > .1) {
            robot.robotStuff.spit();
        }
        else {
            robot.robotStuff.LIntake.setPower(0);
            robot.robotStuff.RIntake.setPower(0);
        }

            //GRABBING STONES
        if (gamepad2.right_bumper) {
            robot.robotStuff.grabServo.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
        }
        else if (gamepad2.left_bumper) {
            robot.robotStuff.grabServo.setPosition(robot.robotStuff.SERVO_LATCH_UP);
        }


        //LIFT CONTROLS USING JOYSTICK
        float upLift = -gamepad2.right_stick_y;
        float outLift = -gamepad2.left_stick_x;

//        float MagnitudeLift = Math.abs(upLift) + Math.abs(outLift);
//        if (MagnitudeLift < 1) {
//            MagnitudeLift = 1;
//        }
        power -= INCREMENT;

        robot.robotStuff.liftOut.setPower(outLift);


        //MOVING LIFT OUT
//        if (gamepad2.x) {
//            robot.robotStuff.liftOut.setPower(1);
//        }
//        else if (gamepad2.b){
//            robot.robotStuff.liftOut.setPower(-1);
//        }
//        else {
//            robot.robotStuff.liftOut.setPower(0);
//        }
//


              //MOVING LIFT UP
        if (robot.robotStuff.sensorRange.getDistance(DistanceUnit.CM) < 9){
            robot.robotStuff.liftUp.setPower(.3);
        }
        else  {
            robot.robotStuff.liftUp.setPower(upLift);
        }


        if (gamepad2.dpad_up && robot.robotStuff.sensorRange.getDistance(DistanceUnit.CM) < 12.8){
            robot.robotStuff.liftUp.setPower(1);
        }

        telemetry.addData("RightHook Position", robot.robotStuff.rightHook.getPosition());
        telemetry.addData("LeftHook Position", robot.robotStuff.leftHook.getPosition());

        telemetry.addData("distance in inches", robot.robotStuff.sensorRange.getDistance(DistanceUnit.INCH));
        telemetry.addData("distance in centimeters", robot.robotStuff.sensorRange.getDistance(DistanceUnit.CM));


        telemetry.addLine(String.valueOf(robot.robotStuff.liftOut.getCurrentPosition()));
    }
}
