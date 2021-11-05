package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.util.Range.scale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RoverAlanbot" , group = "TOComp")


public class RoboController extends OpMode {
    ImportantStuff robot = new ImportantStuff();


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);


    }

    @Override
    public void loop() {
        //GAMEPAD 1 DRIVER 1
        //Drive
        float speed = -gamepad1.right_stick_y;
        float direction = gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;

        float Magnitude = Math.abs(speed) + Math.abs(direction) + Math.abs(strafe);
        if (Magnitude < 1) {
            Magnitude = 1;
        }
        robot.robotStuff.FLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -1, 1));
        robot.robotStuff.FRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -1, 1));
        robot.robotStuff.BLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -1, 1));
        robot.robotStuff.BRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -1, 1));

        //Mineral lift
        if (gamepad1.dpad_up){
            robot.robotStuff.MineralRaise.setPower(1);
        }
        else if (gamepad1.dpad_down){
            robot.robotStuff.MineralRaise.setPower(-1);
        }
        else {
            robot.robotStuff.MineralRaise.setPower(0);
        }

        //Mineral dump
        if (gamepad1.y){
            robot.robotStuff.MineralDump.setPosition(robot.robotStuff.SERVO_LATCH_UP);
        }
        else if (gamepad1.a){
            robot.robotStuff.MineralDump.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
        }

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~DRIVER BREAK~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        //GAMEPAD 2 DRIVER 2
        //Sweeping in
        if (gamepad2.right_trigger > .1) {
            robot.robotStuff.Sweeper.setPower(1);
        }
        else if (gamepad2.left_trigger > .1) {
            robot.robotStuff.Sweeper.setPower(-1);
        }
        else {
            robot.robotStuff.Sweeper.setPower(0);
        }

        //Sweeper Up and Down JOYSTICK

        if (gamepad2.left_bumper){
            robot.robotStuff.SweeperUp.setPosition(robot.robotStuff.SERVO_LATCH_UP);
            robot.robotStuff.SweeperDown.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
        }
        else if (gamepad2.right_bumper){
            robot.robotStuff.SweeperUp.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
            robot.robotStuff.SweeperDown.setPosition(robot.robotStuff.SERVO_LATCH_UP);
        }
        //Enable/Disable Sweeper Servos
        if (gamepad2.y){

            robot.robotStuff.SweeperUp.getController().pwmDisable();
            robot.robotStuff.SweeperDown.getController().pwmDisable();
        }
        else if (gamepad2.x) {
            robot.robotStuff.SweeperUp.getController().pwmEnable();
            robot.robotStuff.SweeperDown.getController().pwmEnable();
        }
        //Sweeper Extends
        if (gamepad2.dpad_up && robot.robotStuff.SweeperExtends.getCurrentPosition() > -6000){
            robot.robotStuff.SweeperExtends.setPower(-1);
            telemetry.addData("EncoderSweeperCount", robot.robotStuff.SweeperExtends.getCurrentPosition());
            telemetry.update();
        }
        else if (robot.robotStuff.SweeperExtends.getCurrentPosition() < -6000){
            robot.robotStuff.SweeperExtends.setPower(0);
            if (gamepad2.dpad_left){
                robot.robotStuff.SweeperExtends.setPower(1);
            }
            telemetry.addData("EncoderSweeperCount", robot.robotStuff.SweeperExtends.getCurrentPosition());
            telemetry.update();
        }
        else if (gamepad2.dpad_down && robot.robotStuff.SweeperExtends.getCurrentPosition() < 0){
            robot.robotStuff.SweeperExtends.setPower(1);
            telemetry.addData("EncoderSweeperCount", robot.robotStuff.SweeperExtends.getCurrentPosition());
            telemetry.update();
        }
        else {
            robot.robotStuff.SweeperExtends.setPower(0);
        }

        //Latch Lift
        if (gamepad2.a) {

//            robot.robotStuff.Latch.setTargetPosition(3410);
//            robot.robotStuff.Latch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.robotStuff.Latch.setPower(1);
            telemetry.addData("Latch Encoder", robot.robotStuff.Latch.getCurrentPosition());
            telemetry.update();
        }
        else if (gamepad2.b){
//            robot.robotStuff.Latch.setTargetPosition(0);
//            robot.robotStuff.Latch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.robotStuff.Latch.setPower(-1);
            telemetry.addData("Latch Encoder", robot.robotStuff.Latch.getCurrentPosition());
            telemetry.update();
        }
        else {
            robot.robotStuff.Latch.setPower(0);
        }

    }
}
