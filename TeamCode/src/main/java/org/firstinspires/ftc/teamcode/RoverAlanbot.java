package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.util.Range.scale;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RoverAlanbot" , group = "TOComp")


public class RoverAlanbot extends OpMode {
    ImportantStuff robot = new ImportantStuff();

    //int MineralDumpDebounce = 0;
    boolean MineralDumpPressed;

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
            telemetry.update();
        }
        else {
            robot.robotStuff.MineralRaise.setPower(0);
        }

        // Mineral dump
        // Old Code \/
        if (gamepad1.y){
            robot.robotStuff.MineralDump.setPosition(robot.robotStuff.SERVO_LATCH_UP);
        }
        else if (gamepad1.a){
            robot.robotStuff.MineralDump.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
        }
        /*if(gamepad1.a || gamepad1.y) {
            if(!MineralDumpPressed) {
                MineralDumpPressed = true;
                if (robot.robotStuff.MineralDump.getPosition() == (robot.robotStuff.SERVO_LATCH_DOWN)) {
                    robot.robotStuff.MineralDump.setPosition(robot.robotStuff.SERVO_LATCH_UP);
                } else {
                    robot.robotStuff.MineralDump.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                }
                telemetry.addData("MineralDumpPressed", MineralDumpPressed);
                telemetry.addData("MineralDumpPos", +robot.robotStuff.MineralDump.getPosition());
                telemetry.update();
                // TODO Double check this
            }
        } else if(!MineralDumpPressed && !(gamepad1.a || gamepad1.y)){
            MineralDumpPressed = false;
        }*/

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

        telemetry.addData("AngularOrientation", robot.robotStuff.imu.getAngularOrientation());
        telemetry.addData("AngularVelocityX", robot.robotStuff.imu.getAngularVelocity().xRotationRate);
        telemetry.addData("AngularVelocityY", robot.robotStuff.imu.getAngularVelocity().yRotationRate);
        telemetry.addData("AngularVelocityZ", robot.robotStuff.imu.getAngularVelocity().zRotationRate);
        telemetry.update();

    }
}
