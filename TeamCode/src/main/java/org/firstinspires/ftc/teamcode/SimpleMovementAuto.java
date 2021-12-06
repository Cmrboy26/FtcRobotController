package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "SimpleMovementAuto", group = "ds")
public class SimpleMovementAuto extends OpMode {
    ImportantStuff robot = new ImportantStuff();

    private enum State {
        START,
        MOVE5,
        PAUSE,
        IMU,
        END,
    }

    private State CurrentState;
    ElapsedTime Runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        //composeTelemetry();
    }

    @Override
    public void start() {
        super.start();
        //robot.robotStuff.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        super.start();
        //initVuforia();

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }*/

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        Runtime.reset();
        newState(State.START);
    }

    @Override
    public void loop() {
        switch(CurrentState) {
            case START: {

                if(Runtime.milliseconds() > 1000) {
                    BNO055IMU.CalibrationData calibrationData = robot.robotStuff.imu.readCalibrationData();
                    robot.robotStuff.FRight.setPower(.1);
                    robot.robotStuff.FLeft.setPower(.1);
                    robot.robotStuff.BRight.setPower(.1);
                    robot.robotStuff.BLeft.setPower(.1);
                    newState(State.MOVE5);
                }
                break;
            }
            case MOVE5: {
                if(Runtime.milliseconds() > 1000) {
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    newState(State.PAUSE);
                }
                break;
            }
            case PAUSE: {
                if(Runtime.milliseconds() > 1000) {
                    newState(State.IMU);
                }
                break;
            }
            case IMU: {
                if(Runtime.milliseconds() > 1000) {
                    //telemetry.addData("Current Room Temperature", robot.robotStuff.imu.getTemperature().temperature);
                    //telemetry.update();
                    newState(State.END);
                }
                //robot.robotStuff.imu;
                break;
            }
            case END: {
                if(Runtime.milliseconds() > 1000) {
                    robot.stop();
                }
                break;
            }
        }
        //if(CurrentState != State.END) {
            //telemetry.addData("Position:", "["+robot.robotStuff.imu.getPosition().x+","+robot.robotStuff.imu.getPosition().y+", "+robot.robotStuff.imu.getPosition().z+"]");
            telemetry.addData("Angle", robot.robotStuff.imu.getAngularOrientation());
            telemetry.update();
        //}
    }

    private void newState(State newState) {
        Runtime.reset();
        CurrentState = newState;
    }


}

