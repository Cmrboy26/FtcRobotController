package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@Autonomous(name = "TRYTHISGYRO", group = "ds")

public class GyroStrafe extends OpMode {
    ImportantStuff robot = new ImportantStuff();

    double leftSpeed;
    double rightSpeed;

    double direction;

    Acceleration gravity;
    //    DigitalChannel          touch;
    Orientation angles;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;


    double startPosition = robot.robotStuff.roboterror.firstAngle;

    private enum State {
        STATE_CALIBRATE,
        STATE_STRAFE,
        STATE_STOP
    }

    private State CurrentState;
    ElapsedTime Runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        composeTelemetry();
    }

    @Override
    public void start() {
        super.start();
        robot.robotStuff.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        super.start();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        Runtime.reset();
        newState(State.STATE_CALIBRATE);
    }

    @Override
    public void loop() {


        direction = robot.robotStuff.roboterror.firstAngle;

        robot.robotStuff.roboterror = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        composeTelemetry();


        switch (CurrentState) {

            case STATE_CALIBRATE:
                if (robot.robotStuff.Latch.getCurrentPosition() < 8) {
                    composeTelemetry();
                    BNO055IMU.CalibrationData calibrationData = robot.robotStuff.imu.readCalibrationData();
                    telemetry.addData("State Calibrate", "State Calibrate");


                    // Save the calibration data to a file. You can choose whatever file
                    // name you wish here, but you'll want to indicate the same file name
                    // when you initialize the IMU in an opmode in which it is used. If you
                    // have more than one IMU on your robot, you'll of course want to use
                    // different configuration file names for each.
                    String filename = "AdafruitIMUCalibration.json";
                    File file = AppUtil.getInstance().getSettingsFile(filename);
                    ReadWriteFile.writeFile(file, calibrationData.serialize());
                    telemetry.log().add("saved to '%s'", filename);
                    composeTelemetry();
                    telemetry.update();

                    while (!robot.robotStuff.imu.isGyroCalibrated()) {
                        try {
                            Thread.sleep(50);
                        } catch (InterruptedException e) {
                            System.out.println("got interrupted!");
                        }
                    }
                    robot.robotStuff.roboterror = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    newState(State.STATE_STRAFE);
                }
                break;
            case STATE_STRAFE:
                if (robot.robotStuff.imu.isGyroCalibrated()){

                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(leftSpeed);


                    newState(State.STATE_STOP);
                }
                break;
            case STATE_STOP:
                if (Runtime.milliseconds() > 3000){
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                }


        }

    }

    private void newState(State newState) {
        Runtime.reset();
        CurrentState = newState;
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            robot.robotStuff.angles   = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.robotStuff.gravity  = robot.robotStuff.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.robotStuff.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.robotStuff.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.robotStuff.angles.angleUnit, robot.robotStuff.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.robotStuff.angles.angleUnit, robot.robotStuff.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.robotStuff.angles.angleUnit, robot.robotStuff.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return robot.robotStuff.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(robot.robotStuff.gravity.xAccel*robot.robotStuff.gravity.xAccel
                                        + robot.robotStuff.gravity.yAccel*robot.robotStuff.gravity.yAccel
                                        + robot.robotStuff.gravity.zAccel*robot.robotStuff.gravity.zAccel));
                    }
                }); }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    private void resetAngle()
    {
        lastAngles = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}
