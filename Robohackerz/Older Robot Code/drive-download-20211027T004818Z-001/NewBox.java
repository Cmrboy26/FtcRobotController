package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "NewBox", group = "ds")
public class NewBox extends OpMode {
    ImportantStuff robot = new ImportantStuff();

    private enum State {
        STATE_LOWER,
        STATE_MOVE,
        STATE_ARM,
        STATE_CALIBRATE,
        STATE_VIEW,
        STATE_SAMPLE,
        STATE_STOP,
        STATE_CENTER,
        STATE_CRASHC,
        STATE_BOXC,
        STATE_DUMPC,
        STATE_CRASHR,
        STATE_BOXR,
        STATE_DUMPR,
        STATE_STRAFEL,
        STATE_LEFT,
        STATE_CRASHL,
        STATE_FORWARDL,
        STATE_TURNL,
        STATE_DUMPL,
        STATE_BOXL,
        STATE_ESCAPE
    }

    private State CurrentState;
    ElapsedTime Runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AVvRb+j/////AAAAGabuHrQd6kQ7l5YaZFAEzC6AmLVcjBBw1TKdOQhxTBsNH/lNWBs70Q3M5qMoXQyOrF1IPTU6moBI3dDs+e93BWS4bQ2/LdMj10COpd8u9BAXwXgjAuXa3gkmQ3FvTtz78/1ynZ25zMe0po8fOMttCGhcL/PU1eaDmvI2Cdr41Lj3LJc2ASoLw8wWbJ2I1kTHCDRw/63m6zQklqWPCTHfLMOmdGRE1lgP/ukUtmbvWXnUCGPxI1YiKkwIzPk7CyNeUnRSXiXmPHp3Gp5kNj+9YmjheT5pJe9QID2AR+a8fJQZIYcLkODLGj83xcwBJPXupOWGb/R9dRevEx4CJWa8SNuGrcqxGPDeVP4CfEpTf0B1";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double leftSpeed;
    double rightSpeed;

    double halfLeftSpeed;
    double halfRightSpeed;

    double direction;

    Acceleration gravity;
    //    DigitalChannel          touch;
    Orientation angles;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        composeTelemetry();
        Runtime.reset();
        }

    @Override
    public void start() {
        super.start();
        robot.robotStuff.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        super.start();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        Runtime.reset();
        newState(State.STATE_LOWER);
    }

    @Override
    public void loop() {
        direction = robot.robotStuff.roboterror.firstAngle;
        robot.robotStuff.roboterror = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //robot.robotStuff.roboterror = robot.robotStuff.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        composeTelemetry();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("state", CurrentState);

        switch (CurrentState) {
            case STATE_LOWER:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.Latch.setPower(1);
                    telemetry.addData("Latch Encoder", robot.robotStuff.Latch.getCurrentPosition());
                    telemetry.update();
                    newState(State.STATE_MOVE);
                }
                break;

            case STATE_MOVE:
                if (robot.robotStuff.Latch.getCurrentPosition() > 3030) {
                    robot.robotStuff.FLeft.setPower(-1);
                    robot.robotStuff.FRight.setPower(1);
                    robot.robotStuff.BLeft.setPower(1);
                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_ARM);
                }
                break;

            case STATE_ARM:
                if (Runtime.milliseconds() > 125) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.Latch.setPower(-1);
                    newState(State.STATE_CALIBRATE);
                }
                break;

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
                    newState(State.STATE_CRASHC);
                }
                break;

            case STATE_VIEW:
                if (robot.robotStuff.imu.isGyroCalibrated()) {
                    robot.robotStuff.FLeft.setPower(.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(.5);
                    newState(State.STATE_STOP);
                }
                break;


            case STATE_SAMPLE:
                if (Runtime.milliseconds() > 150) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 2) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1) {
                                    if (goldMineralX < silverMineral1X /*&& goldMineralX < silverMineral2X*/) {
                                        telemetry.addData("Gold Mineral Position", "Center");
                                        newState(State.STATE_CENTER);
                                    } else if (goldMineralX > silverMineral1X) {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                        newState(State.STATE_CRASHR);
                                    }
                                    else {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        newState(State.STATE_STRAFEL);
                                    }
                                } else if (silverMineral1X != -1 && silverMineral2X != -1){
                                    if (silverMineral1X < silverMineral2X){
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        newState(State.STATE_STRAFEL);
                                    }
                                    else {
                                        telemetry.addData("Gold Mineral Position", "Left");
                                        newState(State.STATE_STRAFEL);
                                    }
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
                break;

            case STATE_CENTER:
                if (Runtime.milliseconds() > 1000) {
                    robot.robotStuff.FLeft.setPower(-.25);
                    robot.robotStuff.FRight.setPower(-.25);
                    robot.robotStuff.BLeft.setPower(-.25);
                    robot.robotStuff.BRight.setPower(-.25);
                    newState(State.STATE_CRASHC);
                }
                break;

            case STATE_CRASHC:
                if (Runtime.milliseconds() > 250) {
                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(-leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(-leftSpeed);
//                    robot.robotStuff.FLeft.setPower(-1);
//                    robot.robotStuff.FRight.setPower(1);
//                    robot.robotStuff.BLeft.setPower(1);
//                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_BOXC);
                }
                break;

            case STATE_BOXC:
                if (Runtime.milliseconds() > 500) {
                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(-leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(-leftSpeed);
//                    robot.robotStuff.FLeft.setPower(-1);
//                    robot.robotStuff.FRight.setPower(1);
//                    robot.robotStuff.BLeft.setPower(1);
//                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_DUMPC);
                }
                break;

            case STATE_DUMPC:
                if (Runtime.milliseconds() > 400) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.MarkerDump.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                    newState(State.STATE_STOP);
                }
                break;

            case STATE_CRASHR:
                if (Runtime.milliseconds() > 1000) {
                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(-leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(-leftSpeed);
//                    robot.robotStuff.FLeft.setPower(-1);
//                    robot.robotStuff.FRight.setPower(1);
//                    robot.robotStuff.BLeft.setPower(1);
//                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_BOXR);
                }
                break;

            case STATE_BOXR:
                if (Runtime.milliseconds() > 1300) {
                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(-leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(-leftSpeed);
//                    robot.robotStuff.FLeft.setPower(-1);
//                    robot.robotStuff.FRight.setPower(1);
//                    robot.robotStuff.BLeft.setPower(1);
//                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_DUMPR);
                }
                break;

            case STATE_DUMPR:
                if (Runtime.milliseconds() > 900) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.MarkerDump.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                    newState(State.STATE_STOP);
                }
                break;

            case STATE_STRAFEL:
                if (Runtime.milliseconds() > 1000) {
                    halfLeftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    halfRightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    halfLeftSpeed = Range.clip(leftSpeed, -.5, .5);
                    halfRightSpeed = Range.clip(rightSpeed, -.5, .5);

                    robot.robotStuff.FLeft.setPower(-halfLeftSpeed);
                    robot.robotStuff.FRight.setPower(halfRightSpeed);
                    robot.robotStuff.BLeft.setPower(halfRightSpeed);
                    robot.robotStuff.BRight.setPower(-halfLeftSpeed);
//                    robot.robotStuff.FLeft.setPower(-.5);
//                    robot.robotStuff.FRight.setPower(.5);
//                    robot.robotStuff.BLeft.setPower(.5);
//                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_LEFT);
                }
                break;

            case STATE_LEFT:
                if (Runtime.milliseconds() > 900) {
                    robot.robotStuff.FLeft.setPower(-.25);
                    robot.robotStuff.FRight.setPower(-.25);
                    robot.robotStuff.BLeft.setPower(-.25);
                    robot.robotStuff.BRight.setPower(-.25);
                    newState(State.STATE_CRASHL);
                }
                break;

            case STATE_CRASHL:
                if (Runtime.milliseconds() > 2200) {
                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(-leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(-leftSpeed);
//                    robot.robotStuff.FLeft.setPower(-1);
//                    robot.robotStuff.FRight.setPower(1);
//                    robot.robotStuff.BLeft.setPower(1);
//                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_FORWARDL);
                }
                break;

            case STATE_FORWARDL:
                if (Runtime.milliseconds() > 800) {
                    leftSpeed = .5 + (getAngle() - robot.robotStuff.angles.firstAngle)/100;
                    rightSpeed = .5 - (getAngle() - robot.robotStuff.angles.firstAngle)/100;

                    leftSpeed = Range.clip(leftSpeed, -1, 1);
                    rightSpeed = Range.clip(rightSpeed, -1, 1);

                    robot.robotStuff.FLeft.setPower(-leftSpeed);
                    robot.robotStuff.FRight.setPower(rightSpeed);
                    robot.robotStuff.BLeft.setPower(rightSpeed);
                    robot.robotStuff.BRight.setPower(-leftSpeed);
//                    robot.robotStuff.FLeft.setPower(-1);
//                    robot.robotStuff.FRight.setPower(1);
//                    robot.robotStuff.BLeft.setPower(1);
//                    robot.robotStuff.BRight.setPower(-1);
                    newState(State.STATE_DUMPL);
                }
//add to box
            case STATE_DUMPL:
                if (Runtime.milliseconds() > 1250) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.MarkerDump.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                    newState(State.STATE_ESCAPE);
                }
                break;

            case STATE_ESCAPE:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(.5);
                }

            case STATE_STOP:
                if (Runtime.milliseconds() > 300) { //was 1200 nope 300
                    robot.stop();
                }
                break;
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
                });
        telemetry.addData("Runtime", Runtime.milliseconds());
    }

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
//        robot.robotStuff.lastAngles = robot.robotStuff.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        robot.robotStuff.globalAngle = 0;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
