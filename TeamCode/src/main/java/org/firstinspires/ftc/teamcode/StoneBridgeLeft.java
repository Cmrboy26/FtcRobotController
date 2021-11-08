package org.firstinspires.ftc.teamcode;

import android.icu.text.RelativeDateTimeFormatter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "StoneBridgeLeft" , group = "LeftTurn")
public class StoneBridgeLeft extends OpMode {
    ImportantStuff robot = new ImportantStuff();

    private enum State {
        STATE_APPROACH,
        STATE_COLORR,
        STATE_RIGHTR,
        STATE_SKYSTONER,
        STATE_WHEELSR,
        STATE_COLLECTR,
        STATE_BACKR,
        STATE_PAUSER,
        STATE_SPINR,
        STATE_ACROSSR,
        STATE_WAITR,
        STATE_LEFTC,
        STATE_COLORC,
        STATE_SKYSTONEC,
        STATE_RIGHTC,
        STATE_COLLECTC,
        STATE_BACKC,
        STATE_PAUSEC,
        STATE_SPINC,
        STATE_ACROSSC,
        STATE_WAITC,
        STATE_SKYSTONEL,
        STATE_COLLECTL,
        STATE_BACKL,
        STATE_BRIDGE,
        STATE_PAUSEL,
        STATE_SPINL,
        STATE_ACROSSL,
        STATE_WAITL,
        STATE_TURN,
        STATE_UP,
        STATE_OUT,
        STATE_PLOP,
        STATE_DOWN,
        STATE_RETRACT,
        STATE_PARK,
        STATE_HALT,
        STATE_WHEELSC,
        STATE_WHEELSL,
        STATE_HOLDR,
        STATE_HOLDC,
        STATE_HOLDL
    }

    private State CurrentState;
    ElapsedTime Runtime = new ElapsedTime();

    @Override
    public void init() {robot.init(hardwareMap, telemetry); }

    @Override
    public void start() {
        super.start();
        Runtime.reset();
        newState(State.STATE_APPROACH);
    }

    @Override
    public void loop() {
        telemetry.addData("state", CurrentState);
        robot.robotStuff.colorSensor.red();
        robot.robotStuff.colorSensor.green();
        robot.robotStuff.colorSensor.blue();
        robot.robotStuff.colorSensor.alpha();
        robot.robotStuff.colorSensor.argb();

        switch (CurrentState) {
            case STATE_APPROACH:
                if (Runtime.milliseconds() > 250) {
                    robot.robotStuff.FLeft.setPower(.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(.5);
                    newState(State.STATE_COLORR);
                }
                break;

            case STATE_COLORR:
                if (Runtime.milliseconds() > 370) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    if (robot.robotStuff.colorSensor.red() > 40) {
                        newState(State.STATE_LEFTC);
                    } else if (robot.robotStuff.colorSensor.red() < 20) {
                        newState(State.STATE_RIGHTR);
                    } else {
                        newState(State.STATE_SKYSTONER);
                    }
                }
                break;

            case STATE_RIGHTR:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(-.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_SKYSTONER);
                }
                break;

            case STATE_SKYSTONER:
                if (Runtime.milliseconds() > 400) {
                    robot.robotStuff.RIntake.setPower(.5);
                    robot.robotStuff.LIntake.setPower(.5);
                    robot.robotStuff.FLeft.setPower(.25);
                    robot.robotStuff.FRight.setPower(.25);
                    robot.robotStuff.BLeft.setPower(.25);
                    robot.robotStuff.BRight.setPower(.25);
                    newState(State.STATE_WHEELSR);
                }
                break;

            case STATE_WHEELSR:
                if (Runtime.milliseconds() > 750) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.RIntake.setPower(.5);
                    robot.robotStuff.LIntake.setPower(.5);
                    newState(State.STATE_COLLECTR);
                }
                break;

            case STATE_COLLECTR:
                if (Runtime.milliseconds() > 300) {
                    robot.robotStuff.RIntake.setPower(0);
                    robot.robotStuff.LIntake.setPower(0);
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.liftUp.setPower(-1);
                    newState(State.STATE_BACKR);
                }
                break;

            case STATE_BACKR:
                if (Runtime.milliseconds() > 220) {
                    robot.robotStuff.liftUp.setPower(0);
                    robot.robotStuff.FLeft.setPower(-.5);
                    robot.robotStuff.FRight.setPower(-.5);
                    robot.robotStuff.BLeft.setPower(-.5);
                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_PAUSER);
                }
                break;

            case STATE_PAUSER:
                if (Runtime.milliseconds() > 450) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_SPINR);
                }
                break;

            case STATE_SPINR:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(1);
                    robot.robotStuff.FRight.setPower(-1);
                    robot.robotStuff.BLeft.setPower(1);
                    robot.robotStuff.BRight.setPower(-1);
                    robot.robotStuff.grabServo.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                    newState(State.STATE_HOLDR);
                }
                break;

            case STATE_HOLDR:
                if (Runtime.milliseconds() > 62) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_ACROSSR);
                }
                break;

            case STATE_ACROSSR:
                if (Runtime. milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(-.50);
                    robot.robotStuff.FRight.setPower(-.50);
                    robot.robotStuff.BLeft.setPower(-.50);
                    robot.robotStuff.BRight.setPower(-.50);
                    newState(State.STATE_WAITR);
                }
                break;

            case STATE_WAITR:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_UP);
                }
                break;

            case STATE_LEFTC:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(.5);
                    robot.robotStuff.FRight.setPower(-.5);
                    robot.robotStuff.BLeft.setPower(-.5);
                    robot.robotStuff.BRight.setPower(.5);
                    newState(State.STATE_COLORC);
                }
                break;

            case STATE_COLORC:
                if (Runtime.milliseconds() > 250) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    if (robot.robotStuff.colorSensor.red() > 40) {
                        newState(State.STATE_SKYSTONEL);
                    } else if (robot.robotStuff.colorSensor.red() < 20) {
                        newState(State.STATE_RIGHTC);
                    } else {
                        newState(State.STATE_SKYSTONEL);
                    }
                }
                break;

            case STATE_RIGHTC:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(-.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_SKYSTONEC);
                }
                break;

            case STATE_SKYSTONEC:
                if (Runtime.milliseconds() > 400) {
                    robot.robotStuff.RIntake.setPower(.5);
                    robot.robotStuff.LIntake.setPower(.5);
                    robot.robotStuff.FLeft.setPower(.25);
                    robot.robotStuff.FRight.setPower(.25);
                    robot.robotStuff.BLeft.setPower(.25);
                    robot.robotStuff.BRight.setPower(.25);
                    newState(State.STATE_WHEELSC);
                }
                break;

            case STATE_WHEELSC:
                if (Runtime.milliseconds() > 750) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.RIntake.setPower(.5);
                    robot.robotStuff.LIntake.setPower(.5);
                    newState(State.STATE_COLLECTC);
                }
                break;

            case STATE_COLLECTC:
                if (Runtime.milliseconds() > 300) {
                    robot.robotStuff.RIntake.setPower(0);
                    robot.robotStuff.LIntake.setPower(0);
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.liftUp.setPower(-1);
                    newState(State.STATE_BACKC);
                }
                break;

            case STATE_BACKC:
                if (Runtime.milliseconds() > 220) {
                    robot.robotStuff.liftUp.setPower(0);
                    robot.robotStuff.FLeft.setPower(-.5);
                    robot.robotStuff.FRight.setPower(-.5);
                    robot.robotStuff.BLeft.setPower(-.5);
                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_PAUSEC);
                }
                break;

            case STATE_PAUSEC:
                if (Runtime.milliseconds() > 450) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_SPINC);
                }
                break;

            case STATE_SPINC:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(1);
                    robot.robotStuff.FRight.setPower(-1);
                    robot.robotStuff.BLeft.setPower(1);
                    robot.robotStuff.BRight.setPower(-1);
                    robot.robotStuff.grabServo.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                    newState(State.STATE_HOLDC);
                }
                break;

            case STATE_HOLDC:
                if (Runtime.milliseconds() > 62) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_ACROSSC);
                }
                break;

            case STATE_ACROSSC:
                if (Runtime. milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(-.50);
                    robot.robotStuff.FRight.setPower(-.50);
                    robot.robotStuff.BLeft.setPower(-.50);
                    robot.robotStuff.BRight.setPower(-.50);
                    newState(State.STATE_WAITC);
                }
                break;

            case STATE_WAITC:
                if (Runtime.milliseconds() > 550) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_UP);
                }
                break;

            case STATE_SKYSTONEL:
                if (Runtime.milliseconds() > 250) {
                    robot.robotStuff.RIntake.setPower(.5);
                    robot.robotStuff.LIntake.setPower(.5);
                    robot.robotStuff.FLeft.setPower(.25);
                    robot.robotStuff.FRight.setPower(.25);
                    robot.robotStuff.BLeft.setPower(.25);
                    robot.robotStuff.BRight.setPower(.25);
                    newState(State.STATE_WHEELSL);
                }
                break;

            case STATE_WHEELSL:
                if (Runtime.milliseconds() > 750) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.RIntake.setPower(.5);
                    robot.robotStuff.LIntake.setPower(.5);
                    newState(State.STATE_COLLECTL);
                }
                break;

            case STATE_COLLECTL:
                if (Runtime.milliseconds() > 300) {
                    robot.robotStuff.RIntake.setPower(0);
                    robot.robotStuff.LIntake.setPower(0);
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.liftUp.setPower(-1);
                    newState(State.STATE_BACKL);
                }
                break;

            case STATE_BACKL:
                if (Runtime.milliseconds() > 220) {
                    robot.robotStuff.liftUp.setPower(0);
                    robot.robotStuff.FLeft.setPower(-.5);
                    robot.robotStuff.FRight.setPower(-.5);
                    robot.robotStuff.BLeft.setPower(-.5);
                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_PAUSEL);
                }
                break;

            case STATE_PAUSEL:
                if (Runtime.milliseconds() > 450) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_SPINL);
                }
                break;

            case STATE_SPINL:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(1);
                    robot.robotStuff.FRight.setPower(-1);
                    robot.robotStuff.BLeft.setPower(1);
                    robot.robotStuff.BRight.setPower(-1);
                    robot.robotStuff.grabServo.setPosition(robot.robotStuff.SERVO_LATCH_DOWN);
                    newState(State.STATE_HOLDL);
                }
                break;

            case STATE_HOLDL:
                if (Runtime.milliseconds() > 62) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_ACROSSL);
                }
                break;

            case STATE_ACROSSL:
                if (Runtime. milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(-.50);
                    robot.robotStuff.FRight.setPower(-.50);
                    robot.robotStuff.BLeft.setPower(-.50);
                    robot.robotStuff.BRight.setPower(-.50);
                    newState(State.STATE_WAITL);
                }
                break;

            case STATE_WAITL:
                if (Runtime.milliseconds() > 600) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    newState(State.STATE_UP);
                }
                break;

            case STATE_UP:
                if (Runtime.milliseconds() > 500) {
                    robot.robotStuff.FLeft.setPower(0);
                    robot.robotStuff.FRight.setPower(0);
                    robot.robotStuff.BLeft.setPower(0);
                    robot.robotStuff.BRight.setPower(0);
                    robot.robotStuff.liftUp.setPower(1);
                    newState(State.STATE_OUT);
                }
                break;

            case STATE_OUT:
                if (Runtime.milliseconds() > 135) {
                    robot.robotStuff.liftOut.setPower(-1);
                    robot.robotStuff.liftUp.setPower(0);
                    newState(State.STATE_PLOP);
                }
                break;

            case STATE_PLOP:
                if (Runtime.milliseconds() > 1000) {
                    newState(State.STATE_RETRACT);
                    robot.robotStuff.liftOut.setPower(0);
                    robot.robotStuff.grabServo.setPosition(robot.robotStuff.SERVO_LATCH_UP);
                }
                break;

            case STATE_RETRACT:
                if (Runtime.milliseconds() > 50) {
                    newState(State.STATE_DOWN);
                    robot.robotStuff.liftOut.setPower(1);
                }
                break;

            case STATE_DOWN:
                if (Runtime.milliseconds() > 1000) {
                    robot.robotStuff.liftOut.setPower(0);
                    robot.robotStuff.liftUp.setPower(-1);
                    newState(State.STATE_PARK);
                }
                break;

            case STATE_PARK:
                if (Runtime.milliseconds() > 135) {
                    newState(State.STATE_BRIDGE);
                    robot.robotStuff.liftUp.setPower(0);
                    robot.robotStuff.FLeft.setPower(.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(.5);
                }
                break;

            case STATE_BRIDGE:
                if (Runtime.milliseconds() > 300) {
                    robot.robotStuff.FLeft.setPower(-.5);
                    robot.robotStuff.FRight.setPower(.5);
                    robot.robotStuff.BLeft.setPower(.5);
                    robot.robotStuff.BRight.setPower(-.5);
                    newState(State.STATE_HALT);
                }
                break;

            case STATE_HALT:
                if (Runtime.milliseconds() > 300) {
                    robot.stop();
                }
                break;
        }
        telemetry.addData("state", CurrentState);
    }

    private void newState(State newState) {
        Runtime.reset();
        CurrentState = newState;
    }
}
