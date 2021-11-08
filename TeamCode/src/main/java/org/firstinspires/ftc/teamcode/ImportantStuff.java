package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

/**
 * Created by zahs on 9/28/2017.
 */

public class ImportantStuff {
    private Telemetry telemetry;
    private HardwareMap HWMap = null;

    SkystoneBot robotStuff = new SkystoneBot();

    public void init(HardwareMap Map, Telemetry telemetry) {
        HWMap = Map;
        this.telemetry = telemetry;
        robotStuff.init(Map, telemetry, FORWARD);
    }

    public void stop() {
        robotStuff.stop();
    }
}
