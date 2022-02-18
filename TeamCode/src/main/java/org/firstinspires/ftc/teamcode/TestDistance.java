package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "test distance", group = "test")
public class TestDistance extends LinearOpMode {

    DistanceSensor distance;

    public void runOpMode() {
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        waitForStart();

        while(opModeIsActive()) {
            double d = distance.getDistance(DistanceUnit.INCH);
            telemetry.addData("distance", d);
            telemetry.update();
        }

    }

}
