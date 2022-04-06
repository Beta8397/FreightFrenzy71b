package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestDistanceMulti")
public class TestDistanceMulti extends LinearOpMode {

    DistanceSensor[] distanceSensors;

    public void runOpMode() {
        distanceSensors = new DistanceSensor[]{
                hardwareMap.get(DistanceSensor.class,"left_distance"),
                hardwareMap.get(DistanceSensor.class,"right_distance"),
                hardwareMap.get(DistanceSensor.class,"front_distance"),
                hardwareMap.get(DistanceSensor.class,"back_distance")
        };
        waitForStart();

        double[] distances = new double[4];

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                distances[i] = distanceSensors[i].getDistance(DistanceUnit.INCH);
            }
            telemetry.addData("dist", "L = %.1f  R = %.1f  F = %.1f  B = %.1f",
                    distances[0],distances[1],distances[2],distances[3]);
            telemetry.update();
        }
    }
}
