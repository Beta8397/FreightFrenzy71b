package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;

@Autonomous(name = "TestKalmanAuto", group = "test")
public class TestKalmanAuto extends MecBotAutonomous {

    DistanceSensor[] distanceSensors;
    float[] offSets = new float[]{8.5f,8.5f,8.5f,8.5f};
    MecBot bot = new MecBot(MecBot.MotorType.NeverestOrbital20,43.1f,
            48.1f,562.81f,-1);

    public void runLoggingOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
        distanceSensors = new DistanceSensor[] {
                hardwareMap.get(DistanceSensor.class,"front_distance"),
                hardwareMap.get(DistanceSensor.class,"right_distance"),
                hardwareMap.get(DistanceSensor.class,"back_distance"),
                hardwareMap.get(DistanceSensor.class,"left_distance")
        };
        bot.setKalmanMeasurementUpdater(new KalmanDistanceUpdater(distanceSensors,offSets, AllianceColor.RED));
        bot.setPose(32,79.5f,0);
        waitForStart();

        while (opModeIsActive()){
            driveToPosition(12,79,79.5f,0,1);
            driveToPosition(12,79,126.5f,0,1);
            driveToPosition(12,32,126.5f,0,1);
            driveToPosition(12,32,79.5f,0,1);
        }
    }
}
