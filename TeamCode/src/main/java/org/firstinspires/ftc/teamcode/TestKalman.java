package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.KalmanDistanceUpdater;
import org.firstinspires.ftc.teamcode.util.KalmanMeasurementUpdater;
import org.firstinspires.ftc.teamcode.util.Pose;

@TeleOp (name = "TestKalman", group = "test")
public class TestKalman extends MecBotTeleOp {

    DistanceSensor[] distanceSensors;
    MecBot bot = new MecBot();

    public void runOpMode(){
        bot.init(hardwareMap);

        super.setup(bot);

        distanceSensors = new DistanceSensor[] {
                hardwareMap.get(DistanceSensor.class,"front_distance"),
                hardwareMap.get(DistanceSensor.class,"right_distance"),
                hardwareMap.get(DistanceSensor.class,"back_distance"),
                hardwareMap.get(DistanceSensor.class,"left_distance")
        };
        float[] offSets = {
                7.5f,8.5f,7.5f,8.5f
        };
        bot.setKalmanMeasurementUpdater(new KalmanDistanceUpdater(distanceSensors,offSets, AllianceColor.RED));

        bot.setPose(7.5f,79,0);
        bot.setCovariance(0,0,0);

        waitForStart();
        while (opModeIsActive()) {
            bot.updateOdometry();
            telemetry.addData("Pose","x = %.1f  y = %.1f  th = %.1f",
                    bot.getPose().x,bot.getPose().y,Math.toDegrees(bot.getPose().theta));
            telemetry.addData("Cov","xx = %.1f  yy = %.1f  xy = %.1f",
                    bot.getCovariance().get(0,0),bot.getCovariance().get(1,1),
                    bot.getCovariance().get(0,1));
            telemetry.addData("Dist", "FR %.1f  RT %.1f  BK %.1f  LT %.1f",
                    distanceSensors[0].getDistance(DistanceUnit.INCH),
                    distanceSensors[1].getDistance(DistanceUnit.INCH),
                    distanceSensors[2].getDistance(DistanceUnit.INCH),
                    distanceSensors[3].getDistance(DistanceUnit.INCH));
            doDriveControl();
            telemetry.update();
        }
    }
}
