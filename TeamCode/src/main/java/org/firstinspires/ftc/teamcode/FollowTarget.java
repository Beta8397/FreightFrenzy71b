package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous (name = "follow target", group = "test")
public class FollowTarget extends LinearOpMode {

    MecBot bot = new MecBot(MecBot.MotorType.NeverestOrbital20, 43.1f,
            48.1f, 562.81f, -1);

    final OpenGLMatrix cameraLocation = OpenGLMatrix.translation(1.5f*25.4f, 8*25.4f, 1*25.4f).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.XZX, AngleUnit.DEGREES, 90, 180, 0));


    Pose pose = new Pose(0,0,0);

    public void runOpMode() {
        bot.init(hardwareMap);
        final OpenGLMatrix[] targetPositions = new OpenGLMatrix[4];
        for (int i = 0; i < 4; i++) {
            targetPositions[i] = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XYX, AngleUnit.DEGREES, 90, 0, 0));
        }
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");

        VuforiaNavigator.activate("FreightFrenzy", targetPositions,
                cameraLocation, VuforiaLocalizer.CameraDirection.BACK, webcamName);
        waitForStart();
        while (opModeIsActive()) {
            OpenGLMatrix poseMatrix = null;
            for (int i = 0; i < 4; i++) {
                poseMatrix = VuforiaNavigator.getFieldFromRobot(i);
                if (poseMatrix != null) {
                    telemetry.addData("Target", " %d", i);
                    break;
                }
            }
            if (poseMatrix == null) {
                telemetry.addData("No Target Found", "");
                telemetry.addData("Last Pose", "X = %.1f  Y = %.1f  Th = %.1f", pose.x, pose.y,
                        Math.toDegrees(pose.theta));
                telemetry.update();
                bot.setDrivePower(0, 0, 0);
                continue;
            }
            pose = VuforiaNavigator.getPoseFromLocationTransform(poseMatrix);
            float xError = -1.5f - pose.x;
            float yError = -20 - pose.y;
            float d1 = (float) Math.hypot(xError, yError);

            OpenGLMatrix fieldFromCamera = poseMatrix.multiplied(cameraLocation);
            float[] camData = fieldFromCamera.getData();
            float camXmm = camData[12];
            float camYmm = camData[13];

//            float headingTargetRadians = (float) AngleUtils.normalizeRadians(
//                    Math.atan2(-pose.y, -pose.x) + Math.atan2(1.5, 8));
//            float thetaError = (float) AngleUtils.normalizeRadians(headingTargetRadians - pose.theta);

            float headingTargetRadians = (float)AngleUtils.normalizeRadians(Math.atan2(-camYmm, -camXmm));
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - pose.theta);

            float sinTheta = (float) Math.sin(pose.theta);
            float cosTheta = (float) Math.cos(pose.theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = 0, vy = 0, va = 0;

            if (Math.abs(Math.toDegrees(thetaError)) > 5) {
                va = 3*thetaError;
            } else if (d1 > 2) {
                float speed = 3 * Math.abs(d1);
                if (speed > 20) {
                    speed = 20;
                } else if (speed < 4){
                    speed = 4;
                }
                vx = xErrorRobot * speed / d1;
                vy = yErrorRobot * speed / d1;
            }

            bot.setDriveSpeed(vx,vy,va);

            telemetry.addData("pose","x=%.1f  y=%.1f  theta=%.1f",pose.x,pose.y,
                    Math.toDegrees(pose.theta));
            telemetry.update();
        }
    }
}