package org.firstinspires.ftc.teamcode.cv.webcam;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PinHole {

    private float cameraFocalLengthPixels = 0;
    private float elevationAngleDegrees = 0;
    private float elevationAngleRadians = 0;
    private float cameraHeight = 0;
    private Pose cameraPoseOnRobot = new Pose(0, 0, 0);

    private MatrixF cameraToRobotTransform = null;

    public PinHole(float focalLengthPixels, float elevationDegrees, float heightInches, Pose poseOnRobot){
        cameraFocalLengthPixels = focalLengthPixels;
        elevationAngleDegrees = elevationDegrees;
        elevationAngleRadians = (float)Math.toRadians(elevationDegrees);
        cameraHeight = heightInches;
        cameraPoseOnRobot = poseOnRobot;
        float sin = (float)Math.sin(poseOnRobot.theta);
        float cos = (float)Math.cos(poseOnRobot.theta);
        cameraToRobotTransform = new GeneralMatrixF(3, 3,
                new float[]{
                        cos, -sin, poseOnRobot.x,
                        sin, cos, poseOnRobot.y,
                        0, 0, 1
                });
    }

    public void setElevationAngle(float elevationDegrees){
        elevationAngleDegrees = elevationDegrees;
        elevationAngleRadians = (float)Math.toRadians(elevationDegrees);
    }

    public void setCameraHeight(float heightInches){
        cameraHeight = heightInches;
    }

    public void setCameraPoseOnRobot(Pose poseOnRobot){
        cameraPoseOnRobot = poseOnRobot;
        float sin = (float)Math.sin(poseOnRobot.theta);
        float cos = (float)Math.cos(poseOnRobot.theta);
        cameraToRobotTransform = new GeneralMatrixF(3, 3,
                new float[]{
                        cos, -sin, poseOnRobot.x,
                        sin, cos, poseOnRobot.y,
                        0, 0, 1
                });
    }

    public void setCameraFocalLengthPixels(float focalLengthPixels){
        cameraFocalLengthPixels = focalLengthPixels;
    }


}
