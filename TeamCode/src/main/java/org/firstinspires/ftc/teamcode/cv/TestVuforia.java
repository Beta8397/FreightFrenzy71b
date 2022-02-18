package org.firstinspires.ftc.teamcode.cv;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by FTC Team 8397 on 9/13/2019.
 */
@Disabled
@Autonomous(name="TestVuforia", group="Test")
public class TestVuforia extends LinearOpMode {

    final OpenGLMatrix landscapePhoneLocation = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
            , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, 0));

    public void runOpMode() {
        final OpenGLMatrix[] targetPositions = new OpenGLMatrix[13];

        for(int i = 0; i < 13; i++) {
            targetPositions[i] = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XYX, AngleUnit.DEGREES, 90, 0, 0));
        }

        VuforiaNavigator.activate("Skystone", targetPositions,
                OpenGLMatrix.translation(/*-7.25f*25.4f*/ 0, 0, 0
                ).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC
                , AxesOrder.YXY, AngleUnit.DEGREES, 90, 180, 0)),
                VuforiaLocalizer.CameraDirection.BACK, null);
       //VuforiaNavigator.setFlashTorchMode(true);
        /*CameraDevice.getInstance().setField("zoom", "24");
        while(!opModeIsActive() && !isStopRequested()){
            OpenGLMatrix stuff = VuforiaNavigator.getRobotLocation(0);
            if(stuff == null){
                telemetry.addData("No Target found", "");
            } else {
                float[] locations = VuforiaNavigator.getX_Y_Theta_FromLocationTransform(stuff);

                telemetry.addData("Target found at ", "x:%.1f   y:%.1f   th:%.1f", locations[0], locations[1], locations[2] * 180.0f / Math.PI);
                telemetry.addData("", "%s", locations[0] < -10? "RIGHT" :  locations[0] < 10? "CENTER" : "LEFT");
            }
            telemetry.update();
        }*/
        CameraDevice.getInstance().setField("zoom", "0");
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive()) {
            if(et.milliseconds() <= 200) {
                continue;
            } else {
                et.reset();
                OpenGLMatrix pose = null;
                for(int i = 0; i < 13; i++) {
                    pose = VuforiaNavigator.getRobotLocation(i);
                    if(pose != null) {
                        telemetry.addData("Target", " %d", i);
                        break;
                    }
                }
                if(pose == null) {
                    telemetry.addData("No Target Found", "");
                } else {
                    float[] XYTheta = VuforiaNavigator.getX_Y_Theta_FromLocationTransform(pose);
                    telemetry.addData("Pose", " x=%.1f  y=%.1f  theta=%.1f",
                            XYTheta[0], XYTheta[1], XYTheta[2] * 180.0 / Math.PI);
//                    float[] data = pose.getData();
//                    telemetry.addData("Pose", "X = %.1f  Y = %.1f  Z = %.1f  Th = %.1f",
//                            data[12]/10.0, data[13]/10.0, data[14]/10.0,
//                            Math.atan2(data[1], data[0])*180.0/Math.PI);
                }

                telemetry.update();
            }
        }

        //VuforiaNavigator.setFlashTorchMode(false);
    }
}
