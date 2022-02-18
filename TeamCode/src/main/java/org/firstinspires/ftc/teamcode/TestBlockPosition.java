package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

import java.util.List;
@Disabled
@Autonomous (name = "TestBlockPosition", group = "test")
public class TestBlockPosition extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaNavigator.activate(null,null);
        VuforiaNavigator.setFlashTorchMode(true);
        waitForStart();
        BlobHelper helper = new BlobHelper(1280,720,0,0,1280,720,4);
        HSV_Range range = new HSV_Range(35,55,0.8f,1.0f,0.3f,1.0f);
        while (opModeIsActive() && !helper.updateImage()) continue;
        List<Blob> list = helper.getBlobs(range);
        if (list != null && list.size()> 0) {
            while (list.size()>1) {
                if (list.get(1).getNumPts()>list.get(0).getNumPts()) list.remove(0);
                else list.remove(1);
            }
            float xi = list.get(0).getAvgX();
            float yi = list.get(0).getAvgY();
            double xr = 3.2632 - 0.0638 * xi - 0.1604 * yi + 1.5532e-4 * xi * xi
                    - 6.2807e-4 * yi * yi + 0.0016 * xi * yi;
            double yr = -78.52 + 0.1099 * xi + 1.3319 * yi - 1.5e-5 * xi * xi- 0.0078 * yi * yi
                    - 0.0015 * xi * yi;
            telemetry.addData("Block", "x = %.2f  y = %.2f", xr, yr);
        }else {
            telemetry.addData("no blob found","");
        }
        telemetry.update();
        while (opModeIsActive()) continue;
        VuforiaNavigator.setFlashTorchMode(false);
    }
}
