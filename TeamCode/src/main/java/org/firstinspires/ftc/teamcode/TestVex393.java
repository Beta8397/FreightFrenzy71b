package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "test vex 393", group = "test")
public class TestVex393 extends LinearOpMode {

    private CRServo vex;

    public void runOpMode(){

        vex = hardwareMap.get(CRServo.class, "vex");

        waitForStart();

        while (opModeIsActive()){
            float pow = -gamepad1.left_stick_y;
            vex.setPower(pow);
            telemetry.addData("Vex Power: ", pow);
            telemetry.update();
        }
    }
}
