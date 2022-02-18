package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TestMotor", group = "test")
public class TestMotor extends LinearOpMode {

    DcMotorEx motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "arm_extender_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean running = false;
        float power = 0.1f;

        waitForStart();

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()) {
            if (et.milliseconds() > 10) {
                et.reset();
                if (gamepad1.y) {
                    power += 0.002f;
                } else if (gamepad1.a) {
                    power = power - 0.002f;
                }
            }

            if (power > 1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
            }

            telemetry.addData("" +
                    "power =", power);
            telemetry.addData("ticks", motor.getCurrentPosition());
            telemetry.addData("ticks per sec", motor.getVelocity());
            telemetry.update();

            if (gamepad1.x) {
                motor.setPower(power);
            } else {
                motor.setPower(0);
            }
        }
    }
}
