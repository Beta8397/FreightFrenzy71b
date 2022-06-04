package org.firstinspires.ftc.teamcode.skinnybot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "SkinnyBotTeleOp", group = "test")
public class SkinnyBotTeleop extends MecBotTeleOp {

    SkinnyBot bot = new SkinnyBot();

    private ButtonToggle toggleLeftBumper = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.left_bumper;
        }
    };
    private ButtonToggle toggleRightBumper = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.right_bumper;
        }
    };
    private ButtonToggle toggleY = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.y;
        }
    };

    public void runLoggingOpMode() {
        bot.init(hardwareMap);
        super.setup(bot);
        waitForStart();
        while (opModeIsActive()) {
            doDriveControl();
            int currentArmTicks = bot.armMotor.getCurrentPosition();
            telemetry.addData("arm ticks",currentArmTicks);
            int targetTicks = currentArmTicks;
            if (gamepad1.dpad_left) {
                targetTicks = targetTicks - 50;
            } else if (gamepad1.dpad_right) {
                targetTicks += 50;
            }
            bot.setArmPosition(targetTicks);
            boolean rightBumperToggled = toggleRightBumper.update();
            boolean leftBumperToggled = toggleLeftBumper.update();
            float currentIntakePower = (float)bot.intake.getPower();
            telemetry.addData("intake power",currentIntakePower);
            if (rightBumperToggled) {
                if (currentIntakePower <= 0) {
                    bot.setIntakePower(1);
                } else {
                    bot.setIntakePower(0);
                }
            } else if (leftBumperToggled) {
                if (currentIntakePower >= 0) {
                    bot.setIntakePower(-1);
                } else {
                    bot.setIntakePower(0);
                }
            }

            if (toggleY.update()) {
                telemetry.speak("take me to your leader");
            }

            telemetry.update();
        }
    }
}
