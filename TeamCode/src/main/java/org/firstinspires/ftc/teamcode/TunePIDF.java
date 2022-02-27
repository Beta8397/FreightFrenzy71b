package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "TunePIDF", group = "Test")
public class TunePIDF extends MecBotAutonomous {

    MecBot bot = new MecBot();

    ButtonToggle toggleA = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };
    ButtonToggle toggleDpadDown = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_down;
        }
    };
    ButtonToggle toggleY = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.y;
        }
    };
    ButtonToggle toggleX = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.x;
        }
    };
    ButtonToggle toggleB = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.b;
        }
    };
    ButtonToggle toggleDPadUp = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_up;
        }
    };

    public void runLoggingOpMode(){

        bot.init(hardwareMap);
        super.setBot(bot);

        while (!opModeIsActive() && !isStopRequested()){
            PIDFCoefficients pidf = bot.getDriveVelocityPIDF();
            telemetry.addData("PIDF", "p = %.2f  i = %.4f  f = %.2f",
                    pidf.p, pidf.i,
                    pidf.f);
            telemetry.update();

            if (toggleA.update()){
                bot.setDriveVelocityPIDF(pidf.p - 5, pidf.i, pidf.d, pidf.f);
            }
            if (toggleY.update()){
                bot.setDriveVelocityPIDF(pidf.p + 5, pidf.i, pidf.d, pidf.f);
            }
            if (toggleX.update()){
                bot.setDriveVelocityPIDF(pidf.p, pidf.i - 0.1, pidf.d, pidf.f);
            }
            if (toggleB.update()){
                bot.setDriveVelocityPIDF(pidf.p, pidf.i + 0.1, pidf.d, pidf.f);
            }
            if (toggleDpadDown.update()){
                bot.setDriveVelocityPIDF(pidf.p, pidf.i, pidf.d, pidf.f - 5);
            }
            if (toggleDPadUp.update()){
                bot.setDriveVelocityPIDF(pidf.p - 5, pidf.i, pidf.d, pidf.f + 5);
            }
        }


        while (opModeIsActive()){

            if (toggleY.update()){
                bot.setPose(0, 0, 0);
                driveToPosition(10, 24, 0, 0, 0.5f);
            }

            if (toggleB.update()){
                bot.setPose(0, 0, 0);
                driveToPosition(10, 0, -24, 0, 0.5f);
            }

            if (toggleX.update()){
                bot.setPose(0, 0, 0);
                turnToHeading(90, 3, 8, 60);
            }

        }

    }
}
