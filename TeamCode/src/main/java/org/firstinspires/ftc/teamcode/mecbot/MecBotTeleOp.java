package org.firstinspires.ftc.teamcode.mecbot;

import android.annotation.SuppressLint;
import android.content.Context;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * Abstract class to provide TeleOp functionality for a MechBot.
 *
 * Note:  Subclasses of MechBotTeleOp must call MechBotTeleOp.setup(...), to provide MechBotTeleOp
 * with a reference to a MechBot, and do other initialization.
 *
 * It is the responsibility of the subclass to initialize the MechBot.
 *
 * The control loop of the subclass of MechBotTeleOp should call MechBotTeleOp.doDriveControl()
 * during each iteration.
 *
 */
public abstract class MecBotTeleOp extends LinearOpMode {

    MecBot bot = null;

    public enum SpeedMode {
        SLOW, FAST, PANIC
    }

    private SpeedMode speedMode = SpeedMode.FAST;
    private boolean quadMode = false;
    private boolean telemetryEnabled = true;
    private boolean fieldCentric = false;
    private float px, py, pa;

    //Set up gamepad buttons as toggles

    private ButtonToggle toggleD1up = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad1.dpad_up;
        }
    };
    private ButtonToggle toggleD1down = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad1.dpad_down;
        }
    };
    private ButtonToggle toggleD1left = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad1.dpad_left;
        }
    };


    protected static final float SLOW_MODE_SCALER = 3.0f;
    protected static final float FAST_MODE_SCALER = 2f;
    protected static final float JOYSTICK_DEADZONE = 0.05f;
    protected static final float TRIGGER_DEADZONE = 0.05f;

    protected void setup(MecBot mecBot){
        bot = mecBot;
    }

    /**
     * Handle one iteration of MechBot drive control.
     * Checks toggle buttons for slow mode, quad mode, and telemetry
     * Checks left joystick and triggers, then sets robot drive powers (x,y,angle)
     * If telemetry enabled, adds drive data to telemetry
     */
    protected void doDriveControl(){

        if (toggleD1up.update()) {
             speedMode = speedMode == SpeedMode.SLOW? SpeedMode.FAST: SpeedMode.SLOW;

        }
        if (toggleD1left.update()) {
            speedMode = speedMode == SpeedMode.PANIC? SpeedMode.FAST: SpeedMode.PANIC;
        }
        if (toggleD1down.update()) {
            quadMode = !quadMode;
        }



        px = gamepad1.left_stick_x;
        py = -gamepad1.left_stick_y;



        float leftTrigger = gamepad1.left_trigger;
        if (leftTrigger < TRIGGER_DEADZONE) leftTrigger = 0;

        float rightTrigger = gamepad1.right_trigger;
        if (rightTrigger < TRIGGER_DEADZONE) rightTrigger = 0;

        pa = leftTrigger > TRIGGER_DEADZONE ? leftTrigger : -rightTrigger;

        if (Math.abs(px) < JOYSTICK_DEADZONE) px = 0;
        if (Math.abs(py) < JOYSTICK_DEADZONE) py = 0;
        if (Math.abs(pa) < TRIGGER_DEADZONE) pa = 0;

        if (quadMode){
            //This is just value squared.
            //(-1/1) * px * px.
            px = Math.signum(px) * px * px;
            py = Math.signum(py) * py * py;
            pa = Math.signum(pa) * pa * pa;
        }
        if (speedMode == SpeedMode.SLOW) {
            px /= SLOW_MODE_SCALER;
            py /= SLOW_MODE_SCALER;
            pa /= SLOW_MODE_SCALER;
        } else if (speedMode == SpeedMode.FAST) {
            px /= FAST_MODE_SCALER;
            py /= FAST_MODE_SCALER;
            pa /= FAST_MODE_SCALER;
        }

        bot.setDrivePower( px,  py,  pa);

        //This should always be set to enabled.

        //By sending telemetry to the driver station during any loop or stop condition. This prevents
        //a rare disconnect or desync error caused by lack of communication over the network. This
        //should not be a common error on the motog play4 phones but can be seen on phones that
        //support an higher network level, 5ghz.
        if (telemetryEnabled) doTelemetry();

    }

    private void doTelemetry(){
        telemetry.addData("Mode: ",speedMode);
        telemetry.addData("QUAD: ", quadMode);
        telemetry.addData("Drive Power: ","px %.2f  py %.2f  pa %.2f",
                px, py, pa);
    }


}
