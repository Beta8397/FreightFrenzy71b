package org.firstinspires.ftc.teamcode.skinnybot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mecbot.MecBot;

public class SkinnyBot extends MecBot {

    DcMotorEx armMotor;
    CRServo intake;

    public final int MIN_ARM_TICKS = 0;
    public final int MAX_ARM_TICKS = 2025;

    public SkinnyBot() {
        super(MotorType.NeverestOrbital20, 43.1f, 48.1f, 433,1);
    }

    public boolean init(HardwareMap hardwareMap) {
        boolean result = super.init(hardwareMap);
        armMotor = hardwareMap.get(DcMotorEx.class,"arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake = hardwareMap.get(CRServo.class,"intake");
        return result;
    }

    public void setArmPosition(int ticks) {
        if (ticks > MAX_ARM_TICKS) {
            ticks = MAX_ARM_TICKS;
        } else if (ticks < MIN_ARM_TICKS) {
            ticks = MIN_ARM_TICKS;
        }
        armMotor.setTargetPosition(ticks);
        armMotor.setPower(0.5f);
    }

    public void setIntakePower(float pwr) {
        intake.setPower(pwr);
    }

}
