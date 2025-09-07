package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.HardwareInterface;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends Mechanism {
    private DcMotorEx arm;


    private double targetPosition;
    private double kP = 0.001;

    @Override
    public void init(HardwareMap hwMap) {
        arm = hwMap.get(DcMotorEx.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(AIMPad gamepad) {
        updateArm();


        if (gamepad.isXPressed()) {
            setArm(-50);
        } else if (gamepad.isAPressed()) {
            setArm(0);
        }


    }

    public void updateArm () {
        double error = targetPosition - arm.getCurrentPosition();

        double power = -error * kP;
        arm.setPower(power);
    }
    void setArm (double target) {
        targetPosition = target;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Arm pos", arm.getCurrentPosition());
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Power", arm.getPower());
        telemetry.update();
    }


}
