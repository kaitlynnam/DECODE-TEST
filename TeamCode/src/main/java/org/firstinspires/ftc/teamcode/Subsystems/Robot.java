package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends Mechanism {

    Drivebase db;
    Arm arm = new Arm();

    Pose2d startingPose;
    boolean isAuto;

    public Robot(Pose2d startingPose, boolean isAuto) {
        this.startingPose = startingPose;
        this.isAuto = isAuto;

        Drivebase db = new Drivebase(startingPose);
    }


    @Override
    public void init(HardwareMap hwMap) {

        db.init(hwMap);
        arm.init(hwMap);
    }

    @Override
    public void loop(AIMPad gamepad) {
        db.loop(gamepad);
        arm.loop(gamepad);
    }

    public void telemetry (Telemetry telemetry) {
        arm.telemetry(telemetry);
    }
}
