package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Drivebase extends Mechanism {

    public MecanumDrive drive;
    private Pose2d startingPose;
    //TODO: Import starting pose !!!
    public Drivebase (Pose2d startingPose) {
        this.startingPose = startingPose;
    }

    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, startingPose);
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        drive.leftFront.setZeroPowerBehavior(behavior);
        drive.leftBack.setZeroPowerBehavior(behavior);
        drive.rightFront.setZeroPowerBehavior(behavior);
        drive.leftFront.setZeroPowerBehavior(behavior);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        drive.leftFront.setMode(mode);
        drive.leftBack.setMode(mode);
        drive.rightFront.setMode(mode);
        drive.leftFront.setMode(mode);
    }

    @Override
    public void loop (AIMPad gamepad) {
        manualDrive(gamepad);
    }
    public void manualDrive (AIMPad gamepad) {
        double y = -gamepad.getLeftStickY();
        double x = gamepad.getLeftStickX();
        double rx = gamepad.getRightStickX();

        // Create left stick vector
        Vector2d leftStick = new Vector2d(y, -x);


        // Set drive powers
        drive.setDrivePowers(new PoseVelocity2d(leftStick, -rx));
    }
}
