package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.Pose2d;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.HardwareInterface;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="teleoptest", group="Iterative OpMode")
public class Teleop extends OpMode {

    Robot robot;

    AIMPad aimPad1;
    AIMPad aimPad2;


    //hardwareMap = new

    public void init() {


        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);


        robot = new Robot(new Pose2d(0, 0, 0), false);
        robot.init(hardwareMap);
    }

    public void loop() {


        aimPad1.update(gamepad1);
        aimPad2.update(gamepad2);

        robot.loop(aimPad1, aimPad2);
        robot.telemetry(telemetry);
        telemetry.addData("raw A", gamepad1.a);
        telemetry.addData("raw X", gamepad1.x);
        telemetry.update();
    }

}
