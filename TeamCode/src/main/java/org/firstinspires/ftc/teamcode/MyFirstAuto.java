package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="My First Auto", group="none")
public class MyFirstAuto extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain drivetrain;

    public void initialize() {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new DriveTrain(fl, fr, bl, br);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        drivetrain.Drive(-0.5F, 10, Direction.BACKWARD);


    }
}
