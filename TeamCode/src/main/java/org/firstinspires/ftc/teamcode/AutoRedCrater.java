package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.MyRobot;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="DE RedCrater", group="none")  // this is template for Thursday's auto routine, to be tested and adjusted Monday

public class AutoRedCrater extends AutoBase {


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        telemetry.addData(">", "Press Start to test ScanGold.");
        telemetry.update();

        // need to set up Marker first.
        // Wait for the start button
        waitForStart();
//
        hopper.setPosition(0.9);
        // Lower the robot and detach from the lander
        this.releaseFromLander();

        // Prep steps a) Move forward 3 inches, b) strafe, c) turn about 45 degree, ready to scan mineral
        float a = imu.getAngularOrientation().firstAngle;
        drivetrain.Drive(0.40f, 3.5f, Direction.FORWARD); //3.5
        sleep(100);
        drivetrain.Turn(0.2f, 70, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(100);
        telemetry.addData("tfod: ", tfod == null);
        //drivetrain.Turn(0.25f, 44, Direction.COUNTERCLOCKWISE, imu, this); // 46 too much, 43 44 maybe right.
        MineralPosition position = goldPosition();
        telemetry.addData("GoldPosition", position.toString());
        telemetry.update();
        sleep(100);
        float nextTurn = 0;
        if (position == MineralPosition.RIGHT)
            nextTurn = a - 32f - imu.getAngularOrientation().firstAngle;
        else if (position == MineralPosition.CENTER)
            nextTurn = a - imu.getAngularOrientation().firstAngle;
        else
            nextTurn = a + 32f - imu.getAngularOrientation().firstAngle;

        drivetrain.Turn(.40f, (int) Math.abs(nextTurn), Direction.CLOCKWISE, imu, this);
        drivetrain.Drive(.40f, 24, Direction.FORWARD);
        drivetrain.Drive(.40f, 13, Direction.BACKWARD);

        int turnToImage = 0;
        if (position == MineralPosition.RIGHT)
            turnToImage = 115;
        else if (position == MineralPosition.CENTER)
            turnToImage = 80;
        else
            turnToImage = 45;

        drivetrain.Turn(.40f,turnToImage, Direction.COUNTERCLOCKWISE, imu, this);
        if (position == MineralPosition.RIGHT)
            drivetrain.Drive(.40f, 30, Direction.FORWARD);
        else if (position == MineralPosition.CENTER)
            drivetrain.Drive(.40f, 27, Direction.FORWARD);
        else
            drivetrain.Drive(.40f, 23, Direction.FORWARD);
        drivetrain.Turn(.40f, 50, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(500);

        MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();
        drivetrain.StrafeToImage(0.3f, redTarget, this);
        float angleAfterStrafe = imu.getAngularOrientation().firstAngle;
        sleep(5000);

        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();
        }

        MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();

        drivetrain.Drive(0.6f, 48f, Direction.FORWARD);
        sleep(100);
//        // drop marker
        markerHook.setPosition(0.1);
        sleep(500);
        drivetrain.Drive(1f, 68f, Direction.BACKWARD); // continue to drive to crater
        markerHook.setPosition(1);

        MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();
        }

}
