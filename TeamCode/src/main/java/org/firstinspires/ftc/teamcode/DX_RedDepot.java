package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="Dover RedDepot V2", group="none")
public class DX_RedDepot extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        initialize();

        telemetry.addData(">", "Press Start to test ScanGold.");
        telemetry.update();

        // need to set up Marker first.
        // Wait for the start button
        waitForStart();

        arm.setPosition(0.1);
        hopper.setPosition(0.9);
        // Lower the robot and detach from the lander
        this.releaseFromLander();

        // Prep steps a) Move forward 3 inches, b) strafe, c) turn about 45 degree, ready to scan mineral
        float a = imu.getAngularOrientation().firstAngle;
        drivetrain.Drive(0.40f, 3.5f, Direction.FORWARD); //3.5
        sleep(100);
        drivetrain.Turn(0.2f, 70, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(500);
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
            turnToImage = 90;
        else
            turnToImage = 45;

        drivetrain.Turn(.40f,turnToImage, Direction.COUNTERCLOCKWISE, imu, this);
        if (position == MineralPosition.RIGHT)
            drivetrain.Drive(.40f, 30, Direction.FORWARD);
        else if (position == MineralPosition.CENTER)
            drivetrain.Drive(.40f, 27, Direction.FORWARD);
        else
            drivetrain.Drive(.40f, 25, Direction.FORWARD);
        drivetrain.Turn(.40f, 50, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(1000);

        drivetrain.StrafeToImage(0.3f, backTarget, this);

        sleep(100);
        // VERY IMPORTANT, PLEASE KEEP THIS PART WHEN UPDATING FINAL VERSION OF AUTONOMOUS PROGRAM !!
        drivetrain.Strafe(.4F, 3f, Direction.LEFT);
        sleep(100);
        drivetrain.Turn(.5f, 180, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(100);
        drivetrain.Strafe(.4F, 5f, Direction.LEFT);

        telemetry.addData(" after the strafe to image", "after strafe to image");
        Log.i("[phoenix]:after strafe", "after strafe to image");
        // this sleep could be tuned to accommodate alliance partner, depending on when they coming to depot and drop their maker
        sleep(100);

        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();
        }

        drivetrain.Drive(.7f, 40f, Direction.FORWARD);
        sleep(100);
        markerHook.setPosition(0.1);
        sleep(500 );
        drivetrain.Drive(.7f, 66.5f, Direction.BACKWARD);

        */

        initialize();

        telemetry.addData(">", "Press Start to test ScanGold.");
        telemetry.update();

        // need to set up Marker first.
        // Wait for the start button
        waitForStart();

        arm.setPosition(0.1);
        hopper.setPosition(0.9);
        // Lower the robot and detach from the lander
        this.releaseFromLander();

        sleep(50);
        setRobotStartingAngle();
        sleep(50);

        // Prep steps a) Move forward 3 inches, b) strafe, c) turn about 45 degree, ready to scan mineral
        drivetrain.Drive(0.4f, 3.5f, Direction.FORWARD); //3.5
        sleep(100);
        drivetrain.Turn(0.25f, 44, Direction.COUNTERCLOCKWISE, imu, this); // 43 is good number

        sleep(100);
        this.sampleGold(this);

        sleep(100);
        float secondTurningAngle = Math.abs(robotStartingAngle + 135f - imu.getAngularOrientation().firstAngle);
        drivetrain.Turn(0.4f, (int)secondTurningAngle, Direction.COUNTERCLOCKWISE, imu, this);
        // then turn to image
        sleep(100);
        telemetry.addData(" after the turn, before strafe to image", "before strafe to image");
        Log.i("[phoenix]:after turn", "before strafe to image");
        // this is optional, as most likely the robot will see image after above 52 degree turn.
        drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, backTarget, imu, this);
        //strafe to image
        drivetrain.StrafeToImage(0.4f, backTarget, this); // was 0.4

        sleep(100);
        // VERY IMPORTANT, PLEASE KEEP THIS PART WHEN UPDATING FINAL VERSION OF AUTONOMOUS PROGRAM !!
        drivetrain.Strafe(.4F, 3f, Direction.LEFT);
        sleep(100);
        drivetrain.Turn(.5f, 180, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(100);
        drivetrain.Strafe(.4F, 4.5f, Direction.LEFT); // strafe back more to ensure touch the wall

        telemetry.addData(" after the strafe to image", "after strafe to image");
        Log.i("[phoenix]:after strafe", "after strafe to image");
        // this sleep could be tuned to accommodate alliance partner, depending on when they coming to depot and drop their maker
        sleep(100);

        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();
        }



        drivetrain.Drive(.7f, 40f, Direction.FORWARD);
        sleep(100);
        markerHook.setPosition(0.1);
        sleep(500 );
        drivetrain.Drive(.7f, 66.5f, Direction.BACKWARD);

    }

}
