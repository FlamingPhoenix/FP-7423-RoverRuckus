package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MyClass.Calculator;
import org.firstinspires.ftc.teamcode.MyClass.MyRobot;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="DE BlueDepot", group="none")
public class AutoBlueDepot extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        telemetry.addData(">", "Press Start to test ScanGold.");
        telemetry.update();

        // need to set up Marker first.
        // Wait for the start button
        waitForStart();
        Log.i("[phoenix]: ", String.format("imu1 = %f", imu.getAngularOrientation().firstAngle));

        hopper.setPosition(0.9);
        // Lower the robot and detach from the lander
        this.releaseFromLander();

        // Prep steps a) Move forward 3 inches, b) strafe, c) turn about 45 degree, ready to scan mineral
        float a = imu.getAngularOrientation().firstAngle;
        Log.i("[phoenix]: ", String.format("imu2 = %f", a));
        float distanceFromLander = 3.5f;
        drivetrain.Drive(0.40f, distanceFromLander, Direction.FORWARD); //3.5

        float rightMineralAngle = Calculator.getMineralAngle(distanceFromLander);
        telemetry.addData("angle: ", rightMineralAngle);
        Log.i("[phoenix]: ", String.format("%f", rightMineralAngle));
        telemetry.update();

        sleep(100);
        drivetrain.Turn(0.3f, 70, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(500);
        telemetry.addData("tfod: ", tfod == null);
        //drivetrain.Turn(0.25f, 44, Direction.COUNTERCLOCKWISE, imu, this); // 46 too much, 43 44 maybe right.
        MineralPosition position = goldPosition();
        telemetry.addData("GoldPosition", position.toString());
        telemetry.update();
        sleep(100);
        float nextTurn = 0;
        if (position == MineralPosition.RIGHT)
            nextTurn = a - rightMineralAngle - imu.getAngularOrientation().firstAngle; //3.7f
        else if (position == MineralPosition.CENTER)
            nextTurn = a - imu.getAngularOrientation().firstAngle; //4.24f
        else
            nextTurn = a + rightMineralAngle - imu.getAngularOrientation().firstAngle; //3.7f

        drivetrain.Turn(.40f, (int) Math.abs(nextTurn), Direction.CLOCKWISE, imu, this);
        sleep(500);
        Log.i("[phoenix]: ", String.format("imu3 = %f", imu.getAngularOrientation().firstAngle));

//        grabGold(position);
        drivetrain.Drive(.40f, 24, Direction.FORWARD);
        drivetrain.Drive(.40f, 13, Direction.BACKWARD);

//        int turnToImage = 0;
//        if (position == MineralPosition.RIGHT)
//            turnToImage = 115;
//        else if (position == MineralPosition.CENTER)
//            turnToImage = 85;
//        else
//            turnToImage = 45;

        drivetrain.Turn(.40f, (int)nextTurn, Direction.COUNTERCLOCKWISE, imu, this);
        // drivetrain.driveAndSwerve();
        if (position == MineralPosition.RIGHT)
            drivetrain.Drive(.40f, 30, Direction.FORWARD);
        else if (position == MineralPosition.CENTER)
            drivetrain.Drive(.40f, 27, Direction.FORWARD);
        else
            drivetrain.Drive(.40f, 25, Direction.FORWARD);
        drivetrain.Turn(.40f, 50, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(500);

        MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();
        drivetrain.StrafeToImage(0.3f, frontTarget, this);
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

        MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();

        drivetrain.Drive(.7f, 40f, Direction.FORWARD);
        sleep(100);
        markerHook.setPosition(0.1);
        sleep(500 );
        drivetrain.Drive(1f, 68f, Direction.BACKWARD);
        markerHook.setPosition(1);

        MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();
    }


}