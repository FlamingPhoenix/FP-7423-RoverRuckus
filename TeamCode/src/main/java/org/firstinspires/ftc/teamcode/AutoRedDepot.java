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

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="M_W RedDepot", group="none")
public class AutoRedDepot extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

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
        drivetrain.Drive(0.4f, 3.0f, Direction.FORWARD); //3.5
        sleep(500);
        drivetrain.Turn(0.25f, 46, Direction.COUNTERCLOCKWISE, imu, this);

        sleep(500);
        this.sampleGold(this);

        sleep(200);
        drivetrain.Turn(0.35f, 52, Direction.COUNTERCLOCKWISE, imu, this);
        // then turn to image
        sleep(300);
        telemetry.addData(" after the turn, before strafe to image", "before strafe to image");
        Log.i("[phoenix]:after turn", "before strafe to image");
        // this is optional, as most likely the robot will see image after above 52 degree turn.
        drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, backTarget, imu, this);
        //strafe to image
        drivetrain.StrafeToImage(0.25f, backTarget, this); // was 0.4

        // VERY IMPORTANT, PLEASE KEEP THIS PART WHEN UPDATING FINAL VERSION OF AUTONOMOUS PROGRAM !!
        drivetrain.Strafe(.4F, 3, Direction.LEFT);
        drivetrain.Turn(.3f, 180, Direction.COUNTERCLOCKWISE, imu, this);
        drivetrain.Strafe(.4F, 3, Direction.LEFT);

        telemetry.addData(" after the strafe to image", "after strafe to image");
        Log.i("[phoenix]:after strafe", "after strafe to image");
        // this sleep could be tuned to accommodate alliance partner, depending on when they coming to depot and drop their maker
        sleep(300);

        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();
        }



        drivetrain.Drive(.4f, 48f, Direction.FORWARD);
        sleep(300);
        markerHook.setPosition(0.1);
        sleep(300 );
        drivetrain.Drive(.4f, 70f, Direction.BACKWARD);

        // drive backward to depot, it was 58

        // end of auto routine.
    }

}
