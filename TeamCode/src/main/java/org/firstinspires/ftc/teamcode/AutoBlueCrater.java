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
@Autonomous(name="M_W BlueCrater", group="none")

public class AutoBlueCrater extends AutoBase {


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
            drivetrain.Drive(0.40f, 3.5f, Direction.FORWARD); //3.5
            sleep(100);
            drivetrain.Turn(0.25f, 44, Direction.COUNTERCLOCKWISE, imu, this); // was 46

            sleep(100);
            this.sampleGold(this);

            // ??? dont think this line is needed
            //drivetrain.Strafe(0.6f, 3.5f, Direction.RIGHT);
            sleep(100);

            drivetrain.Turn(0.4f, 42, Direction.COUNTERCLOCKWISE, imu, this);
            // then turn to image
            sleep(100);
            telemetry.addData(" after the turn, before strafe to image", "before strafe to image");
            Log.i("[phoenix]:after turn", "before strafe to image");
            // this is optional, as most likely the robot will see image after above 52 degree turn.
            drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, blueTarget, imu, this);
            //strafe to image
            drivetrain.StrafeToImage(0.4f, blueTarget, this); // was 0.4
            telemetry.addData(" after the strafe to image", "after strafe to image");
            Log.i("[phoenix]:after strafe", "after strafe to image");
            // this sleep could be tuned to accommodate alliance partner, depending on when they coming to depot and drop their maker
            sleep(300);

            if (tfod != null) { // now it is ok to shutdown tfod/vuforia
                tfod.deactivate();
                tfod.shutdown();
            }

            // drive backward for to depot, it was 58
            drivetrain.Drive(0.6f, 54.5f, Direction.FORWARD);
            sleep(100);
            // drop marker
            markerHook.setPosition(0.0);
            sleep(500);
            drivetrain.Drive(.65f, 68f, Direction.BACKWARD); // continue to drive to crater

            // end of auto routine.
        }











    }
