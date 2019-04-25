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
import org.firstinspires.ftc.teamcode.MyClass.MyRobot;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="Worlds BlueCrater", group="none")

public class AutoBlueCrater extends AutoBase {


        @Override
        public void runOpMode() throws InterruptedException {

            initialize();

            telemetry.addData(">", "Press Start to test ScanGold.");
            telemetry.update();

            // need to set up Marker first.
            // Wait for the start button
            waitForStart();


            hopper.setPosition(0.9);
            // Lower the robot and detach from the lander
            this.releaseFromLander();

            // Prep steps a) Move forward 3 inches, b) strafe, c) turn about 45 degree, ready to scan mineral
            float a = imu.getAngularOrientation().firstAngle;
            drivetrain.Drive(0.40f, 3.5f, Direction.FORWARD); //3.5

            sampleByGrabbing(3.5f, a);

            MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();
            drivetrain.StrafeToImage(0.3f, blueTarget, this, 15);
            sleep(1000);

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
            drivetrain.Drive(1f, 40f, Direction.BACKWARD);
            drivetrain.Strafe(.4F, 3f, Direction.LEFT);
            sleep(100);
            drivetrain.Turn(.5f, 180, Direction.COUNTERCLOCKWISE, imu, this);
            sleep(100);
            drivetrain.Strafe(.4F, 5f, Direction.LEFT);
            drivetrain.Drive(1f, 20f, Direction.FORWARD);
            markerHook.setPosition(1);

            MyRobot.linearSlidePosition = intakeMotor.getCurrentPosition();
        }











    }
