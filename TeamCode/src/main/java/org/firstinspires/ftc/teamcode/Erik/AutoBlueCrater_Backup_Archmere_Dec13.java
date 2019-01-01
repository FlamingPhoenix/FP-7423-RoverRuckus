package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="Archmere BlueCrater", group="none")

public class AutoBlueCrater_Backup_Archmere_Dec13 extends AutoBase {


        @Override
        public void runOpMode() throws InterruptedException {

            initialize();

            telemetry.addData(">", "Press Start to test ScanGold.");
            telemetry.update();

            // need to set up Marker first.
            // Wait for the start button
            waitForStart();


            // Lower the robot

            // Detach from the lander

            // Prep steps a) Move forward 3 inches, b) strafe, c) turn about 45 degree, ready to scan mineral

/////////////////  ////////////////////////////////////////////////////////////////////////////////////////////////
            //drivetrain.Drive(0.20f, 6.0f, Direction.FORWARD); //3.5
            //sleep(300);
            //drivetrain.Strafe(0.25f, 7.0f, Direction.RIGHT );
            //sleep(300);
            //drivetrain.Turn(0.25f, 51, Direction.COUNTERCLOCKWISE, imu, this);
            //sleep(300);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            ////////////alternative-less tested///////
            // drivetrain.Drive(0.20f, 3.0f, Direction.FORWARD); //3.5
            // sleep(300);
            // drivetrain.Turn(0.25f, 35, Direction.COUNTERCLOCKWISE, imu, this);
            // sleep(300);
            // drivetrain.Strafe(0.25f, 3.0f, Direction.RIGHT);
            // sleep(300);
            // drivetrain.Drive(0.2f, 2.0f, Direction.BACKWARD);
            // sleep(500);
////////////////////////fully-tested//////////////////////////////////////////////////////////////////////////////////////
            drivetrain.Strafe(0.3f, 6.5f, Direction.RIGHT);
            sleep(300);
            //drivetrain.Drive(0.2f, 1.5f, Direction.BACKWARD);
            //sleep(300);
/////////////////////////////////////////////////////


            // scan first mineral

            int detectionOutcome = 0;

            // could have another variable, stop_distance, so that even no GOLD is found, will stop and pause.
            detectionOutcome = DriveToScanFirstMineral(0.11f, Direction.FORWARD, this); // 0.11 at Dr Warner, 0.15 at carpets
            sleep(300);

            // Explanation: decide next step based on outcome of first mineral,
            // if A is gold, hit it anc come back, turn 45 dgree CCW
            // if A is not Gold, turn 45 degrees CCW, scan Gold Diagonally, once find the gold, hit and come back.

            if (detectionOutcome == 1) { //ScanFirstMineral() == 1
                telemetry.addData("Gold found", "during first scan");
                Log.i("[phoenix]:gold detected", "found gold");
                sleep(300);
                StrafeWhileVisible(0.4f, 12.5f, 520f, 10, this); // was 10, goldwidth was 460
                telemetry.addData("Gold aft straf", "after strafe");
                Log.i("[phoenix]:gold aft str", "after strafe");
                sleep(300);
                drivetrain.Strafe(0.3f, 8.5f, Direction.LEFT); // was 6.5
                sleep(100);
                drivetrain.Turn(0.4f, 35, Direction.COUNTERCLOCKWISE, imu, this); // shouid be 45, compensate for wheel issue
                sleep(100);
                drivetrain.Drive(0.3f, 32f, Direction.FORWARD);}
            else if (detectionOutcome == 2) { //ScanFirstMineral() == 2, in this scenario, either B or C is GOLD
                telemetry.addData("Silver found", "during first scan");
                Log.i("[phoenix]:Silv detected", "found silver");
                sleep(300);
                drivetrain.Turn(0.2f, 35, Direction.COUNTERCLOCKWISE, imu, this); // should be 45, compensate for wheels issue
                telemetry.addData("Silver aft turn", "after turn");
                Log.i("[phoenix]:Silv aft turn", "aft turn");
                sleep(300);
                drivetrain.Drive(0.2f, 3.0f, Direction.BACKWARD);
                sleep(300);
                // scan the next two minerals for GOLD
                scanGold_Diagonal(0.11f, 200, 420, this); // was 240 and 380
                sleep(100);
                //drivetrain.Drive(0.3f, 1f, Direction.FORWARD);
            }

            else {
                telemetry.addData("no mineral found", "during first scan");
                telemetry.update();
                Log.i("[phoenix]: scan result", "no gold");
            }



            sleep(200);
            drivetrain.Turn(0.35f, 52, Direction.COUNTERCLOCKWISE, imu, this);
            // then turn to image
            sleep(300);
            telemetry.addData(" after the turn, before strafe to image", "before strafe to image");
            Log.i("[phoenix]:after turn", "before strafe to image");
            // this is optional, as most likely the robot will see image after above 52 degree turn.
            drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, blueTarget, imu, this);
            //strafe to image
            drivetrain.StrafeToImage(0.25f, blueTarget, this); // was 0.4
            telemetry.addData(" after the strafe to image", "after strafe to image");
            Log.i("[phoenix]:after strafe", "after strafe to image");
            // this sleep could be tuned to accommodate alliance partner, depending on when they coming to depot and drop their maker
            sleep(300);

            if (tfod != null) { // now it is ok to shutdown tfod/vuforia
                tfod.deactivate();
                tfod.shutdown();
            }

            // drive backward for to depot, it was 58
            drivetrain.Drive(.4f, 53f, Direction.FORWARD);
            sleep(300);
            // drop marker
            markerHook.setPosition(0.1);
            sleep(300);
            drivetrain.Drive(.65f, 75, Direction.BACKWARD); // continue to drive to crater

            // end of auto routine.
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }











    }
