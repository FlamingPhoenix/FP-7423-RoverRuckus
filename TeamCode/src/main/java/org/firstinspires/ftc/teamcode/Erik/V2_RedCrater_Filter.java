package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;

/**
 * Created by Steve on 7/22/2018.
 */

@Disabled
@Autonomous(name="V2 RedCrater New", group="none")  // this is template for Thursday's auto routine, to be tested and adjusted Monday

public class V2_RedCrater_Filter extends AutoBase {

    //private ElapsedTime runtime = new ElapsedTime();
    //private static final long firstHitTime = 1250; // this is from calibration, it is time to detect first object
    //private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    //private static final long thirdHitTime = 12000;

    //private static final float firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
    //private static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
    //private static final float thirdHitDistance = 27.5f;
    private float reference_Bottom_Y = 0;

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
//        drivetrain.Strafe(0.3f, 6.5f, Direction.RIGHT);
  //      sleep(300);
    //    drivetrain.Drive(0.2f, 1.80f, Direction.BACKWARD);
      //  sleep(500);
//////////////new approach ///////////////////////////////////////


        drivetrain.Drive(0.20f, 3.0f, Direction.FORWARD); //3.5
        sleep(500);
        //drivetrain.Strafe(0.25f, 7.0f, Direction.RIGHT );
        //sleep(1000);
        drivetrain.Turn(0.25f, 46, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(500);
        //drivetrain.Turn(0.40f, 70, Direction.COUNTERCLOCKWISE, imu, this);

        // scan first mineral

        int detectionOutcome = 0;

        //detectionOutcome = DriveToScanFirstMineral(0.11f, Direction.FORWARD, this); // 0.11 at Dr Warner, 0.15 at carpets
        detectionOutcome = ScanFirstMineralSimple();
        telemetry.addData("mineral scan outcome", detectionOutcome);
        Log.i("[phoenix]:min outcome", Integer.toString(detectionOutcome));

        // Explanation: decide next step based on outcome of first mineral,
        // if A is gold, hit it anc come back, turn 45 dgree CCW
        // if A is not Gold, turn 45 degrees CCW, scan Gold Diagonally, once find the gold, hit and come back.
        sleep(200);
        if (detectionOutcome == 1) { //ScanFirstMineral() == 1
            telemetry.addData("Gold found", "during first scan");
            Log.i("[phoenix]:gold detected", "found gold");
            sleep(200);
            StrafeWhileVisible(0.3f, 22f, 720, 10, this);
            telemetry.addData("Gold aft straf", "after strafe");
            Log.i("[phoenix]:gold aft str", "after strafe");
            sleep(300);
            drivetrain.Strafe(0.3f, 8.5f, Direction.LEFT);
            drivetrain.Turn(0.4f, 35, Direction.COUNTERCLOCKWISE, imu, this); // shouid be 45, compensate for wheel issue
            drivetrain.Drive(0.3f, 32f, Direction.FORWARD);}
        else { //ScanFirstMineral() == 2, in this scenario, either B or C is GOLD
            telemetry.addData("Silver found", "during first scan");
            Log.i("[phoenix]:Silv detected", "found silver");
            // strafe to the right position
            drivetrain.Strafe(0.25f, 5.5f, Direction.RIGHT);  // was 2 or 5.5 before..need to evaluate the risk of hitting lander leg
            sleep(500);
            drivetrain.Turn(0.2f, 35, Direction.COUNTERCLOCKWISE, imu, this); // should be 45, compensate for wheels issue
            telemetry.addData("Silver aft turn", "after turn");
            Log.i("[phoenix]:Silv aft turn", "aft turn");
            sleep(2000);
            // here will do a still scan, return mineral bottom, as reference for filtering.
            reference_Bottom_Y = FindClosestMineral_Y(0.9f,this);
            telemetry.addData("ref bottomY", reference_Bottom_Y);
            Log.i("[phoenix]:refBottomY", Float.toString(reference_Bottom_Y));
            sleep(1000);
            drivetrain.Drive(0.2f, 1.5f, Direction.BACKWARD);
            sleep(300);
            // scan the next two minerals for GOLD
            //scanGold_Diagonal(0.11f, 200, 420, this); // was 240 and 380
            scanGold_Diagonal_Filter(0.11f, 200, 420, reference_Bottom_Y, this);
            sleep(100);
            //drivetrain.Drive(0.3f, 1f, Direction.FORWARD);
            }




        sleep(200);
        drivetrain.Turn(0.35f, 52, Direction.COUNTERCLOCKWISE, imu, this);
        // then turn to image
        sleep(300);
        telemetry.addData(" after the turn, before strafe to image", "before strafe to image");
        Log.i("[phoenix]:after turn", "before strafe to image");
        // this is optional, as most likely the robot will see image after above 52 degree turn.
        drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, redTarget, imu, this);
        //strafe to image
        drivetrain.StrafeToImage(0.25f, redTarget, this); // was 0.4
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

        // drive backward for to depot, it was 58

       //drivetrain.Drive(.4f, 53f, Direction.FORWARD);
        //sleep(300);
        // drop marker
        //markerHook.setPosition(0.1);
        //sleep(300);
       //drivetrain.Drive(.65f, 75, Direction.BACKWARD); // continue to drive to crater

        // end of auto routine.
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }
        


}
