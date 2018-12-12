package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="RedCrater StrafeVersion", group="none")  // this is template for Thursday's auto routine, to be tested and adjusted Monday

public class AutoRedCraterNewStrafe extends AutoBase {

    private ElapsedTime runtime = new ElapsedTime();
    private static final long firstHitTime = 1250; // this is from calibration, it is time to detect first object
    private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    private static final long thirdHitTime = 12000;

    private static final float firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
    private static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
    private static final float thirdHitDistance = 27.5f;

    private static final float firstStopAngle = 50f;  // 20 by rough measure
    private static final float secondStopAngle = 90f; // 90 by rough measure.


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
            StrafeWhileVisible(0.3f, 14.0f, 520f, 10, this); // was 10, goldwidth was 460
            telemetry.addData("Gold aft straf", "after strafe");
            Log.i("[phoenix]:gold aft str", "after strafe");
            sleep(300);
            drivetrain.Strafe(0.30f, 8.5f, Direction.LEFT); // was 6.5
            sleep(100);
            drivetrain.Turn(0.4f, 35, Direction.COUNTERCLOCKWISE, imu, this); // shouid be 45, compensate for wheel issue
            //this is to test strafe directly to image w/o driving forward
            //sleep(100);
            //drivetrain.Drive(0.3f, 32f, Direction.FORWARD);
            }
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
        // check angle, 80 is ok, a bit over, try 70 or 72
        drivetrain.Turn(0.35f, 70, Direction.COUNTERCLOCKWISE, imu, this); // was 52 for drive forward. 95 for no drving
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

        // drive backward for to depot
   //     drivetrain.Drive(.4f, 58f, Direction.FORWARD);
        sleep(300);
        // drop marker
   //     markerHook.setPosition(0.1);
        sleep(300);
        //drivetrain.Drive(1.0f, 5, Direction.BACKWARD);//  drop marker, need to add back 5 ?
   //     drivetrain.Drive(.65f, 75, Direction.BACKWARD); // continue to drive to crater

        // end of auto routine.
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }

    public void scanGold_Diagonal ( float power, float leftScreenPosition, float rightScreenPosition,LinearOpMode opMode){
        long currentTime;
        int turnAngle;
        int gold_Found = 0;
        float distanceFromStart = 0f;
        int firstHitEncoderCount = Math.round(1120f * firstHitDistance / (4 * 3.1416f)); //89*3.5 = 312, 0.8 is a factor
        int secondHitEncoderCount = Math.round(1120f * secondHitDistance / (4 * 3.1416f)); // 89*15.5 = 1380
        int thirdEncoderCount = Math.round(1120f * thirdHitDistance / (4 * 3.1416f)); // 89*27.5 = 2448
        int currentPosition = 0;
        int gold_loop_No = 0;

        float currentAngle; // angle provided by TF method
        double estimatedAngle;
        float left_Gold;
        float right_Gold;
        float center_Gold;  // center coodinates of gold
        float current_Gold_Width; // width of gold, to drive towards it..
        float actualPower = power;

        runtime.reset(); // need to use time for tracking minerals instead of just  number of objects
        currentTime = Math.round(runtime.milliseconds());

        telemetry.addData("before moving time", currentTime);
        Log.i("[phoenix]:pretime", Double.toString(currentTime));

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (gold_Found == 0 && opModeIsActive() && (currentTime < (thirdHitTime - 2000))) { // 12000-2000 = 10000


            fl.setPower(actualPower);  // first calibrate time on floor..0.12 at dr warners, 0.14 at carpet
            fr.setPower(actualPower);
            bl.setPower(actualPower);
            br.setPower(actualPower);

            currentTime = Math.round(runtime.milliseconds());

            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() <= 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                gold_loop_No = gold_loop_No + 1;
                                currentTime = Math.round(runtime.milliseconds());
                                opMode.telemetry.addData("1st Gold time ", currentTime);
                                Log.i("[phoenix]:goldtime", Double.toString(currentTime));

                                //if (currentTime < (1.5*secondHitTime)) {
                                //goldMineralX = (int) recognition.getLeft();
                                left_Gold = recognition.getLeft();
                                right_Gold = recognition.getRight();
                                // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                center_Gold = (left_Gold + right_Gold) / 2f;
                                current_Gold_Width = right_Gold - left_Gold;

                                opMode.telemetry.addData("center of gold ", center_Gold);
                                Log.i("[phoenix]:i_goldcenter", Float.toString(center_Gold));
                                Log.i("[phoenix]:i_goldleft", Float.toString(left_Gold));
                                Log.i("[phoenix]:i_goldright", Float.toString(right_Gold));
                                Log.i("[phoenix]:i_width", Float.toString(current_Gold_Width));
                                Log.i("[phoenix]:i_Pre Cnt ", Integer.toString(fr.getCurrentPosition()));

                                // 2018-11-26 Ended the evening not sure what to put for the if statement below.  May need to do hand testing to test left, right and center values
                                if ((center_Gold > leftScreenPosition) && (center_Gold < rightScreenPosition)) {   //270f, 340f here 300 can change to other numbers, perhaps 400 ?
                                    //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                    currentTime = Math.round(runtime.milliseconds());
                                    Log.i("[phoenix]:goldtime ", Double.toString(currentTime));
                                    drivetrain.StopAll();
                                    currentPosition = Math.abs(fr.getCurrentPosition());
                                    opMode.telemetry.addData("aft encoder ", currentPosition);
                                    Log.i("[phoenix]:aft encoder ", Integer.toString(currentPosition));
                                    opMode.telemetry.update();
                                    if (currentPosition < (firstHitEncoderCount + 900) && (gold_Found == 0)) {//900+312 = 1112 //300+firsthit = 612,about 3.5 inches extra(currentTime < (secondHitTime - 500)) { // this is first time hit
                                        // should try pro-strafe instead of regular strafe, to test navigation control

                                        //StrafeWhileVisible(0.30f, 29f, 5);

// drivetrain.StopAll();
                                              drivetrain.Drive(0.2f, 3.0f, Direction.FORWARD);
                                              sleep(200);
                                              drivetrain.Strafe(0.30f, 7.5f, Direction.RIGHT);// was 6.5
                                              sleep(200);
                                              drivetrain.Strafe(0.30f, 5.5f, Direction.LEFT); // was 4.5
                                              drivetrain.StopAll();
                                              //sleep(200);
                                              // here drive 1.414*12 inch = 17.0
                                        // ACTIVE TESTING:  Original value 34f for distance
                                              //drivetrain.Drive(0.3f, 25f, Direction.FORWARD);  //skip if strafe directly
                                        gold_Found = 2; // gold is in B position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        opMode.telemetry.addData("at end of gold loop", "gold 2");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 2");
                                        opMode.telemetry.addData("gold frequency", gold_loop_No);
                                        Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                        opMode.telemetry.update();
                                    } else if ((gold_Found == 0)) { //1380 + 700 = 2180, 1380+600= 1980, 7 inches more, currentTime > (secondHitTime + 4000)) { // third time hit
                                        //StrafeWhileVisible(0.30f, 5.5f, 5);
                                            drivetrain.Drive(0.2f, 3.0f, Direction.FORWARD);
                                            drivetrain.Strafe(0.30f, 7.5F, Direction.RIGHT); //// was 6.5
                                            drivetrain.StopAll();
                                            sleep(500);
                                            drivetrain.Strafe(0.30f, 5.5F, Direction.LEFT); //// was 4.5
                                            drivetrain.StopAll();
                                            //sleep(200);
                                            //drivetrain.Drive(0.3f, 14f, Direction.FORWARD);// skip if strafe directly

                                        gold_Found = 3;  // gold is in C position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        opMode.telemetry.addData("at end of gold loop", "gold 3");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 3");
                                        opMode.telemetry.addData("gold frequency", gold_loop_No);
                                        Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                        opMode.telemetry.update();
                                    }
                                }
                            }
                        }
                    }
                }

                opMode.telemetry.update();
            }
        }

        if (gold_Found == 0) {
            currentTime = Math.round(runtime.milliseconds());
            currentPosition = fr.getCurrentPosition();
            opMode.telemetry.addData("no gold found, current time ", currentTime);
            opMode.telemetry.addData("no gold found, current counter ", currentPosition);
            opMode.telemetry.addData("gold found flag ", gold_Found);

            Log.i("[phoenix]:NoGold, time ", Double.toString(currentTime));
            Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
            Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
            opMode.telemetry.update();
            drivetrain.StopAll();
        }

    }

}
