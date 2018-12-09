package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="Test Scan Gold New", group="none")

public class ScanGoldTestNew extends AutoBase {

    private ElapsedTime runtime = new ElapsedTime();
    private static final long firstHitTime = 1250; // this is from calibration, it is time to detect first object
    private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    private static final long thirdHitTime = 12000;

    private static final float firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
    private static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
    private static final float thirdHitDistance = 27.5f;

    private static final float firstStopAngle = 50f;  // 20 by rough measure
    private static final float secondStopAngle = 90f; // 90 by rough measure.
    //List<Recognition> updatedRecognitions;
    //Recognition recognition;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        int detectionOutcome = 0;
        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the start button
        telemetry.addData(">", "Press Start to test ScanGold.");
        telemetry.update();

        waitForStart();


        drivetrain.Drive(0.20f, 3.0f, Direction.FORWARD); //3.5
        sleep(1000);
        //drivetrain.Strafe(0.25f, 7.0f, Direction.RIGHT );
        //sleep(1000);
        drivetrain.Turn(0.25f, 51, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(1000);
        //drivetrain.Turn(0.40f, 70, Direction.COUNTERCLOCKWISE, imu, this);



        //detectionOutcome = DriveToScanFirstMineral(0.11f, Direction.FORWARD, this); // 0.11 at Dr Warner, 0.15 at carpets
        detectionOutcome = ScanFirstMineral();
        telemetry.addData("mineral scan outcome", detectionOutcome);
        Log.i("[phoenix]:min outcome", Integer.toString(detectionOutcome));

        sleep(500);
        if (detectionOutcome == 1) { //ScanFirstMineral() == 1
            telemetry.addData("Gold found", "during first scan");
            Log.i("[phoenix]:gold detected", "found gold");
            sleep(1000);
            StrafeWhileVisible(0.4f, 10.0f, 10);
            telemetry.addData("Gold aft straf", "after strafe");
            Log.i("[phoenix]:gold aft str", "after strafe");
            sleep(1000);
            drivetrain.Strafe(0.3f, 6.5f, Direction.LEFT);
            drivetrain.Turn(0.4f, 35, Direction.COUNTERCLOCKWISE, imu, this); // shouid be 45, compensate for wheel issue
            drivetrain.Drive(0.3f, 10f, Direction.FORWARD); }
        else if (detectionOutcome == 2) { //ScanFirstMineral() == 2
            telemetry.addData("Silver found", "during first scan");
            Log.i("[phoenix]:Silv detected", "found silver");
            sleep(1000);
           // drivetrain.Strafe(.3F, 6.5f, Direction.LEFT);
            sleep(1000);
            drivetrain.Turn(0.2f, 35, Direction.COUNTERCLOCKWISE, imu, this); // should be 45, compensate for wheels issue
            telemetry.addData("Silver aft turn", "after turn");
            Log.i("[phoenix]:Silv aft turn", "aft turn");
            sleep(1000);
            drivetrain.Drive(0.2f, 5.0f, Direction.BACKWARD);
            sleep(1000);
            scanGold_Diagonal(0.11f);
            drivetrain.Drive(0.3f, 3f, Direction.FORWARD);}

         else {
            telemetry.addData("no mineral found", "during first scan");
            telemetry.update();
            Log.i("[phoenix]: scan result", "no gold");
        }

    //StrafeWhileVisible(0.30f, 34, 5);
    //TurnToGold(0.14f, 175, Direction.COUNTERCLOCKWISE, imu, this, 10 );
    //scanGold(0.125f); // used 0.105 for calibartion, in FTC field, power = 0.12f, on carpet, 0.14f
    sleep(1000);
    //scanGold_Diagonal(0.16f);

    //drivetrain.Turn(), first turn 100 - 120 degree, can test proturn
            /*drivetrain.Turn(0.45f, 95, Direction.COUNTERCLOCKWISE, imu, this);
            // then turn to image
            sleep(1000);
            //drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, blueTarget, imu, this);
            //strafe to image
            drivetrain.StrafeToImage(0.25f, blueTarget, this);
            sleep(3000);

            if (tfod != null) { // now it is ok to shutdown tfod/vuforia
                tfod.deactivate();
                tfod.shutdown();


            // drive backward for certain distance. here can also test prodrive
            drivetrain.Drive(.4f, 58f, Direction.FORWARD);
            sleep(300);
            drivetrain.Drive(1.0f, 5, Direction.BACKWARD);//  drop marker
            drivetrain.Drive(.65f, 70, Direction.BACKWARD); // continue to drive
            */
        }


        public void scanGold ( float power){
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

            //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() <= 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    gold_loop_No = gold_loop_No + 1;
                                    currentTime = Math.round(runtime.milliseconds());
                                    telemetry.addData("1st Gold time ", currentTime);
                                    Log.i("[phoenix]:goldtime", Double.toString(currentTime));

                                    //if (currentTime < (1.5*secondHitTime)) {
                                    //goldMineralX = (int) recognition.getLeft();
                                    left_Gold = recognition.getLeft();
                                    right_Gold = recognition.getRight();
                                    // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                    center_Gold = (left_Gold + right_Gold) / 2f;
                                    current_Gold_Width = right_Gold - left_Gold;

                                    telemetry.addData("center of gold ", center_Gold);
                                    Log.i("[phoenix]:i_goldcenter", Float.toString(center_Gold));
                                    Log.i("[phoenix]:i_goldleft", Float.toString(left_Gold));
                                    Log.i("[phoenix]:i_goldright", Float.toString(right_Gold));
                                    Log.i("[phoenix]:i_width", Float.toString(current_Gold_Width));
                                    Log.i("[phoenix]:i_Pre Cnt ", Integer.toString(fr.getCurrentPosition()));

                                    // 2018-11-26 Ended the evening not sure what to put for the if statement below.  May need to do hand testing to test left, right and center values
                                    if ((center_Gold > 270f) && (center_Gold < 340f)) {   // here 300 can change to other numbers, perhaps 400 ?
                                        //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                        currentTime = Math.round(runtime.milliseconds());
                                        Log.i("[phoenix]:goldtime ", Double.toString(currentTime));
                                        drivetrain.StopAll();
                                        currentPosition = Math.abs(fr.getCurrentPosition());
                                        telemetry.addData("aft encoder ", currentPosition);
                                        Log.i("[phoenix]:aft encoder ", Integer.toString(currentPosition));
                                        telemetry.update();
                                        if (currentPosition < (firstHitEncoderCount + 900) && (gold_Found == 0)) {//900+312 = 1112 //300+firsthit = 612,about 3.5 inches extra(currentTime < (secondHitTime - 500)) { // this is first time hit
                                            // should try pro-strafe instead of regular strafe, to test navigation control

                                            StrafeWhileVisible(0.30f, 29f, 5);

// drivetrain.StopAll();
//                                              drivetrain.Strafe(0.4f, 25f, Direction.RIGHT);
//  sleep(500);
//                                            drivetrain.Strafe(0.4f, 25, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(500);
                                            gold_Found = 1; // gold is in A position
                                            currentPosition = 0;
                                            //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            telemetry.addData("at end of gold loop", "gold 1");
                                            Log.i("[phoenix]:goldloop", "at end of gold loop 1");
                                            telemetry.addData("gold frequency", gold_loop_No);
                                            Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                            telemetry.update();
                                        } else if (currentPosition > (secondHitEncoderCount + 700) && (gold_Found == 0)) { //1380 + 700 = 2180, 1380+600= 1980, 7 inches more, currentTime > (secondHitTime + 4000)) { // third time hit
                                            StrafeWhileVisible(0.30f, 5.5f, 5);
//                                            drivetrain.Strafe(0.4f, 5.5F, Direction.RIGHT);
//                                            drivetrain.StopAll();
//                                            sleep(500);
//                                            drivetrain.Strafe(0.4f, 5.5F, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(500);
                                            gold_Found = 3;  // gold is in C position
                                            currentPosition = 0;
                                            //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            telemetry.addData("at end of gold loop", "gold 3");
                                            Log.i("[phoenix]:goldloop", "at end of gold loop 3");
                                            telemetry.addData("gold frequency", gold_loop_No);
                                            Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                            telemetry.update();
                                        } else if (gold_Found == 0) {

                                            StrafeWhileVisible(0.30f, 14f, 5);
//                                            drivetrain.Strafe(0.4f, 14F, Direction.RIGHT);
//                                            drivetrain.StopAll();
//                                            sleep(1500);
//                                            drivetrain.Strafe(0.4f, 14F, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(1500);                                            //tfod.deactivate();
                                            gold_Found = 2;  // gold is in B position
                                            currentPosition = 0;
                                            //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                            telemetry.addData("at end of gold loop", "gold 2");
                                            Log.i("[phoenix]:goldloop", "at end of gold loop 2");
                                            telemetry.addData("gold frequency", gold_loop_No);
                                            Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                            telemetry.update();
                                        }
                                    }
                                }
                            }
                        }
                    }

                    telemetry.update();
                }
            }

            if (gold_Found == 0) {
                currentTime = Math.round(runtime.milliseconds());
                currentPosition = fr.getCurrentPosition();
                telemetry.addData("no gold found, current time ", currentTime);
                telemetry.addData("no gold found, current counter ", currentPosition);
                telemetry.addData("gold found flag ", gold_Found);

                Log.i("[phoenix]:NoGold, time ", Double.toString(currentTime));
                Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
                Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
                telemetry.update();
                drivetrain.StopAll();
            }

        }

        public void StrafeWhileVisible ( float power, float stop_distance, long report_time){

            double cur_Angle; // angle provided by TF method
            float cur_left_Gold;
            float cur_right_Gold;
            float cur_center_Gold = 0.0f;  // center coodinates of gold
            float cur_Gold_Width = 0.0f; // width of gold, to drive towards it..
            long log_time_interval = 100;
            //long report_Time;
            long base_Time;
            float lateral_power = 0;
            int stop_encoder_count;
            int base_encoder_count;
            int local_encoder_count = 0; // encoder count after method is called.
            telemetry.addData("in strafe test ", "strafe test");

            Log.i("[phoenix]:str-tst dist", Float.toString(stop_distance));
            //     Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
            //     Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
            telemetry.update();

            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            stop_encoder_count = Math.round(1120 * (stop_distance / (4f * 3.1416f)));
            base_Time = Math.round(runtime.milliseconds());
            base_encoder_count = Math.abs(fr.getCurrentPosition());
            Log.i("[phoenix]:bfr setpower", Float.toString(power));
            Log.i("[phoenix]:encoder", Float.toString(stop_encoder_count));
            Log.i("[phoenix]:base time", Long.toString(base_Time));
            // stafe to right..

            while (opModeIsActive() && (cur_Gold_Width < 460.0f) && (local_encoder_count<= stop_encoder_count)) { //600 is too much, 500 not engouth(local_encoder_count < stop_encoder_count)) {
                // report every 50 m

                //base_Time = Math.round(runtime.milliseconds());
                local_encoder_count = Math.abs(fr.getCurrentPosition()) - base_encoder_count;

                Log.i("[phoenix]:opAct l-enco", Integer.toString(local_encoder_count));

                fl.setPower(power + lateral_power);
                fr.setPower(-power + lateral_power);
                bl.setPower(-power + lateral_power);
                br.setPower(power + lateral_power);
                Log.i("[phoenix]:aft setpower", "setpower");

                if (tfod != null) {
                    Log.i("[phoenix]:check tfod", "tfod");
                    List<Recognition> updatedRecognitions_2 = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions_2 != null) {
                        Log.i("[phoenix]:updatedReg", "updateR");
                        if (updatedRecognitions_2.size() <= 3) {
                            for (Recognition recognition : updatedRecognitions_2) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    Log.i("[phoenix]: see gold", "gold");
                                    cur_left_Gold = recognition.getLeft();
                                    cur_right_Gold = recognition.getRight();

                                    //cur_Angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                                    // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                    cur_center_Gold = (cur_left_Gold + cur_right_Gold) / 2f;
                                    cur_Gold_Width = cur_right_Gold - cur_left_Gold;
                                    Log.i("[phoenix]:s_goldwidth_a", Float.toString(cur_Gold_Width));


                                    if ((Math.round(runtime.milliseconds()) - base_Time) > report_time) {

                                        telemetry.addData("center of gold ", cur_center_Gold);
                                        Log.i("[phoenix]:s_goldcenter", Float.toString(cur_center_Gold));
                                        //Log.i("[phoenix]:s_goldleft", Float.toString(cur_left_Gold));
                                        //Log.i("[phoenix]:s_goldright", Float.toString(cur_right_Gold));
                                        Log.i("[phoenix]:s_goldwidth", Float.toString(cur_Gold_Width));
                                        //Log.i("[phoenix]:s_angle", Double.toString(cur_Angle));
                                        // reset basetime
                                        base_Time = Math.round(runtime.milliseconds());
                                        Log.i("[phoenix]:s_cur time", Long.toString(base_Time));
                                        //local_encoder_count = Math.abs(fr.getCurrentPosition()) - base_encoder_count;
                                        Log.i("[phoenix]:s_encoder", Integer.toString(local_encoder_count));

                                    }

                                    // adjustment

                                    if (cur_center_Gold < 180f) {
                                        lateral_power = 0.08f;
                                    } else if (cur_center_Gold > 350) {
                                        lateral_power = -0.10f;
                                    } else {
                                        lateral_power = 0;
                                    }
                                    Log.i("[phoenix]:s_lat_power", Float.toString(lateral_power));

                                }
                            }
                        }
                    }

                }
            }

            drivetrain.StopAll();
        }

    public void scanGold_Diagonal ( float power){
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
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() <= 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                gold_loop_No = gold_loop_No + 1;
                                currentTime = Math.round(runtime.milliseconds());
                                telemetry.addData("1st Gold time ", currentTime);
                                Log.i("[phoenix]:goldtime", Double.toString(currentTime));

                                //if (currentTime < (1.5*secondHitTime)) {
                                //goldMineralX = (int) recognition.getLeft();
                                left_Gold = recognition.getLeft();
                                right_Gold = recognition.getRight();
                                // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                center_Gold = (left_Gold + right_Gold) / 2f;
                                current_Gold_Width = right_Gold - left_Gold;

                                telemetry.addData("center of gold ", center_Gold);
                                Log.i("[phoenix]:i_goldcenter", Float.toString(center_Gold));
                                Log.i("[phoenix]:i_goldleft", Float.toString(left_Gold));
                                Log.i("[phoenix]:i_goldright", Float.toString(right_Gold));
                                Log.i("[phoenix]:i_width", Float.toString(current_Gold_Width));
                                Log.i("[phoenix]:i_Pre Cnt ", Integer.toString(fr.getCurrentPosition()));

                                // 2018-11-26 Ended the evening not sure what to put for the if statement below.  May need to do hand testing to test left, right and center values
                                if ((center_Gold > 270f) && (center_Gold < 340f)) {   // here 300 can change to other numbers, perhaps 400 ?
                                    //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                    currentTime = Math.round(runtime.milliseconds());
                                    Log.i("[phoenix]:goldtime ", Double.toString(currentTime));
                                    drivetrain.StopAll();
                                    currentPosition = Math.abs(fr.getCurrentPosition());
                                    telemetry.addData("aft encoder ", currentPosition);
                                    Log.i("[phoenix]:aft encoder ", Integer.toString(currentPosition));
                                    telemetry.update();
                                    if (currentPosition < (firstHitEncoderCount + 900) && (gold_Found == 0)) {//900+312 = 1112 //300+firsthit = 612,about 3.5 inches extra(currentTime < (secondHitTime - 500)) { // this is first time hit
                                        // should try pro-strafe instead of regular strafe, to test navigation control

                                        //StrafeWhileVisible(0.30f, 29f, 5);

// drivetrain.StopAll();
                                              drivetrain.Strafe(0.4f, 6.5f, Direction.RIGHT);
                                              sleep(500);
                                              drivetrain.Strafe(0.4f, 4.5f, Direction.LEFT);
                                              drivetrain.StopAll();
                                              sleep(500);
                                        gold_Found = 2; // gold is in B position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        telemetry.addData("at end of gold loop", "gold 2");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 2");
                                        telemetry.addData("gold frequency", gold_loop_No);
                                        Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                        telemetry.update();
                                    } else if ((gold_Found == 0)) { //1380 + 700 = 2180, 1380+600= 1980, 7 inches more, currentTime > (secondHitTime + 4000)) { // third time hit
                                        //StrafeWhileVisible(0.30f, 5.5f, 5);
                                            drivetrain.Strafe(0.4f, 6.5F, Direction.RIGHT);
                                            drivetrain.StopAll();
                                            sleep(500);
                                            drivetrain.Strafe(0.4f, 4.5F, Direction.LEFT);
                                            drivetrain.StopAll();
                                            sleep(500);
                                        gold_Found = 3;  // gold is in C position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        telemetry.addData("at end of gold loop", "gold 3");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 3");
                                        telemetry.addData("gold frequency", gold_loop_No);
                                        Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                        telemetry.update();
                                    } /*else if (gold_Found == 0) {

                                        StrafeWhileVisible(0.30f, 14f, 5);
//                                            drivetrain.Strafe(0.4f, 14F, Direction.RIGHT);
//                                            drivetrain.StopAll();
//                                            sleep(1500);
//                                            drivetrain.Strafe(0.4f, 14F, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(1500);                                            //tfod.deactivate();
                                        gold_Found = 2;  // gold is in B position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        telemetry.addData("at end of gold loop", "gold 2");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 2");
                                        telemetry.addData("gold frequency", gold_loop_No);
                                        Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                        telemetry.update();
                                    }*/
                                }
                            }
                        }
                    }
                }

                telemetry.update();
            }
        }

        if (gold_Found == 0) {
            currentTime = Math.round(runtime.milliseconds());
            currentPosition = fr.getCurrentPosition();
            telemetry.addData("no gold found, current time ", currentTime);
            telemetry.addData("no gold found, current counter ", currentPosition);
            telemetry.addData("gold found flag ", gold_Found);

            Log.i("[phoenix]:NoGold, time ", Double.toString(currentTime));
            Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
            Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
            telemetry.update();
            drivetrain.StopAll();
        }

    }

    public void TurnToGold ( float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode,long report_time){ // turn to A, B mineral

            long currentTime;
            int turnAngle;
            int gold_Found = 0;
            float distanceFromStart = 0f;
            int firstHitEncoderCount = Math.round(1120f * firstHitDistance / (4 * 3.1416f)); //89*3.5 = 312, 0.8 is a factor
            int secondHitEncoderCount = Math.round(1120f * secondHitDistance / (4 * 3.1416f)); // 89*15.5 = 1380
            int thirdEncoderCount = Math.round(1120f * thirdHitDistance / (4 * 3.1416f)); // 89*27.5 = 2448
            float startAngle;
            int currentPosition = 0;
            int gold_loop_No = 0;


            float targetAngle;
            float currentAngle;
            float actualPower = power;
            float stoppingAngle = 0;

            Orientation startOrientation = imu.resetAndStart(d);

            double estimatedAngle;
            float left_Gold;
            float right_Gold;
            float center_Gold;  // center coodinates of gold
            float current_Gold_Width; // width of gold, to drive towards it.

            runtime.reset(); // need to use time for tracking minerals instead of just  number of objects
            currentTime = Math.round(runtime.milliseconds());

            telemetry.addData("before moving time", currentTime);
            Log.i("[phoenix]:pretime", Double.toString(currentTime));

            //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (d == Direction.COUNTERCLOCKWISE) {

                actualPower = power;

                targetAngle = startOrientation.firstAngle + angle;
                currentAngle = startOrientation.firstAngle;
                startAngle = currentAngle;

                while ((gold_Found == 0) && (currentAngle < targetAngle) && opMode.opModeIsActive()) {

                    currentAngle = imu.getAngularOrientation().firstAngle;

                    opMode.telemetry.addData("start:", startOrientation.firstAngle);
                    opMode.telemetry.addData("current:", currentAngle);
                    opMode.telemetry.addData("target:", targetAngle);
                    opMode.telemetry.update();


                    //fl.setPower(actualPower);  // first calibrate time on floor..0.12 at dr warners, 0.14 at carpet

                    fl.setPower(-(actualPower));
                    fr.setPower(actualPower);
                    bl.setPower(-(actualPower));
                    br.setPower(actualPower);

                    currentTime = Math.round(runtime.milliseconds());

                    if (tfod != null) {

                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() <= 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        gold_loop_No = gold_loop_No + 1;
                                        currentTime = Math.round(runtime.milliseconds());
                                        telemetry.addData("1st Gold time ", currentTime);
                                        Log.i("[phoenix]:goldtime", Double.toString(currentTime));

                                        //if (currentTime < (1.5*secondHitTime)) {
                                        //goldMineralX = (int) recognition.getLeft();
                                        left_Gold = recognition.getLeft();
                                        right_Gold = recognition.getRight();
                                        // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                        center_Gold = (left_Gold + right_Gold) / 2f;
                                        current_Gold_Width = right_Gold - left_Gold;

                                        telemetry.addData("center of gold ", center_Gold);
                                        Log.i("[phoenix]:i_goldcenter", Float.toString(center_Gold));
                                        Log.i("[phoenix]:i_goldleft", Float.toString(left_Gold));
                                        Log.i("[phoenix]:i_goldright", Float.toString(right_Gold));
                                        Log.i("[phoenix]:i_width", Float.toString(current_Gold_Width));
                                        Log.i("[phoenix]:i_Pre Cnt ", Integer.toString(fr.getCurrentPosition()));

                                        // 2018-11-26 Ended the evening not sure what to put for the if statement below.  May need to do hand testing to test left, right and center values
                                        if ((center_Gold > 80f) && (center_Gold < 640f)) {   // here 300 can change to other numbers, perhaps 400 ?
                                            //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                            currentTime = Math.round(runtime.milliseconds());
                                            Log.i("[phoenix]:goldtime ", Double.toString(currentTime));
                                            drivetrain.StopAll();
                                            sleep(5000);
                                            currentPosition = Math.abs(fr.getCurrentPosition());
                                            telemetry.addData("aft encoder ", currentPosition);
                                            Log.i("[phoenix]:aft encoder ", Integer.toString(currentPosition));
                                            telemetry.update();
                                            if (currentAngle < (startAngle + firstStopAngle + 20) && (gold_Found == 0)) {
                                                // should try pro-strafe instead of regular strafe, to test navigation control

                                                StrafeWhileVisible(0.30f, 19f, 5);

// drivetrain.StopAll();
//                                              drivetrain.Strafe(0.4f, 25f, Direction.RIGHT);
//  sleep(500);
//                                            drivetrain.Strafe(0.4f, 25, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(500);
                                                gold_Found = 1; // gold is in A position
                                                currentPosition = 0;
                                                //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                telemetry.addData("at end of gold loop", "gold 1");
                                                Log.i("[phoenix]:goldloop", "at end of gold loop 1");
                                                telemetry.addData("gold frequency", gold_loop_No);
                                                Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                                telemetry.update();
                                            } else if (currentAngle < (startAngle + secondStopAngle + 20) && (gold_Found == 0)) {
                                                StrafeWhileVisible(0.30f, 5.5f, 5);
//                                            drivetrain.Strafe(0.4f, 5.5F, Direction.RIGHT);
//                                            drivetrain.StopAll();
//                                            sleep(500);
//                                            drivetrain.Strafe(0.4f, 5.5F, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(500);
                                                gold_Found = 3;  // gold is in C position
                                                currentPosition = 0;
                                                //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                telemetry.addData("at end of gold loop", "gold 3");
                                                Log.i("[phoenix]:goldloop", "at end of gold loop 3");
                                                telemetry.addData("gold frequency", gold_loop_No);
                                                Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                                telemetry.update();
                                            } else if (gold_Found == 0) {   // gold is at third position..

                                                StrafeWhileVisible(0.30f, 19f, 5);
//
//                                             drivetrain.Strafe(0.4f, 14F, Direction.RIGHT);
//                                            drivetrain.StopAll();
//                                            sleep(1500);
//                                            drivetrain.Strafe(0.4f, 14F, Direction.LEFT);
//                                            drivetrain.StopAll();
//                                            sleep(1500);                                            //tfod.deactivate();
                                                gold_Found = 2;  // gold is in B position
                                                currentPosition = 0;
                                                //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                                telemetry.addData("at end of gold loop", "gold 2");
                                                Log.i("[phoenix]:goldloop", "at end of gold loop 2");
                                                telemetry.addData("gold frequency", gold_loop_No);
                                                Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                                telemetry.update();
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        telemetry.update();

                    }
                }

                if (gold_Found == 0) {
                    currentTime = Math.round(runtime.milliseconds());
                    currentPosition = fr.getCurrentPosition();
                    telemetry.addData("no gold found, current time ", currentTime);
                    telemetry.addData("no gold found, current counter ", currentPosition);
                    telemetry.addData("gold found flag ", gold_Found);

                    Log.i("[phoenix]:NoGold, time ", Double.toString(currentTime));
                    Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
                    Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
                    telemetry.update();
                    drivetrain.StopAll();
                }

            } else {
            } // direction clockwise..
        }



        public Integer ScanFirstMineral() {

            int scanResult = 0;

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                if (tfod != null) {
                    tfod.activate();
                }

                while (opModeIsActive() && (scanResult == 0)) {

                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() <= 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                        telemetry.addData("Gold Mineral Position", goldMineralX);
                                        Log.i("[phoenix]:Gold:", Integer.toString(goldMineralX));
                                        scanResult = 1;
                                    } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        silverMineral1X = (int) recognition.getLeft();
                                        telemetry.addData("Silver Mineral Position", goldMineralX);
                                        Log.i("[phoenix]:Silver ", Integer.toString(goldMineralX));
                                        scanResult = 2;
                                    } else {
                                        telemetry.addData("no mineral found", 0);
                                        Log.i("[phoenix]:No Mineral:", Integer.toString(0));
                                        scanResult = 0;
                                    }
                                }

                            }
                            telemetry.update();
                        }
                    }
                }
            }

            return scanResult;
        }

    public Integer DriveToScanFirstMineral(float power, Direction d, LinearOpMode opMode) {

        int scanResult = 0;
        float actualPower = power;

        if (d == Direction.BACKWARD) {
            actualPower = - power;
        }

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && (scanResult == 0)) {

                fl.setPower(actualPower);  // first calibrate time on floor..0.12 at dr warners, 0.14 at carpet
                fr.setPower(actualPower);
                bl.setPower(actualPower);
                br.setPower(actualPower);

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() <= 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("Gold Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Gold:", Integer.toString(goldMineralX));
                                    scanResult = 1;
                                    drivetrain.StopAll();
                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    telemetry.addData("Silver Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Silver ", Integer.toString(goldMineralX));
                                    scanResult = 2;
                                    drivetrain.StopAll();
                                } else {
                                    telemetry.addData("no mineral found", 0);
                                    Log.i("[phoenix]:No Mineral:", Integer.toString(0));
                                    scanResult = 0;
                                }
                            }

                        }
                        telemetry.update();
                    }
                }
            }
        }

        return scanResult;
    }


}
