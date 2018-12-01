package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

//@Disabled
@Autonomous(name="Test Scan Gold", group="none")

public class ScanGoldTest extends AutoBase {

        private ElapsedTime runtime = new ElapsedTime();
        private static final long  firstHitTime = 1250; // this is from calibration, it is time to detect first object
        private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
        private static final long thirdHitTime = 12000;

        private static final float  firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
        private static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
        private static final float thirdHitDistance = 27.5f;

        //List<Recognition> updatedRecognitions;
        //Recognition recognition;

        @Override
        public void runOpMode() throws InterruptedException {

            initialize();
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Wait for the start button
            telemetry.addData(">", "Press Start to test ScanGold." );
            telemetry.update();

            waitForStart();

            sleep(500);
            StrafeWhileVisible(0.30f, 34, 5);
            //scanGold(0.125f); // used 0.105 for calibartion, in FTC field, power = 0.12f, on carpet, 0.14f
            sleep(300);

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
            }

            // drive backward for certain distance. here can also test prodrive
            drivetrain.Drive(.4f, 58f, Direction.FORWARD);
            sleep(300);
            drivetrain.Drive(1.0f, 5, Direction.BACKWARD);//  drop marker
            drivetrain.Drive(.65f, 70, Direction.BACKWARD); // continue to drive
            */




        }

        public void scanGold(float power){
            long currentTime;
            int turnAngle;
            int gold_Found = 0;
            float distanceFromStart = 0f;
            int firstHitEncoderCount = Math.round(1120f*firstHitDistance/(4*3.1416f)); //89*3.5 = 312, 0.8 is a factor
            int secondHitEncoderCount = Math.round(1120f*secondHitDistance/(4*3.1416f)); // 89*15.5 = 1380
            int thirdEncoderCount = Math.round(1120f*thirdHitDistance/(4*3.1416f)); // 89*27.5 = 2448
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
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (gold_Found == 0 && opModeIsActive() && (currentTime < (thirdHitTime-2000))) { // 12000-2000 = 10000


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
                                    if ((center_Gold > 270f) && (center_Gold < 340f)  ) {   // here 300 can change to other numbers, perhaps 400 ?
                                        //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                        currentTime = Math.round(runtime.milliseconds());
                                        Log.i("[phoenix]:goldtime ", Double.toString(currentTime));
                                        drivetrain.StopAll();
                                        currentPosition = Math.abs(fr.getCurrentPosition());
                                        telemetry.addData("aft encoder ", currentPosition);
                                        Log.i("[phoenix]:aft encoder ", Integer.toString(currentPosition));
                                        telemetry.update();
                                        if (currentPosition < (firstHitEncoderCount + 900) && (gold_Found  == 0)) {//900+312 = 1112 //300+firsthit = 612,about 3.5 inches extra(currentTime < (secondHitTime - 500)) { // this is first time hit
                                            // should try pro-strafe instead of regular strafe, to test navigation control

                                            StrafeWhileVisible(0.30f,29f, 5);

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
                                        }
                                        else if (currentPosition > (secondHitEncoderCount + 700) && (gold_Found  == 0)) { //1380 + 700 = 2180, 1380+600= 1980, 7 inches more, currentTime > (secondHitTime + 4000)) { // third time hit
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
                                        }
                                        else if (gold_Found  == 0) {

                                            StrafeWhileVisible(0.30f,14f, 5);
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
            drivetrain.StopAll();}

        }

        public void StrafeWhileVisible(float power, float stop_distance, long report_time) {

            double cur_Angle; // angle provided by TF method
            float cur_left_Gold;
            float cur_right_Gold;
            float cur_center_Gold = 0.0f;  // center coodinates of gold
            float cur_Gold_Width=0.0f; // width of gold, to drive towards it..
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
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            stop_encoder_count = Math.round(1120*(stop_distance/(4f*3.1416f)));
            base_Time = Math.round(runtime.milliseconds());
            base_encoder_count = Math.abs(fr.getCurrentPosition());
            Log.i("[phoenix]:bfr setpower", Float.toString(power));
            Log.i("[phoenix]:encoder", Float.toString(stop_encoder_count));
            Log.i("[phoenix]:base time", Long.toString(base_Time));
            // stafe to right..

            while (opModeIsActive() && (cur_Gold_Width < 460.0f)) { //600 is too much, 500 not engouth(local_encoder_count < stop_encoder_count)) {
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
                                             lateral_power = 0.08f; }
                                        else if (cur_center_Gold > 350)   {
                                             lateral_power = -0.10f;}
                                        else { lateral_power = 0; }
                                        Log.i("[phoenix]:s_lat_power", Float.toString(lateral_power));

                                }
                            }
                        }
                    }

                }
            }

            drivetrain.StopAll();
        }
    }
