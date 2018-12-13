package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name="Test Scan Gold New", group="none")// this approach do not scan while drive, will just stop and scan.

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
    private static final int originalAngle = 90;
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
        sleep(200);
        //drivetrain.Strafe(0.25f, 7.0f, Direction.RIGHT );
        //sleep(1000);
        drivetrain.Turn(0.25f, 46, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(200);
        //drivetrain.Turn(0.40f, 70, Direction.COUNTERCLOCKWISE, imu, this);


        //detectionOutcome = DriveToScanFirstMineral(0.11f, Direction.FORWARD, this); // 0.11 at Dr Warner, 0.15 at carpets
        detectionOutcome = ScanFirstMineralSimple();
        telemetry.addData("mineral scan outcome", detectionOutcome);
        Log.i("[phoenix]:min outcome", Integer.toString(detectionOutcome));

        sleep(200);
        if (detectionOutcome == 1) { //ScanFirstMineral() == 1
            telemetry.addData("Gold found", "during first scan");
            Log.i("[phoenix]:gold detected", "found gold");
            sleep(200);
            StrafeWhileVisible(0.4f, 10f, 520, 10, this);
            telemetry.addData("Gold aft straf", "after strafe");
            Log.i("[phoenix]:gold aft str", "after strafe");
            sleep(200);
            drivetrain.Strafe(0.3f, 15f, Direction.LEFT);
            drivetrain.Turn(0.4f, 32, Direction.COUNTERCLOCKWISE, imu, this); // shouid be 45, compensate for wheel issue
            drivetrain.Drive(0.3f, 10f, Direction.FORWARD);
        } else if (detectionOutcome == 2) { //ScanFirstMineral() == 2
            telemetry.addData("Silver found", "during first scan");
            Log.i("[phoenix]:Silv detected", "found silver");
            sleep(200);
            // drivetrain.Strafe(.3F, 6.5f, Direction.LEFT);
            //sleep(1000);
            drivetrain.Turn(0.2f, 35, Direction.COUNTERCLOCKWISE, imu, this); // should be 45, compensate for wheels issue
            telemetry.addData("Silver aft turn", "after turn");
            Log.i("[phoenix]:Silv aft turn", "aft turn");
            sleep(200);
            Log.i("[phoenix]:b 2nd scan", "true");
            detectionOutcome = ScanFirstMineralSimple();
            Log.i("[phoenix]:a 2nd scan", "true");
            sleep(200);
            if (detectionOutcome == 1) {
                drivetrain.Turn(.2F, originalAngle, Direction.CLOCKWISE, imu, this);
                sleep(200);
                drivetrain.Drive(0.2f, 20f, Direction.FORWARD);
                sleep(200);
                Log.i("[phoenix]:second gold", "true");
                drivetrain.Drive(0.2f, 20f, Direction.BACKWARD);
                drivetrain.Turn(0.2f, (originalAngle + 35), Direction.COUNTERCLOCKWISE, imu, this);
            } else {
                drivetrain.Drive(0.2f, 2.5f, Direction.FORWARD);
                sleep(200);
                drivetrain.Turn(0.2f, 43, Direction.CLOCKWISE, imu, this);
                sleep(200);
                drivetrain.Drive(0.2f, 20f, Direction.FORWARD);
                sleep(200);
                drivetrain.Drive(0.2f, 20f, Direction.BACKWARD);

                Log.i("[phoenix]:third gold", "true");
            }
            sleep(200);
            //scanGold_Diagonal(0.11f);
            //drivetrain.Drive(0.3f, 3f, Direction.FORWARD);}
        } else {
            telemetry.addData("no mineral found", "during first scan");
            telemetry.update();
            Log.i("[phoenix]: scan result", "no gold");
        }

        //StrafeWhileVisible(0.30f, 34, 5);
        //TurnToGold(0.14f, 175, Direction.COUNTERCLOCKWISE, imu, this, 10 );
        //scanGold(0.125f); // used 0.105 for calibartion, in FTC field, power = 0.12f, on carpet, 0.14f
        sleep(500);
        //scanGold_Diagonal(0.16f);

        //drivetrain.Turn(), first turn 100 - 120 degree, can test proturn
        //drivetrain.Turn(0.45f, 95, Direction.COUNTERCLOCKWISE, imu, this);
        // then turn to image
        //sleep(1000);
        drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, blueTarget, imu, this);
        //strafe to image
        drivetrain.StrafeToImage(0.25f, redTarget, this);
        sleep(500);

        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();


            // drive forward for to depot, it was 58
            drivetrain.Drive(.4f, 56f, Direction.FORWARD);
            sleep(300);
            // drop marker
            markerHook.setPosition(0.1);


            sleep(300);
            drivetrain.Drive(.65f, 75, Direction.BACKWARD); // continue to drive to crater

            // end of auto routine.
        }

    }


        public Integer ScanFirstMineralSimple() {
            // this version of scan first mineral, will return Gold, OR if not GOLD, will not confirm but just assume Silver
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
                                    /*} else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        silverMineral1X = (int) recognition.getLeft();
                                        telemetry.addData("Silver Mineral Position", goldMineralX);
                                        Log.i("[phoenix]:Silver ", Integer.toString(goldMineralX));
                                        scanResult = 2;*/
                                    } else {
                                        silverMineral1X = (int) recognition.getLeft();
                                        telemetry.addData("assume Silver Mineral Position", silverMineral1X);
                                        Log.i("[phoenix]:assume Sil", Integer.toString(silverMineral1X));
                                        scanResult = 2;
                                        //telemetry.addData("no mineral found", 0);
                                        //Log.i("[phoenix]:No Mineral:", Integer.toString(0));
                                        //scanResult = 0;
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
