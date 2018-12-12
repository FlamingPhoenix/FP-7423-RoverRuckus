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
@Autonomous(name="Archmere BlueDepot", group="none")
public class AutoBlueDepot extends AutoBase {
    private ElapsedTime runtime = new ElapsedTime();
    private static final long  firstHitTime = 1250; // this is from calibration, it is time to detect first object
    private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    private static final long thirdHitTime = 12000;

    private static final float  firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
    private static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
    private static final float thirdHitDistance = 27.5f;


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        scanGold(0.12f);
        sleep(300);
        //drivetrain.Turn(), first turn 100 - 120 degree, can test proturn
        drivetrain.Turn(0.35f, 95, Direction.COUNTERCLOCKWISE, imu, this);
        // then turn to image
        sleep(1000);
        //drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, frontTarget, imu, this);
        //strafe to image
        drivetrain.StrafeToImage(0.25f, frontTarget, this);

        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();
        }

        // drive backward for certain distance. here can also test prodrive
        drivetrain.Drive(.4f, 58f, Direction.BACKWARD);
        sleep(300);
        drivetrain.Drive(1.0f, 5, Direction.FORWARD);//  drop marker
        drivetrain.Drive(.65f, 70, Direction.FORWARD); // continue to drive





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

        Double estimatedAngle;
        Float center_Gold;  // center coodinates of gold
        Float current_Gold_Width; // width of gold, to drive towards it..
        float actualPower = power;

        runtime.reset(); // need to use time for tracking minerals instead of just  number of objects
        currentTime = Math.round(runtime.milliseconds());

        telemetry.addData("before moving time", currentTime);
        Log.i("[phoenix]:pretime", Double.toString(currentTime));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (gold_Found == 0 && opModeIsActive()) {


            fl.setPower(actualPower);  // first calibrate time on floor..0.12 at dr warners, 0.14 at carpet
            fr.setPower(actualPower);
            bl.setPower(actualPower);
            br.setPower(actualPower);


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
                                currentTime = Math.round(runtime.milliseconds());
                                telemetry.addData("1st Gold time ", currentTime);
                                Log.i("[phoenix]:goldtime", Double.toString(currentTime));

                                //if (currentTime < (1.5*secondHitTime)) {
                                goldMineralX = (int) recognition.getLeft();

                                // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                center_Gold = (recognition.getLeft() + recognition.getRight()) / 2f;
                                telemetry.addData("center of gold ", center_Gold);
                                Log.i("[phoenix]: goldcenter", Float.toString(center_Gold));
                                Log.i("[phoenix]: encodeCnt ", Integer.toString(fl.getCurrentPosition()));
                                // 2018-11-26 Ended the evening not sure what to put for the if statement below.  May need to do hand testing to test left, right and center values
                                if ((center_Gold > 270f) && (center_Gold < 340f)  ) {   // here 300 can change to other numbers, perhaps 400 ?
                                    //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                    currentTime = Math.round(runtime.milliseconds());
                                    Log.i("[phoenix]: goldtime ", Double.toString(currentTime));
                                    drivetrain.StopAll();
                                    currentPosition = Math.abs(fr.getCurrentPosition());
                                    telemetry.addData("cur encoder ", currentPosition);
                                    Log.i("[phoenix]: cur encoder", Integer.toString(currentPosition));
                                    telemetry.update();
                                    if (currentPosition < (firstHitEncoderCount + 450) && (gold_Found  == 0)) {//612,about 3.5 inches extra(currentTime < (secondHitTime - 500)) { // this is first time hit
                                        // should try prostrafe instead of regular strafe, to test navigation control
                                        drivetrain.Strafe(0.4f, 30, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        drivetrain.Strafe(0.4f, 30, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        gold_Found = 1; // gold is in A position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        telemetry.addData("at end of gold loop", "gold 1");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 1");
                                        telemetry.update();
                                    }
                                    else if (currentPosition > (secondHitEncoderCount + 600) && (gold_Found  == 0)) { //1980, 7 inches more, currentTime > (secondHitTime + 4000)) { // third time hit
                                        //drivetrain.StopAll();
                                        drivetrain.Strafe(0.4f, 7F, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        drivetrain.Strafe(0.4f, 7F, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        gold_Found = 3;  // gold is in C position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        telemetry.addData("at end of gold loop", "gold 3");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 3");
                                        telemetry.update();
                                    }
                                    else {
                                        drivetrain.Strafe(0.4f, 14F, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(1500);
                                        drivetrain.Strafe(0.4f, 14F, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(1500);                                            //tfod.deactivate();
                                        gold_Found = 2;  // gold is in B position
                                        currentPosition = 0;
                                        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                        telemetry.addData("at end of gold loop", "gold 2");
                                        Log.i("[phoenix]:goldloop", "at end of gold loop 2");
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
    }
}
