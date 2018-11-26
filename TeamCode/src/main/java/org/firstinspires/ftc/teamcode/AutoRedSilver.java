package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="Auto Red Silver", group="none")
public class AutoRedSilver extends AutoBase {
    private ElapsedTime runtime = new ElapsedTime();
    private static final long  firstHitTime = 1250; // this is from calibration, it is time to detect first object
    private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    private static final long thirdHitTime = 12000;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        scanGold();









    }

    public void scanGold(){
        long currentTime;
        int turnAngle;
        int gold_Found = 0;
        Double estimatedAngle;
        Float center_Gold;  // center coodinates of gold
        Float current_Gold_Width; // width of gold, to drive towards it..

        runtime.reset(); // need to use time for tracking minerals instead of just  number of objects

        while (gold_Found == 0 && opModeIsActive()) {

            fl.setPower(0.14f);  // first calibrate time on floor..0.12 at dr warners, 0.14 at carpet
            fr.setPower(0.14f);
            bl.setPower(0.14f);
            br.setPower(0.14f);

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
                                currentTime = Math.round(runtime.milliseconds());
                                telemetry.addData("1st Gold time ", currentTime);
                                Log.i("1st Gold time ", Double.toString(currentTime));

                                //if (currentTime < (1.5*secondHitTime)) {
                                goldMineralX = (int) recognition.getLeft();

                                // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                center_Gold = (recognition.getLeft() + recognition.getRight()) / 2f;
                                telemetry.addData("center of gold ", center_Gold);
                                Log.i("center of gold", Float.toString(center_Gold));
                                // here decide if gold is left of center or right of center, to tell robot move right/left.
                                // if center_Gold < 360 or > 360...
                                // for now, just knock it off..
                                if (center_Gold < 300f) {   // here 300 can change to other numbers, perhaps 400 ?
                                    //currentTime = Math.round(runtime.milliseconds()); // use this to control position
                                    currentTime = Math.round(runtime.milliseconds());
                                    drivetrain.StopAll();
                                    if (currentTime < (secondHitTime - 500)) { // this is first time hit

                                        drivetrain.Strafe(0.4f, 34, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        drivetrain.Strafe(0.4f, 34, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        //tfod.deactivate();
                                        //tfod.shutdown();
                                        gold_Found = 1; // gold is in A position
                                        telemetry.addData("at end of gold loop", "gold 1");
                                        Log.i("gold loop", "at end of gold loop");
                                        telemetry.update();
                                    } else if (currentTime > (secondHitTime + 4000)) { // third time hit
                                        //drivetrain.StopAll();
                                        drivetrain.Strafe(0.4f, 4F, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        drivetrain.Strafe(0.4f, 4F, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        //tfod.deactivate();
                                        //tfod.shutdown();
                                        gold_Found = 3;  // gold is in C position
                                        telemetry.addData("at end of gold loop", "gold 2");
                                        Log.i("gold loop", "at end of gold loop");
                                        telemetry.update();
                                    } else {
                                        drivetrain.Strafe(0.4f, 16F, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(1500);
                                        drivetrain.Strafe(0.4f, 15F, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(1500);                                            //tfod.deactivate();
                                        //tfod.shutdown();
                                        gold_Found = 2;  // gold is in B position
                                        telemetry.addData("at end of gold loop", "gold 3");
                                        Log.i("gold loop", "at end of gold loop");
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
