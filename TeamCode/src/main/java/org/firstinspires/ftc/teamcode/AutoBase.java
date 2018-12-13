package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.MineralPositionViewModel;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

public abstract class AutoBase extends LinearOpMode {
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    protected static final String VUFORIA_KEY = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";

    protected static final float firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
    protected static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
    protected static final float thirdHitDistance = 27.5f;

    protected static final long firstHitTime = 1250; // this is from calibration, it is time to detect first object
    protected static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    protected static final long thirdHitTime = 12000;

    protected ElapsedTime runtime = new ElapsedTime();

    protected enum MineralPosition {LEFT, CENTER, RIGHT, UNKNOWN}

    protected DcMotor fl;
    protected DcMotor fr;
    protected DcMotor bl;
    protected DcMotor br;
    protected DriveTrain drivetrain;
    protected MyBoschIMU imu;

    protected VuforiaTrackable backTarget;
    protected VuforiaTrackable frontTarget;
    protected VuforiaTrackable redTarget;
    protected VuforiaTrackable blueTarget;

    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    protected Servo markerHook;

    public void initialize() {

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new DriveTrain(fl, fr, bl, br, this);
        // boolean drivetrain.robotWork = true;

        markerHook = hardwareMap.servo.get("markerhook");
        ServoControllerEx primaryController = (ServoControllerEx) markerHook.getController();
        int grabberServoPort = markerHook.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(899, 2150);
        primaryController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        //setting up variable "markerHook" to the hardware of the robot

        // WARNING!!!  Do not enable this line unless you are specifically testing the marker hook because when it is in position 1,
        // It is using power the entire time and the servo will overheat.
        markerHook.setPosition(0.9);

        imu = new MyBoschIMU(hardwareMap);

        imu.initialize(new BNO055IMU.Parameters());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();

        blueTarget = rover.get(0);
        blueTarget.setName("bluetarget");
        redTarget = rover.get(1);
        redTarget.setName("redTarget");
        frontTarget = rover.get(2);
        frontTarget.setName("frontTarget");
        backTarget = rover.get(3);
        backTarget.setName("backTarget");

        initTfod();
        if (tfod != null)
            tfod.activate();

    }

    protected MineralPositionViewModel GetMineralPositions()
    {
        MineralPositionViewModel mpvm = new MineralPositionViewModel();


        return mpvm;
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public MineralPosition getGoldPosition(List<Recognition> recognitions) {
        int goldMineralX = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;

        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralX = (int) recognition.getLeft();
            }
            else if (silverMineral1X == -1) {
                silverMineral1X = (int) recognition.getLeft();
            }
            else {
                silverMineral2X = (int) recognition.getLeft();
            }
        }

        if (goldMineralX == -1)
            return MineralPosition.UNKNOWN;
        else if (goldMineralX != -1)
        {
            if (goldMineralX < silverMineral1X)
                return MineralPosition.LEFT;
            else if (goldMineralX > silverMineral1X)
                return MineralPosition.RIGHT;
        }

        return MineralPosition.UNKNOWN;
    }

    public MineralPositionViewModel getMineralPositions(VuforiaTrackable image)
    {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) image.getListener();

        OpenGLMatrix pose = imageListener.getPose();
        double distance = (pose.getTranslation().get(2)) * 0.0393701;

        Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
        OpenGLMatrix adjustedPose = pose.multiplied(rotationMatrix);
        Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        float imageAngle = Math.abs(adjustedOrientation.secondAngle);
        double actualDistance = ((distance) / (Math.cos(Math.toRadians(imageAngle))));
        double innerHeight = (24 * Math.sqrt(2) * Math.sin(Math.toRadians(45 - imageAngle)));
        double d1 = (24 * Math.sqrt(2) * Math.cos(Math.toRadians(45 - imageAngle)));
        double d2 = (actualDistance - d1);
        double mineralAngle = Math.toDegrees(Math.atan((innerHeight) / (d2)));

        MineralPositionViewModel positions = new MineralPositionViewModel();
        positions.right.angle = (float) mineralAngle;
        positions.center.angle = (float) mineralAngle + 45;
        positions.left.angle = (float) mineralAngle + 90;

        positions.right.distance = 450;
        positions.center.distance = 450;
        positions.left.distance = 450;

        return positions;
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
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() <= 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    opMode.telemetry.addData("Gold Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Gold:", Integer.toString(goldMineralX));
                                    scanResult = 1;
                                    drivetrain.StopAll();
                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    opMode.telemetry.addData("Silver Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Silver ", Integer.toString(goldMineralX));
                                    scanResult = 2;
                                    drivetrain.StopAll();
                                } else {
                                    opMode.telemetry.addData("no mineral found", 0);
                                    Log.i("[phoenix]:No Mineral:", Integer.toString(0));
                                    scanResult = 0;
                                }
                            }

                        }
                        opMode.telemetry.update();
                    }
                }
            }
        }

        return scanResult;
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
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 6.25f, Direction.RIGHT);// was 6.5
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 5.5f, Direction.LEFT); // was 4.5

                                        drivetrain.StopAll();
                                        sleep(200);
                                        // here drive 1.414*12 inch = 17.0
                                        // ACTIVE TESTING:  Original value 34f for distance
                                        drivetrain.Drive(0.3f, 25f, Direction.FORWARD);  //
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
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 6F, Direction.RIGHT); //// was 6.5
                                        sleep(100);
                                        drivetrain.StopAll();
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 5.5F, Direction.LEFT); //// was 4.5
                                        drivetrain.StopAll();
                                        sleep(300);
                                        drivetrain.Drive(0.3f, 14f, Direction.FORWARD);
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

    public void StrafeWhileVisible ( float power, float stop_distance, float maxGoldWidth, long report_time, LinearOpMode opMode){

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
        opMode.telemetry.addData("in strafe test ", "strafe test");

        Log.i("[phoenix]:str-tst dist", Float.toString(stop_distance));
        //     Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
        //     Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
        opMode.telemetry.update();

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

        while (opModeIsActive() && (cur_Gold_Width < maxGoldWidth) && (local_encoder_count<= stop_encoder_count)) { //600 is too much, 500 not engouth(local_encoder_count < stop_encoder_count)) {
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

                                    opMode.telemetry.addData("center of gold ", cur_center_Gold);
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

                                        StrafeWhileVisible(0.30f, 29f, 460f, 5, this);

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
                                        StrafeWhileVisible(0.30f, 5.5f, 460f, 5, this);
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

                                        StrafeWhileVisible(0.30f, 14f, 460f, 5, this);
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

    public Integer ScanFirstMineral(LinearOpMode opMode) {

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
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() <= 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    opMode.telemetry.addData("Gold Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Gold:", Integer.toString(goldMineralX));
                                    scanResult = 1;
                                } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    opMode.telemetry.addData("Silver Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Silver ", Integer.toString(goldMineralX));
                                    scanResult = 2;
                                } else {
                                    opMode.telemetry.addData("no mineral found", 0);
                                    Log.i("[phoenix]:No Mineral:", Integer.toString(0));
                                    scanResult = 0;
                                }
                            }

                        }
                        opMode.telemetry.update();
                    }
                }
            }
        }

        return scanResult;
    }

        // details is defined in subclass erikautobase, this one doesnt work out well since not enough stability while turning
    public void TurnToGold (float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode, long report_time) { // turn to A, B mineral

    }


        public boolean CanSeeGold() {

        boolean canseegold = false;

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
            }
        }

        return canseegold;
    }

}
