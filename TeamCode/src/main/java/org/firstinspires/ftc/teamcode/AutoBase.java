package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

    public float PPR = 1120F;

    protected ElapsedTime runtime = new ElapsedTime();

    protected enum MineralPosition {LEFT, CENTER, RIGHT, UNKNOWN}

    protected DcMotor fl;
    protected DcMotor fr;
    protected DcMotor bl;
    protected DcMotor br;
    protected DriveTrain drivetrain;
    protected MyBoschIMU imu;
    DcMotor rightLift;
    DcMotor leftLift;
    Servo hook;
    boolean isHookOpen = false;
    DigitalChannel liftSensor;
    int magZero = 0;

    protected VuforiaTrackable backTarget;
    protected VuforiaTrackable frontTarget;
    protected VuforiaTrackable redTarget;
    protected VuforiaTrackable blueTarget;

    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    protected Servo markerHook;
    Servo arm;
    Servo hopper;

    public void initialize() {

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLift = hardwareMap.dcMotor.get("rightlift");
        leftLift = hardwareMap.dcMotor.get("leftlift");

        hook = hardwareMap.servo.get("hook");
        ServoControllerEx hookController = (ServoControllerEx) hook.getController();
        int hookServoPort = hook.getPortNumber();
        PwmControl.PwmRange hookPwmRange = new PwmControl.PwmRange(899, 2000);
        hookController.setServoPwmRange(hookServoPort, hookPwmRange);

        hopper = hardwareMap.servo.get("hopper");
        ServoControllerEx hopperController = (ServoControllerEx) hopper.getController();
        int hopperServoPort = hopper.getPortNumber();
        PwmControl.PwmRange hopperPwmRange = new PwmControl.PwmRange(899, 2105);
        hopperController.setServoPwmRange(hopperServoPort, hopperPwmRange);

        liftSensor = hardwareMap.get(DigitalChannel.class, "liftsensor");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drivetrain = new DriveTrain(fl, fr, bl, br, this);

        markerHook = hardwareMap.servo.get("markerhook");
        ServoControllerEx primaryController = (ServoControllerEx) markerHook.getController();
        int grabberServoPort = markerHook.getPortNumber();
        PwmControl.PwmRange grabberPwmRange = new PwmControl.PwmRange(899, 2150);
        primaryController.setServoPwmRange(grabberServoPort, grabberPwmRange);

        arm = hardwareMap.servo.get("arm");
        ServoControllerEx armController = (ServoControllerEx) arm.getController();
        int armServoPort = arm.getPortNumber();
        PwmControl.PwmRange armPwmRange = new PwmControl.PwmRange(1480, 1700);
        armController.setServoPwmRange(armServoPort, armPwmRange);

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
        int firstHitEncoderCount = Math.round(PPR * firstHitDistance / (4 * 3.1416f)); //89*3.5 = 312, 0.8 is a factor
        int secondHitEncoderCount = Math.round(PPR * secondHitDistance / (4 * 3.1416f)); // 89*15.5 = 1380
        int thirdEncoderCount = Math.round(PPR * thirdHitDistance / (4 * 3.1416f)); // 89*27.5 = 2448
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
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                                        drivetrain.Drive(0.2f, 3.0f, Direction.FORWARD);
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 6.5f, Direction.RIGHT);// was 6.5
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 5.75f, Direction.LEFT); // was 4.5

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
                                        drivetrain.Strafe(0.4f, 6.5F, Direction.RIGHT); //// was 6.5
                                        sleep(100);
                                        drivetrain.StopAll();
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 5.75F, Direction.LEFT); //// was 4.5
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




    // this version will add mineral filtering
    public void scanGold_Diagonal_Filter ( float power, float leftScreenPosition, float rightScreenPosition, float known_max_mineral_bottom, LinearOpMode opMode){
        long currentTime;
        int turnAngle;
        int gold_Found = 0;
        float distanceFromStart = 0f;
        int firstHitEncoderCount = Math.round(PPR* firstHitDistance / (4 * 3.1416f)); //89*3.5 = 312, 0.8 is a factor
        int secondHitEncoderCount = Math.round(PPR * secondHitDistance / (4 * 3.1416f)); // 89*15.5 = 1380
        int thirdEncoderCount = Math.round(PPR * thirdHitDistance / (4 * 3.1416f)); // 89*27.5 = 2448
        int currentPosition = 0;
        int gold_loop_No = 0;

        float currentAngle; // angle provided by TF method
        double estimatedAngle;
        float left_Gold;
        float right_Gold;
        float center_Gold;  // center coodinates of gold
        float current_Gold_Width; // width of gold, to drive towards it..
        float actualPower = power;
        float mineral_Bottom = 0;
        float min_Mineral_Bottom = 1100;
        float max_mineral_height = 0;
        int index_Gold = -1;
        int index_Min_Bottom_Mineral = -1;
        int is_Real_Gold = 0;


        runtime.reset(); // need to use time for tracking minerals instead of just  number of objects
        currentTime = Math.round(runtime.milliseconds());

        telemetry.addData("before moving time", currentTime);
        Log.i("[phoenix]:pretime", Double.toString(currentTime));

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                    if (updatedRecognitions.size() <= 10) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        min_Mineral_Bottom = 1100;
                        index_Min_Bottom_Mineral = -1;
                        index_Gold = -1;
                        mineral_Bottom = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            // here needs a loop to find out bottom value for each mineral, and the highest..
                            mineral_Bottom = recognition.getTop();
                            max_mineral_height = recognition.getBottom() - recognition.getTop();
                            telemetry.addData("cur max height", max_mineral_height);
                            Log.i("[phoenix]:max height", Float.toString(max_mineral_height));
                            telemetry.addData("cur btm", mineral_Bottom);
                            Log.i("[phoenix]:cur btm", Float.toString(mineral_Bottom));
                            telemetry.addData("height by call", recognition.getHeight());
                            Log.i("[phoenix]:height byCall", Float.toString(recognition.getHeight()));
                            //telemetry.addData("height by image call", recognition.getImageHeight());
                            //Log.i("[phoenix]:heightByImage", Float.toString(recognition.getImageHeight()));
                            telemetry.update();

                            if (mineral_Bottom < min_Mineral_Bottom) {
                                min_Mineral_Bottom = mineral_Bottom;
                                index_Min_Bottom_Mineral = updatedRecognitions.indexOf(recognition);
                                telemetry.addData("cur max bottom", min_Mineral_Bottom);
                                Log.i("[phoenix]:cur max Bttm", Float.toString(min_Mineral_Bottom));
                                telemetry.addData("max bottom index", index_Min_Bottom_Mineral);
                                Log.i("[phoenix]:max btm ind", Integer.toString(index_Min_Bottom_Mineral));
                                telemetry.addData("cur top", recognition.getBottom());
                                Log.i("[phoenix]:cur top", Float.toString(recognition.getBottom()));
                                telemetry.update();

                            }

                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                index_Gold = updatedRecognitions.indexOf(recognition);
                                telemetry.addData("gold index", index_Gold);
                                Log.i("[phoenix]:gold index", Integer.toString(index_Gold));
                            }
                            telemetry.update();
                        }

                        if (index_Gold == index_Min_Bottom_Mineral && min_Mineral_Bottom < known_max_mineral_bottom)
                         {    // max_Mineral_Bottom > (known_max_mineral_bottom - 0.5*max_mineral_height)
                                gold_loop_No = gold_loop_No + 1;
                                currentTime = Math.round(runtime.milliseconds());
                                opMode.telemetry.addData("1st Gold time ", currentTime);
                                Log.i("[phoenix]:goldtime", Double.toString(currentTime));

                                //if (currentTime < (1.5*secondHitTime)) {
                                //goldMineralX = (int) recognition.getLeft();
                                left_Gold = updatedRecognitions.get(index_Gold).getLeft();//recognition.getLeft();
                                right_Gold = updatedRecognitions.get(index_Gold).getRight();//recognition.getRight();
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
                                        drivetrain.Drive(0.2f, 2.0f, Direction.FORWARD); // was 3
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 7.5f, Direction.RIGHT);// was 6.5
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 5.75f, Direction.LEFT); // was 4.5

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
                                        drivetrain.Drive(0.2f, 2.0f, Direction.FORWARD);
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 7.5F, Direction.RIGHT); //// was 6.5
                                        sleep(100);
                                        drivetrain.StopAll();
                                        sleep(300);
                                        drivetrain.Strafe(0.4f, 5.75F, Direction.LEFT); //// was 4.5
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

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        stop_encoder_count = Math.round(PPR * (stop_distance / (4f * 3.1416f)));
        base_Time = Math.round(runtime.milliseconds());
        base_encoder_count = Math.abs(fr.getCurrentPosition());
        Log.i("[phoenix]:bfr setpower", Float.toString(power));
        Log.i("[phoenix]:encoder", Float.toString(stop_encoder_count));
        Log.i("[phoenix]:base time", Long.toString(base_Time));
        // stafe to right..

        while (opModeIsActive() && (cur_Gold_Width < maxGoldWidth) && (local_encoder_count<= stop_encoder_count)) { //600 is too much, 500 not engouth(local_encoder_count < stop_encoder_count)) {
            // report every 50 m

            //base_Time = Math.round(runtime.milliseconds());
            local_encoder_count = Math.abs(fl.getCurrentPosition()) - base_encoder_count;

            Log.i("[phoenix]:opAct l-enco", Integer.toString(local_encoder_count));

            fl.setPower(power*1.2F + lateral_power*1.2F - 0.1f*power);
            fr.setPower(-power*1.2F + lateral_power*1.2F - 0.1f*power); // adjust for weight, as long as wont exceed 1.0, otherwise, need to normalize power
            bl.setPower(-power + lateral_power - 0.1f*power);
            br.setPower(power + lateral_power - 0.1f*power);
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
                                    lateral_power = 0.10f;
                                } else if (cur_center_Gold > 350) {
                                    lateral_power = -0.12f;
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
        int firstHitEncoderCount = Math.round(PPR * firstHitDistance / (4 * 3.1416f)); //89*3.5 = 312, 0.8 is a factor
        int secondHitEncoderCount = Math.round(PPR * secondHitDistance / (4 * 3.1416f)); // 89*15.5 = 1380
        int thirdEncoderCount = Math.round(PPR * thirdHitDistance / (4 * 3.1416f)); // 89*27.5 = 2448
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
                                //updatedRecognitions.indexOf(recognition)
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
                                        //drivetrain.Strafe(0.4f, 36, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(500);
                                        drivetrain.Strafe(0.4f, 36, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(500);
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
//                                       drivetrain.Strafe(0.4f, 5.5F, Direction.RIGHT);
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
                                        telemetry.addData("gold frequency", gold_loop_No);
                                        Log.i("[phoenix]:gold freq", Integer.toString(gold_loop_No));
                                        telemetry.update();
                                    } else if (gold_Found == 0) {

                                        StrafeWhileVisible(0.30f, 14f, 460f, 5, this);
                                      //  drivetrain.Strafe(0.4f, 14F, Direction.RIGHT);
                                        drivetrain.StopAll();
                                        sleep(1500);
                                        drivetrain.Strafe(0.4f, 14F, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(1500);
                                        //tfod.deactivate();
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

    public Float FindClosestMineral_Y(float rangeFactor, LinearOpMode opMode) {

        float mineral_Bottom = 1100;
        float min_Mineral_Bottom = 1100;
        //int index_Gold = -1;
        float mineral_Height = 0;
        int index_Min_Bottom_Mineral = -1;

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && (min_Mineral_Bottom == 1100)) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() <= 10) {

                            for (Recognition recognition : updatedRecognitions) {
                                // here needs a loop to find out bottom value for each mineral, and the highest..
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) || recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    mineral_Bottom = recognition.getTop(); // should be top because cell fliped
                                    if (mineral_Bottom < min_Mineral_Bottom) {
                                        min_Mineral_Bottom = mineral_Bottom;
                                        index_Min_Bottom_Mineral = updatedRecognitions.indexOf(recognition);
                                        telemetry.addData("cur max bottom", min_Mineral_Bottom);
                                        Log.i("[phoenix]:cur max Bttm", Float.toString(min_Mineral_Bottom));
                                        telemetry.addData("max bottom index", index_Min_Bottom_Mineral);
                                        Log.i("[phoenix]:max btm ind", Integer.toString(index_Min_Bottom_Mineral));
                                        opMode.telemetry.update();
                                    }
                                }
                                else
                                    {telemetry.addData("didnt find any gold or sliver", "no mineral found");
                                    Log.i("[phoenix]:None found", "no mineral found");
                                    opMode.telemetry.update(); }
                            }

                            if (index_Min_Bottom_Mineral != -1 ) {
                                mineral_Height = updatedRecognitions.get(index_Min_Bottom_Mineral).getHeight();
                                telemetry.addData("mineral height: ", mineral_Height);
                                Log.i("[phoenix]:mineral hight", Float.toString(mineral_Height));
                                opMode.telemetry.update();
                            }
                        }
                    }
                }
            }
        }
        return min_Mineral_Bottom + rangeFactor*mineral_Height;
    }

    public Integer ScanFirstMineralSimple() {
        // this version of scan first mineral, will return Gold, OR if not GOLD, will not confirm but just assume Silver
        int scanResult = 0;
        int i = 0;
        int numberOfScanObjects = 0; // this is to deal with no able to scan anything..

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            // scan five times


            while (opModeIsActive() && scanResult == 0 && (i <= 5) && numberOfScanObjects == 0) { // change from if since it is missing scaning gold, (Dec 30)change to if b/c it scans nothing (it is tradeoff)

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        numberOfScanObjects = updatedRecognitions.size();
                        telemetry.addData("# Object Detected", numberOfScanObjects);
                        i = i + 1; // will just scan five times.
                        if (numberOfScanObjects <= 5) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("Gold Mineral Position", goldMineralX);
                                    Log.i("[phoenix]:Gold:", Integer.toString(goldMineralX));
                                    scanResult = 1;
                                } else {
                                    silverMineral1X = (int) recognition.getLeft();
                                    telemetry.addData("assume Silver Mineral Position", silverMineral1X);
                                    Log.i("[phoenix]:assume Sil", Integer.toString(silverMineral1X));
                                    scanResult = 2;

                                }
                            }

                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (scanResult == 0)
            return 2;
        else
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

    public boolean isGoldVisible()
    {
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions.size() > 0)
            {
                if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL))
                    return true;
            }
        }

        return false;
    }

    public void releaseFromLander()
    {
        while (liftSensor.getState() == true && this.opModeIsActive())
        {
            if (rightLift.getCurrentPosition() > -3000)
            {
                rightLift.setPower(-1.0);
                leftLift.setPower(-1.0);
            }
            else
            {
                rightLift.setPower(-0.3);
                leftLift.setPower(-0.3);
            }
        }
        rightLift.setPower(0);
        leftLift.setPower(0);

        hook.setPosition(0.2);
        sleep(700);
    }

    public void sampleGold(LinearOpMode opMode) {
        float reference_Bottom_Y = 0;
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
            drivetrain.Strafe(0.3f, 10f, Direction.RIGHT);
            sleep(300);
            StrafeWhileVisible(0.3f, 14.f, 720, 10, this);
            telemetry.addData("Gold aft straf", "after strafe");
            Log.i("[phoenix]:gold aft str", "after strafe");
            sleep(300);
            drivetrain.Strafe(0.3f, 6.5f, Direction.LEFT);
            drivetrain.Turn(0.4f, 35, Direction.COUNTERCLOCKWISE, imu, this); // shouid be 45, compensate for wheel issue
            drivetrain.Drive(0.3f, 34f, Direction.FORWARD);}
        else { //ScanFirstMineral() == 2, in this scenario, either B or C is GOLD
            telemetry.addData("Silver found", "during first scan");
            Log.i("[phoenix]:Silv detected", "found silver");
            // strafe to the right position
            drivetrain.Strafe(0.3f, 7.5f, Direction.RIGHT);  // was 2 or 5.5 before..need to evaluate the risk of hitting lander leg
            sleep(500);
            drivetrain.Turn(0.2f, 35, Direction.COUNTERCLOCKWISE, imu, this); // should be 45, compensate for wheels issue
            telemetry.addData("Silver aft turn", "after turn");
            Log.i("[phoenix]:Silv aft turn", "aft turn");
            sleep(500);
            // here will do a still scan, return mineral bottom, as reference for filtering.
            reference_Bottom_Y = FindClosestMineral_Y(0.7f,this);
            telemetry.addData("ref bottomY", reference_Bottom_Y);
            Log.i("[phoenix]:refBottomY", Float.toString(reference_Bottom_Y));
            sleep(500);
            drivetrain.Drive(0.3f, 3.5f, Direction.BACKWARD);
            sleep(300);
            // scan the next two minerals for GOLD
            //scanGold_Diagonal(0.11f, 200, 420, this); // was 240 and 380
            scanGold_Diagonal_Filter(0.13f, 200, 420, reference_Bottom_Y, this);
        }
    }
}
