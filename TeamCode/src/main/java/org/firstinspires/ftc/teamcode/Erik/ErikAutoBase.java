package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.MineralPositionViewModel;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Steve on 12/7/2018.
 */

public abstract class ErikAutoBase extends LinearOpMode {
    protected static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    protected static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    protected static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    protected static final float firstStopAngle = 50f;  // 20 by rough measure
    protected static final float secondStopAngle = 90f; // 90 by rough measure.

    protected ElapsedTime myruntime = new ElapsedTime();

    protected static final String VUFORIA_KEY = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";

    protected enum MineralPosition {LEFT, CENTER, RIGHT, UNKNOWN}

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    //protected DriveTrain drivetrain;
    protected ErikDriveTrain erikdrivetrain;

    protected MyBoschIMU imu;

    protected VuforiaTrackable backTarget;
    protected VuforiaTrackable frontTarget;
    protected VuforiaTrackable redTarget;
    protected VuforiaTrackable blueTarget;

    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    public void initialize() {

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        erikdrivetrain = new ErikDriveTrain(fl, fr, bl, br, this);
        // boolean drivetrain.robotWork = true;

        imu = new MyBoschIMU(hardwareMap);

        imu.initialize(new BNO055IMU.Parameters());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        param.useExtendedTracking = false; //new
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();

        // below is added for GL matrix tracking
        blueTarget = rover.get(0);
        blueTarget.setName("bluetarget");
        redTarget = rover.get(1);
        redTarget.setName("redTarget");
        frontTarget = rover.get(2);
        frontTarget.setName("frontTarget");
        backTarget = rover.get(3);
        backTarget.setName("backTarget");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

        allTrackables.addAll(rover);

        final float mmPerInch = 25.4f;
        final float mmBotWidth = 18 * mmPerInch;            // may need to change for new robot, 457.2
        // can calibrate follow measures below so that the measure is more accurate..
        final float mmCamera_Forward_Displacement = 0.5f * mmBotWidth;//6.0f * mmPerInch;   // 229Camera is 110 mm in front of robot center
        final float mmCamera_Height_Off_Ground = 4.25f * mmPerInch;   // Camera is 4.25 inch above ground, 108
        final float mmCamera_Left_Displacement = 7.5f * mmPerInch; //was 6.0, change to 5.0, -0.5f * mmBotWidth;// 152, Camera is ON the robots right handside, put right side of  robot as front side

        // create an image translation/rotation matrix to be used for specified image
        // put image centers 6" (6*2.54 = 150) above the 0:0:0 origin, rotate it so it is along the -X axis.

        telemetry.addData("before image translation", "fisrt call to opGL");
        Log.i("[pheonix]:begin of call", "before openGL");

        OpenGLMatrix imageOrientation = OpenGLMatrix

                .translation(0, 0, 110) // 150

                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XYZ,

                        AngleUnit.DEGREES, 90, 0, -90));// third angle: -90));


        /**

         * Create a transformation matrix describing where the phone is on the robot.
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  Camera and screen will be
         * in "portrait Mode" with screen closest to right handside of robot, camera at bottom(upside down) */
        telemetry.addData("before phone translation & rotation", "second call to opGL");

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix

                .translation(mmCamera_Forward_Displacement, mmCamera_Left_Displacement, mmCamera_Height_Off_Ground) //mmCamera_Forward_Displacement, -mmCamera_Right_Displacement, mmCamera_Height_Off_Ground

                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES,
                        -90, 0, 0));//third angel-90));  //
// x = -90, y, z = 0, forward = 6 inch, rigth = 9 inches.., this is put real front as front.


        // Set image target to have the same location and camera orientation

        for (VuforiaTrackable trackable : allTrackables)

        {

            trackable.setLocation(imageOrientation);
            telemetry.addData("after image set location", "image set location");
            Log.i("[pheonix]:after image", "set location");
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, param.cameraDirection);
            telemetry.addData("after phone set location", "phone set location");

        }

        // above is added for GL matrix tracking

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

    public void TurnToGold (float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode, long report_time){ // turn to A, B mineral

        long currentTime;
        int turnAngle;
        int gold_Found = 0;
        float distanceFromStart = 0f;
        //int firstHitEncoderCount = Math.round(1120f * firstHitDistance / (4 * 3.1416f)); //89*3.5 = 312, 0.8 is a factor
        //int secondHitEncoderCount = Math.round(1120f * secondHitDistance / (4 * 3.1416f)); // 89*15.5 = 1380
        //int thirdEncoderCount = Math.round(1120f * thirdHitDistance / (4 * 3.1416f)); // 89*27.5 = 2448
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

        myruntime.reset(); // need to use time for tracking minerals instead of just  number of objects
        currentTime = Math.round(myruntime.milliseconds());

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

                currentTime = Math.round(myruntime.milliseconds());

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
                                    currentTime = Math.round(myruntime.milliseconds());
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
                                        currentTime = Math.round(myruntime.milliseconds());
                                        Log.i("[phoenix]:goldtime ", Double.toString(currentTime));
                                        erikdrivetrain.StopAll();
                                        sleep(5000);
                                        currentPosition = Math.abs(fr.getCurrentPosition());
                                        telemetry.addData("aft encoder ", currentPosition);
                                        Log.i("[phoenix]:aft encoder ", Integer.toString(currentPosition));
                                        telemetry.update();
                                        if (currentAngle < (startAngle + firstStopAngle + 20) && (gold_Found == 0)) {
                                            // should try pro-strafe instead of regular strafe, to test navigation control

                                            erikdrivetrain.Strafe(0.30f, 19f,  Direction.RIGHT);

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
                                            erikdrivetrain.Strafe(0.30f, 5.5f, Direction.RIGHT);
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

                                            erikdrivetrain.Strafe(0.30f, 19, Direction.RIGHT);
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
                currentTime = Math.round(myruntime.milliseconds());
                currentPosition = fr.getCurrentPosition();
                telemetry.addData("no gold found, current time ", currentTime);
                telemetry.addData("no gold found, current counter ", currentPosition);
                telemetry.addData("gold found flag ", gold_Found);

                Log.i("[phoenix]:NoGold, time ", Double.toString(currentTime));
                Log.i("[phoenix]:NoGold, eCnt ", Integer.toString(currentPosition));
                Log.i("[phoenix]:Gold Fflag ", Integer.toString(gold_Found));
                telemetry.update();
                erikdrivetrain.StopAll();
            }

        } else {
        } // direction clockwise..
    }

}
