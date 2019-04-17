package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

import java.util.List;

@Disabled
@Autonomous(name="Oxford Blue Depot", group="none") //used to be called Red Gold, messed up Gold/Silver
//@TeleOp(name="Erik Auto Subclass", group="none")

// this could be used as a backup copy, as red crater has been heavily modified since Oxford

public class ErikAutoBlueDepot extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final long  firstHitTime = 1250; // this is from calibration, it is time to detect first object
    private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    private static final long thirdHitTime = 12000; //

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain drivetrain;  // why not ErikDriveTrain vs DriveTrain type ?
    MyBoschIMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    private static final String VUFORIA_KEY = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";

    private TFObjectDetector tfod;

    VuforiaLocalizer vuforia;

    public void initialize() {

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //*******************************************************/////
        ///Use Erik's Drive Train to experiment Erik's change
        drivetrain = new ErikDriveTrain(fl, fr, bl, br, this);

        imu = new MyBoschIMU(hardwareMap);

        imu.initialize(new BNO055IMU.Parameters());

        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(); //(cameraMonitorViewID);
        param.vuforiaLicenseKey = VUFORIA_KEY; //"AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        runtime.reset();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        long currentTime;
        int turnAngle;

        waitForStart(); // what is this for ?

        int gold_Found = 0;
        Double estimatedAngle;
        Float center_Gold;  // center coodinates of gold
        Float current_Gold_Width; // width of gold, to drive towards it..


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


//        if (opModeIsActive()) {
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        runtime.reset(); // need to use time for tracking minerals instead of just  number of objects

        while (gold_Found == 0 && opModeIsActive()) {

            fl.setPower(0.12f);  // first calibrate time on floor..0.12 at dr warners, 0.14 at carpet
            fr.setPower(0.12f);
            bl.setPower(0.12f);
            br.setPower(0.12f);

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
                                        sleep(500);
                                        drivetrain.Strafe(0.4f, 16F, Direction.LEFT);
                                        drivetrain.StopAll();
                                        sleep(500);                                            //tfod.deactivate();
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

        telemetry.addData("before setting tfod to null", "..");
        Log.i("scan image", "before setting tfod to null");

        //if (tfod != null) {// keep it open, otherwise, will have issue with vuforia scanning images later on
        //    tfod.deactivate();
        //    tfod.shutdown();
        //}

        //VuforiaLocalizer = null;

        telemetry.addData("before setting vuforia to null", "..");
        Log.i("scan image", "before setting vuforia to null");

        //vuforia = null;
        telemetry.addData("after setting vuforia to null", "..");
        Log.i("scan image", "after setting vuforia to null");

        telemetry.addData("before cam ViewId", "..");
        Log.i("scan image", "bbefore cam ViewId");
        //here need to re initialize vuforia and then get it ready for scanning image.

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("before Para new", "..");
        Log.i("scan image", "before Para new");
        //VuforiaLocalizer.Parameters param2 = new VuforiaLocalizer.Parameters(); //(cameraMonitorViewID);

        //VuforiaLocalizer.Parameters param2 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        telemetry.addData("before key", "..");
        Log.i("scan image", "bbefore key");
        //param2.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        telemetry.addData("before cam direction", "..");
        Log.i("scan image", "bbefore cam direction");
        //param2.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        telemetry.addData("before set hint", "..");
        Log.i("scan image", "bbefore set hint");
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        telemetry.addData("before cam class factory", "..");
        Log.i("scan image", "bbefore class factory");
        //vuforia = ClassFactory.getInstance().createVuforia(param2);

        telemetry.addData("before rover", "..");
        Log.i("scan image", "before rover");
        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        telemetry.addData("before rover activate", "..");
        Log.i("scan image", "bbefore rover activate");
        rover.activate();
        telemetry.addData("before rover get image", "..");
        Log.i("scan image", "bbefore rover get image");

        VuforiaTrackable backTarget = rover.get(2);  // front was 2(red planet), image should be red alliance
        backTarget.setName("red"); // was front, should be back


        telemetry.update();
        //drivetrain.Drive(0.2F, 17F, Direction.FORWARD);
        //drivetrain.Strafe(0.4F, 17F, Direction.LEFT);

        /*
        drivetrain.Turn(0.15F,90,Direction.CLOCKWISE, imu, this);
        drivetrain.Drive(0.25F, 5, Direction.FORWARD);
        drivetrain.Turn(0.15F,90,Direction.COUNTERCLOCKWISE, imu, this);
        */

        // above is code without attempting to knock off Gold
        // here we need to decide which position is Gold and then drive respectively
        currentTime = Math.round(runtime.milliseconds());
        switch (gold_Found) {
            case 0: // didnt detect gold
                telemetry.addData("gold found flag not set", gold_Found);
                Log.i("gold found flag not set", Integer.toString(gold_Found));
                telemetry.addData("current time is ", currentTime);
                Log.i("current time is ", Long.toString(currentTime));
                break;
            case 1:  // position A
                drivetrain.Drive(0.3f, 32F, Direction.FORWARD);
                telemetry.addData("position A ", "gold found is 1");
                Log.i("gold is A ", Integer.toString(gold_Found));
                telemetry.addData("current time is ", currentTime);
                Log.i("current time is ", Long.toString(currentTime));
                break;
            case 2:  // position B
                drivetrain.Drive(0.3f, 21F, Direction.FORWARD);
                telemetry.addData("position B ", "gold found is 2");
                Log.i("gold is B ", Integer.toString(gold_Found));
                telemetry.addData("current time is ", currentTime);
                Log.i("current time is ", Long.toString(currentTime));
                break;
            case 3:  // position C
                drivetrain.Drive(0.3f, 3.5F, Direction.FORWARD);
                telemetry.addData("position C ", "gold found is 3");
                Log.i("gold is C ", Integer.toString(gold_Found));
                telemetry.addData("current time is ", currentTime);
                Log.i("current time is ", Long.toString(currentTime));
                break;
            default:  // didnt detect Gold
                telemetry.addData("gold found flag not set", gold_Found);
                Log.i("gold found flag not set", Integer.toString(gold_Found));
        }

        telemetry.update();

        drivetrain.StopAll();

        // get to center
        drivetrain.Strafe(0.3f, 11.5F, Direction.LEFT);

        drivetrain.StopAll();

        drivetrain.TurnToImage(0.25F, Direction.CLOCKWISE, backTarget, imu, this); // at vinay, 0.4, at erik, can 0.4 or 0.5

        drivetrain.StopAll();

        drivetrain.StrafeToImage(0.3F, backTarget, this, 10);  //

        drivetrain.StopAll();
        /* this is reserved for waiting for alliance partner to set marker..
        telemetry.addData("will wait, 5 sec sleep ", "for alliance partner to set marker");
        Log.i("XZ after 5 sec wait", "for alliance partner to set marker");

        try {
            telemetry.update();
            Thread.sleep(5000);

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        */
        if (tfod != null) { // now it is ok to shutdown tfod/vuforia
            tfod.deactivate();
            tfod.shutdown();
        }

        // drive backward for certain distance.
        drivetrain.Drive(.4f, 85f, Direction.BACKWARD);
        drivetrain.Drive(1, 10, Direction.FORWARD);

        // can try drive straight..a method defined in subclass and just a skeleton at super class(DriveTrain)
        // drivetrain.Drive(0.3f, 89f, Direction.BACKWARD);

    }


    private void initTfod() {
        telemetry.addData("beginning of init TFod", 0);
        Log.i("init tf", "beginning of init TFod" );
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("init TFod, after hardwareMap", 0);
        Log.i("init tf", "init TFod, after hardwareMap" );
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        telemetry.addData("init TFod, after tfodParameters", 0);
        Log.i("init tf", "init TFod, after tfodParameters" );
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        telemetry.addData("init TFod, after tfod object creation", 0);
        Log.i("init tf", "init TFod, after tfod object creation" );
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        telemetry.addData("init TFod, after load model from asset", 0);
        Log.i("init tf", "init TFod, after load model from asset" );
        telemetry.addData("end of init TFOD", 0);
        telemetry.update();
        Log.i("init tf", "end of init TFod" );

    }

}
