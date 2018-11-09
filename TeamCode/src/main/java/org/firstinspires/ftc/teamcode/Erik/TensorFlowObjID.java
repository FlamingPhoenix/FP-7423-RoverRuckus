/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Erik;

//package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Log;
import android.view.ViewDebug;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;


/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name = "TF Object Detection - Erik", group = "Concept")
//@Disabled
public class TensorFlowObjID extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain drivetrain;  // why not ErikDriveTrain vs DriveTrain type ?
    MyBoschIMU imu;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia; // this can be used for image as well as mineral ID

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */


    private TFObjectDetector tfod;

    public void initialize() {   // will include vuforia initialization here..remove initVuforia()
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        //*******************************************************/////
        ///Use Erik's Drive Train to experiment Erik's change
        drivetrain = new ErikDriveTrain(fl, fr, bl, br);

        // following code replaced by MyBoschIMU and its class

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled = true;
        //parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = new MyBoschIMU(hardwareMap);
        //imu.initialize(parameters);
        imu.initialize(new BNO055IMU.Parameters());

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(); //(cameraMonitorViewID);
        param.vuforiaLicenseKey = VUFORIA_KEY; //"AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = CameraDirection.BACK;
        //com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        //parameters.cameraDirection = CameraDirection.BACK;
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }



    @Override
    public void runOpMode() {
        initialize();

        //VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        //rover.activate();

        //VuforiaTrackable backTarget = rover.get(2);
        //backTarget.setName("front");

        // this line can keep, just driving forward after released..
        //drivetrain.Drive(0.2F, 17F, Direction.FORWARD);
        // this line should be changed to drive while looking for gold mineral, then call strafe towards right to knock if off.

        //drivetrain.Strafe(0.4F, 17F, Direction.LEFT);






        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //telemetry.addData("before init vuforia in run mode", 0);
        //Log.i("TF test", "before init vuforia in run mode " );
        //initVuforia();
        //telemetry.addData("after init vuforia in run mode", 0);
        //Log.i("TF test", "after init vuforia in run mode " );

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


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                fl.setPower(0.11f);
                fr.setPower(0.11f);
                bl.setPower(0.11f);
                br.setPower(0.11f);
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    // check methods for recognition
                                    estimatedAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);//(AngleUnit.DEGREES, )
                                    telemetry.addData("Gold esti-angle to obj", estimatedAngle);
                                    Log.i("Gold esti-angle to obj", Double.toString(estimatedAngle));
                                    //  can use this to estimate distance, distance (inch) = 2/(2*pie*estimatedAngle/360) = 360/(pie*estimatedAngle)

                                     // here, need to add code to come up with center of gold and use that as control varialble for moving robot towards it..
                                    center_Gold = (recognition.getLeft() + recognition.getRight())/2f;
                                    telemetry.addData("center of gold ", center_Gold);
                                    Log.i("center of gold", Float.toString(center_Gold));
                                    // here decide if gold is left of center or right of center, to tell robot move right/left.
                                    // if center_Gold < 360 or > 360...
                                    // for now, just knock it off..
                                    if (center_Gold < 300f) {
                                        drivetrain.StopAll();
                                        drivetrain.Strafe(0.25f,10, Direction.RIGHT);
                                        drivetrain.StopAll();}
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    // check methods for recognition
                                    estimatedAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);//(AngleUnit.DEGREES, )
                                    telemetry.addData("Silver-1 esti-angle-obj", estimatedAngle);
                                    Log.i("S-1 esti-angle to obj", Double.toString(estimatedAngle));
                                    //  can use this to estimate distance, distance (inch) = 2/(2*pie*estimatedAngle/360) = 360/(pie*estimatedAngle)
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                    // check methods for recognition
                                    estimatedAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);//(AngleUnit.DEGREES, )
                                    telemetry.addData("S2 esti-angle-obj", estimatedAngle);
                                    Log.i("S-2 esti-angle to obj", Double.toString(estimatedAngle));
                                    //  can use this to estimate distance, distance (inch) = 2/(2*pie*estimatedAngle/360) = 360/(pie*estimatedAngle)
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine. removed, included in initialize() code
     */

    /*private void initVuforia() {

         //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        telemetry.addData("beginning of intiVuforia", 0);
        Log.i("init vuforia", "beginning of intiVuforia " );
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        telemetry.addData("before Vuforia key", 0);
        Log.i("init vuforia", "before Vuforia key" );
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        telemetry.addData("after Vuforia key", 0);
        Log.i("init vuforia", "after Vuforia key" );
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        telemetry.addData("end of intiVuforia", 0);
        telemetry.update();
        Log.i("init vuforia", "end of init Vuforia" );

    }

    */

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
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