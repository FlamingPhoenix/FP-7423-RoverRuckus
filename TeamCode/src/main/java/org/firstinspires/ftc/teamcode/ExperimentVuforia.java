package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRelicRecovery;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import android.util.Log;

import java.util.List;

@Disabled
@TeleOp(name = "TestVuforia", group = "none")
public class ExperimentVuforia extends OpMode {

    VuforiaLocalizer vuforia;
    VuforiaTrackable redWall;
    VuforiaTrackable backWall;
    VuforiaTrackable blueWall;
    VuforiaTrackable frontWall;

    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();


        blueWall = rover.get(0);
        blueWall.setName("bluewall");
        redWall = rover.get(1);
        redWall.setName("bluewall");
        frontWall = rover.get(2);
        frontWall.setName("frontwall");
        backWall = rover.get(3);
        backWall.setName("backwall");


        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        if (tfod != null)
            tfod.activate();

    }

    @Override
    public void loop() {

        VuforiaTrackableDefaultListener redWallListener = (VuforiaTrackableDefaultListener) redWall.getListener();

        if (redWallListener.isVisible()) {

            OpenGLMatrix pose = redWallListener.getPose();
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
            OpenGLMatrix adjustedPose = pose.multiplied(rotationMatrix);
            Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("Raw :", "x=%f, y=%f, z=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
            telemetry.addData("Adj :", "x=%f, y=%f, z=%f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);


            //String s = String.format("x=%f, y=%f, z=%f;  adjX=%f, adjY=%f, adjZ=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle, adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation. thirdAngle);
            Log.i("[phoenix:testVuforia]",
                    String.format("x=%f, y=%f, z=%f;  adjX=%f, adjY=%f, adjZ=%f",
                                   orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle,
                                   adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation. thirdAngle));


            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    int silverCount = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() .equals( LABEL_GOLD_MINERAL)) {
                            telemetry.addData("Gold:", recognition.getLeft());
                            Log.i("[phoenix:testVuforia]", String.format("Gold: %f", recognition.getLeft()));
                        }
                        else if (recognition.getLabel() .equals( LABEL_SILVER_MINERAL)) {
                            silverCount += 1;
                            String label = String.format("Silver %d:", silverCount);
                            telemetry.addData(label, recognition.getLeft());
                            Log.i("[phoenix:testVuforia]", String.format("Silver %d: %f", silverCount, recognition.getLeft()));
                        }
                    }
                }
            }
            else
                telemetry.addData("# problem", "tfod is null");


        }

        telemetry.update();
    }

}
