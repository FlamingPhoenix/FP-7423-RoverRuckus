package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Test Angle", group = "none")
public class TestAngle extends OpMode {

    VuforiaLocalizer vuforia;
    VuforiaTrackable redWall;
    VuforiaTrackable backWall;
    VuforiaTrackable blueWall;
    VuforiaTrackable frontWall;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

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
    }

    @Override
    public void loop() {

        VuforiaTrackableDefaultListener frontWallListener = (VuforiaTrackableDefaultListener) frontWall.getListener();

        if (frontWallListener.isVisible()) {

            OpenGLMatrix pose = frontWallListener.getPose();
            double distance = (pose.getTranslation().get(2)) * 0.0393701;

            Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
            OpenGLMatrix adjustedPose = pose.multiplied(rotationMatrix);
            Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("Raw :", "x=%f, y=%f, z=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
            telemetry.addData("Adj :", "x=%f, y=%f, z=%f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

            float imageAngle = Math.abs(adjustedOrientation.secondAngle);
            double actualDistance = ((distance) / (Math.cos(Math.toRadians(imageAngle))));
            double innerHeight = (24 * Math.sqrt(2) * Math.sin(Math.toRadians(45 - imageAngle)));
            double d1 = (24 * Math.sqrt(2) * Math.cos(Math.toRadians(45 - imageAngle)));
            double d2 = (actualDistance - d1);
            double mineralAngle = Math.toDegrees(Math.atan((innerHeight) / (d2)));

            telemetry.addData("distance: ", distance);
            telemetry.addData("actual distance: ", actualDistance);
            telemetry.addData("inner height: ", innerHeight);
            telemetry.addData("d1: ", d1);
            telemetry.addData("d2: ", d2);
            telemetry.addData("imageAngle: ", imageAngle);
            telemetry.addData("mineralAngle:", mineralAngle);

            //String s = String.format("x=%f, y=%f, z=%f;  adjX=%f, adjY=%f, adjZ=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle, adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation. thirdAngle);
            Log.i("[phoenix:testVuforia]",
                    String.format("x=%f, y=%f, z=%f;  adjX=%f, adjY=%f, adjZ=%f",
                                   orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle,
                                   adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation. thirdAngle));
            }

        telemetry.update();
    }

}
