package org.firstinspires.ftc.teamcode.Erik;

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

@TeleOp(name = "TestVuforiaErik", group = "none")

public class ExperimentVuforiaErik extends OpMode {
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
        OpenGLMatrix pose1 = ((VuforiaTrackableDefaultListener) redWall.getListener()).getPose();
        OpenGLMatrix pose2 = ((VuforiaTrackableDefaultListener) redWall.getListener()).getRobotLocation();
        //OpenGLMatrix pose2 = ((VuforiaTrackableDefaultListener) redWall.getListener()).getUpdatedRobotLocation();


        String tag = "Msg from loop - ";

 
        if (pose2 != null & pose1 !=null ) {
            // code below is to do a comparison between pose1 transformation and getRobotLocation, after comapring
            // getRobotLocation and getUpdatedRobotLocation, they seem to be quite similar

            Orientation orientation1 = Orientation.getOrientation(pose1, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation1.thirdAngle * -1, orientation1.firstAngle * -1, 0);
            OpenGLMatrix adjustedPose = pose1.multiplied(rotationMatrix);
            Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            Orientation orientation2 = Orientation.getOrientation(pose2, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            //telemetry.addData("x Angle(by GetUpdatedRobotLocation):", orientation.firstAngle);

            telemetry.addData("y Angle(by getPose plus MatrixTransform):", adjustedOrientation.secondAngle);
            telemetry.addData("x Cood(by GetPose and MatrixTransform, adj):", adjustedPose.getColumn(3).get(0)); //pose1.getColumn(3).get(0)
            telemetry.addData("y Cood(by GetPose and MatrixTransform, adj):", adjustedPose.getColumn(3).get(1)); //pose1.getColumn(3).get(1)
            telemetry.addData("z Cood(by GetPose and MatrixTransform, adj):", adjustedPose.getColumn(3).get(2)); //pose1.getColumn(3).get(2)


            telemetry.addData("y Angle(by GetRobotLocation):", orientation2.secondAngle);
            //telemetry.addData("z Angle(by GetUpdatedRobotLocation):", orientation.thirdAngle);
            telemetry.addData("x Cood(by GetRobotLocation):", pose2.getColumn(3).get(0));
            telemetry.addData("y Cood(by GetRobotLocation):", pose2.getColumn(3).get(1));
            telemetry.addData("z Cood(by GetRobotLocation):", pose2.getColumn(3).get(2));

            Log.i(tag, "x Angle(by GetPose and MatrixTransform): " + Float.toString(adjustedOrientation.firstAngle));
            Log.i(tag, "y Angle(by GetPose and MatrixTransform): " + Float.toString(adjustedOrientation.secondAngle));
            Log.i(tag, "z Angle(by GetPose and MatrixTransform): " + Float.toString(adjustedOrientation.thirdAngle));
            Log.i(tag, "x Cood(by GetPose and MatrixTransform, adj):  " + Float.toString(adjustedPose.getColumn(3).get(0)));
            Log.i(tag, "y Cood(by GetPose and MatrixTransform, adj): " + Float.toString(adjustedPose.getColumn(3).get(1)));
            Log.i(tag, "z Cood(by GetPose and MatrixTransform, adj): " + Float.toString(adjustedPose.getColumn(3).get(2)));

            Log.i(tag, "x Angle(by getRobotLocation): " + Float.toString(orientation2.firstAngle));
            Log.i(tag, "y Angle(by getRobotLocation): " + Float.toString(orientation2.secondAngle));
            Log.i(tag, "z Angle(by getRobotLocation): " + Float.toString(orientation2.thirdAngle));
            Log.i(tag, "x Cood(by getRobotLocation):  " + Float.toString(pose2.getColumn(3).get(0)));
            Log.i(tag, "y Cood(by getRobotLocation): " + Float.toString(pose2.getColumn(3).get(1)));
            Log.i(tag, "z Cood(by getRobotLocation): " + Float.toString(pose2.getColumn(3).get(2)));

        }
        telemetry.update();
    }
}
