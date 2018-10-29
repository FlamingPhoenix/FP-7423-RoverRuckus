package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
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

@TeleOp(name = "GL-Transform-Erik", group = "none")

public class ErikGLTransform extends OpMode {
    VuforiaLocalizer vuforia;
    VuforiaTrackable redWall;
    VuforiaTrackable backWall;
    VuforiaTrackable blueWall;
    VuforiaTrackable frontWall;
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer.Parameters param;

    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters
        param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
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

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        String TAG = "Vu Nav Test";

        OpenGLMatrix redWallTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(- mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redWall.setLocation(redWallTargetLocationOnField);
        //telemetry.addData("target in Field", format(redWallTargetLocationOnField)); per test, it will report 1.8034 meters away from center, good
        RobotLog.ii(TAG, "Red Target in Field=%s", format(redWallTargetLocationOnField));
        Log.i("Red Target in Field = ", format(redWallTargetLocationOnField));
        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,150,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YXY,
                        AngleUnit.DEGREES, -90,  90, 0));
        //telemetry.addData("Phone on Robot", format(phoneLocationOnRobot)); fixed, will report location of phone relative to robot.
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
        Log.i("phone on robot =", format(phoneLocationOnRobot));

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener)redWall.getListener()).setPhoneInformation(phoneLocationOnRobot, param.cameraDirection);

        /**
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        telemetry.update();



    }

    @Override
    public void loop() {


        //OpenGLMatrix pose1 = ((VuforiaTrackableDefaultListener) redWall.getListener()).getPose();
        //OpenGLMatrix pose2 = ((VuforiaTrackableDefaultListener) redWall.getListener()).getRobotLocation();
        //OpenGLMatrix pose2 = ((VuforiaTrackableDefaultListener) redWall.getListener()).getUpdatedRobotLocation();

        //telemetry.addData(redWall.getName(), ((VuforiaTrackableDefaultListener)redWall.getListener()).isVisible() ? "Visible" : "Not Visible");    //
        Log.i("image is ",Boolean.toString(((VuforiaTrackableDefaultListener)redWall.getListener()).isVisible()));

        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)(VuforiaTrackableDefaultListener) redWall.getListener()).getRobotLocation(); // change to get robotlocation, .getUpdatedRobotLocation();

        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
            }

        /**
        * Provide feedback as to where the robot was last located (if we know).
        */
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Robot last Pos", format(lastLocation));
            Log.i("Robot last Pos =", format(lastLocation));
            Log.i("get-translation x= ", Float.toString(lastLocation.getTranslation().get(0)));
            Log.i("get-translation y= ", Float.toString(lastLocation.getTranslation().get(1)));
            Log.i("translation z = ", Float.toString(lastLocation.getTranslation().get(2)));
            Log.i("get column x= ", Float.toString(lastLocation.getColumn(3).get(0)));
            Log.i("get column y= ", Float.toString(lastLocation.getColumn(3).get(1)));
            Log.i("get column z = ", Float.toString(lastLocation.getColumn(3).get(2)));
        }
        else {
            telemetry.addData("Pos", "Unknown");
            Log.i("Robot last Pos =", "unknown"); }

        //String tag = "Msg from loop - ";


      /*  if (pose2 != null & pose1 !=null ) {
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
        telemetry.update(); */
        }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}