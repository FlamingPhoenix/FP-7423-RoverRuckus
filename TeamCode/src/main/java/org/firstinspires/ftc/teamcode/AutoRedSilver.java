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

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="Auto Red Silver", group="none")
public class AutoRedSilver extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();

        waitForStart();

        MineralPosition goldPosition = MineralPosition.UNKNOWN;;

        if (tfod != null) {
            tfod.activate();

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions.size() == 1)


            {
                if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL))
                {
                    drivetrain.Turn(0.25F, 10, Direction.CLOCKWISE, imu, this);
                    updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions.size() == 1)
                    {
                        if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL))
                            goldPosition = MineralPosition.RIGHT;
                        else
                            goldPosition = MineralPosition.CENTER;
                    }
                    else {
                        MineralPosition p2 = getGoldPosition(updatedRecognitions);

                        if (p2 == MineralPosition.LEFT)
                            goldPosition = MineralPosition.CENTER;
                        else if (p2 == MineralPosition.RIGHT)
                            goldPosition = MineralPosition.RIGHT;
                        else
                            goldPosition = MineralPosition.UNKNOWN;
                    }
                }
                else if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldPosition = MineralPosition.LEFT;
                }
            }
            else if (updatedRecognitions.size() == 2)
            {
                MineralPosition p1 = getGoldPosition(updatedRecognitions);
                if (p1 == MineralPosition.LEFT)
                    goldPosition = MineralPosition.LEFT;
                else if (p1 == MineralPosition.RIGHT)
                    goldPosition = MineralPosition.CENTER;
                else
                    goldPosition = MineralPosition.RIGHT;
            }

        }

        drivetrain.Turn(0.25F, 25, Direction.CLOCKWISE, imu, this);

        VuforiaTrackableDefaultListener listener = ((VuforiaTrackableDefaultListener) frontTarget.getListener());

        if (listener.isVisible()) {

            OpenGLMatrix pose = listener.getPose();
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
            double mineralAngle = Math.abs(Math.toDegrees(Math.atan((innerHeight) / (d2))));

            telemetry.addData("Gold Position :", "%f", goldPosition);

            if (goldPosition == MineralPosition.RIGHT)
                drivetrain.Turn(0.25F, (int) mineralAngle, Direction.CLOCKWISE, imu, this);
            else if (goldPosition == MineralPosition.CENTER) {
                int centerAngle = ((int) mineralAngle) - 45;
                if (centerAngle <= 0)
                    drivetrain.Turn(0.25F, Math.abs(centerAngle), Direction.COUNTERCLOCKWISE, imu, this);
                else
                    drivetrain.Turn(0.25F, Math.abs(centerAngle), Direction.CLOCKWISE, imu, this);
            }
            else {
                int leftAngle = ((int) mineralAngle) - 90;
                if (leftAngle <= 0)
                    drivetrain.Turn(0.25F, Math.abs(leftAngle), Direction.COUNTERCLOCKWISE, imu, this);
                else
                    drivetrain.Turn(0.25F, Math.abs(leftAngle), Direction.CLOCKWISE, imu, this);
            }

            drivetrain.Drive(0.3f, 12, Direction.FORWARD );

         }

        drivetrain.Turn(.3F, 10, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(20000);
       /* drivetrain.Strafe(.4F, 6, Direction.LEFT);

        while (pose == null)
        {
            drivetrain.Turn(.4F, 10, Direction.CLOCKWISE,imu,this);
        }

        drivetrain.StrafeToImage(.4F, backTarget,this);
        drivetrain.Drive(.4F,7,Direction.FORWARD);*/
    }
}
