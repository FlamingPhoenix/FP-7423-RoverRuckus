package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

public class DriveTrain {

    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    public DriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright) {

        fr = frontright;
        fl = frontleft;
        br = backright;
        bl = backleft;


    }
    public void Strafe(float power, float distance, Direction d /*, OpMode op*/) {

        float x = (2240F * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue) {
            /*
            op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(actualPower);
            fr.setPower(-(actualPower));
            bl.setPower(-(actualPower));
            br.setPower(actualPower);
        }

        StopAll();
    }
    public void Drive(float power, float distance, Direction d) {

        float x = (1120F * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }

        StopAll();

    }

    public void Turn(float power, int angle, Direction d, BNO055IMU imu, OpMode opMode) {
        int startAngle = ((Math.round(imu.getAngularOrientation().firstAngle))+180);

        int targetAngle = startAngle + angle;

        int currentAngle = startAngle;

        float actualPower = power;
        if (d == Direction.CLOCKWISE) {
            actualPower = -(power);
        }
        while (currentAngle < targetAngle) {

            opMode.telemetry.addData("start:", startAngle);
            opMode.telemetry.addData("current:", currentAngle);
            opMode.telemetry.addData("target:", targetAngle);
            opMode.telemetry.addData("actual:", (currentAngle-180));
            opMode.telemetry.update();

            currentAngle = ((Math.round(imu.getAngularOrientation().firstAngle))+180);
            fl.setPower(-(actualPower));
            fr.setPower(actualPower);
            bl.setPower(-(actualPower));
            br.setPower(actualPower);
        }

        StopAll();

    }

    public void StrafeToImage(float power, VuforiaTrackable imageTarget, OpMode opMode) {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();
        //OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

        float actualPower = power;

        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            float x = pos.getColumn(3).get(0) * -1;
            float additionalpower = 0;


            while (Math.abs(d) >= 100) {
                pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

                Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
                OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
                Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                opMode.telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

                d = pos.getColumn(3).get(2);
                x = pos.getColumn(3).get(0) * -1;
                opMode.telemetry.addData("x: ", "x = %f", x);
                if(x > 15)
                {
                    additionalpower = -0.05F;
                }
                else if(x < -15)
                {
                    additionalpower = 0.05F;
                }

                float flTurnAdjust = 0;
                float blTurnAdjust = 0;

                if (adjustedOrientation.secondAngle < -3) {
                    flTurnAdjust = 0.2F * (Math.abs(adjustedOrientation.secondAngle) / 10);
                }
                else if (adjustedOrientation.secondAngle > 3) {
                    blTurnAdjust = -0.2F;
                }

                float flPower, frPower, blPower, brPower;

                flPower = actualPower + additionalpower + flTurnAdjust;
                frPower = -actualPower - additionalpower;
                blPower = -actualPower - additionalpower + blTurnAdjust;
                brPower = actualPower + additionalpower;

                float max = Max(flPower, frPower, blPower, brPower);

                fl.setPower(flPower/max);
                fr.setPower(frPower/max);
                bl.setPower(blPower/max);
                br.setPower(brPower/max);

                Log.i("[phoenix:StrafeToImage]", String.format("x = %f, additionalpower = %f", x, additionalpower));
                Log.i("[phoenix:StrafeToImage]", String.format("raw x=%f, y=%f, z=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle));
                Log.i("[phoenix:StrafeToImage]", String.format("adj x=%f, y=%f, z=%f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle));
                opMode.telemetry.update();
            }
        }
        StopAll();
    }

    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, BNO055IMU imu) {
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
        float turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);

        while (pos == null) {

        }
    }

    public void StopAll(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }


    private float Max(float x1, float x2, float x3, float x4) {
        x1 = Math.abs(x1);
        x2 = Math.abs(x2);
        x3 = Math.abs(x3);
        x4 = Math.abs(x4);
        float m = x1;

        if (x2 > m)
            m = x2;
        if (x3 > m)
            m = x3;
        if (x4 > m)
            m = x4;

        return m;
    }

}