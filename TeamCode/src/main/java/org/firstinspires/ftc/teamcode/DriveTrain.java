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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.PositionToImage;

public class DriveTrain {

    protected DcMotor fr;
    protected DcMotor fl;
    protected DcMotor br;
    protected DcMotor bl;

    protected PositionToImage lastKnownPosition;

    protected LinearOpMode op;

   private float PPR = 560F;  // 560 for new robot 1120 for old robot


    public DriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright) {

        fr = frontright;
        fl = frontleft;
        br = backright;
        bl = backleft;

        lastKnownPosition = new PositionToImage(); //instantiate this first

    }

    public DriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright, LinearOpMode op) {

        fr = frontright;
        fl = frontleft;
        br = backright;
        bl = backleft;

        this.op = op;

        lastKnownPosition = new PositionToImage(); //instantiate this first

    }

    public void Strafe(float power, float distance, Direction d /*, OpMode op*/) {

        float x = (PPR * 2 * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && op.opModeIsActive()) {
            /*
            op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            currentPosition = (Math.abs(fl.getCurrentPosition()));

            //if(currentPosition < 200)
                //actualPower = .28F;

            fl.setPower(actualPower);
            fr.setPower(-(actualPower));
            bl.setPower(-(actualPower));
            br.setPower(actualPower);
        }

        StopAll();
    }

    public void Drive(float power, float distance, Direction d) {

        float x = (PPR * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        //added code below to support reverse driving, tested Oct 29, Erik did ofc this

        if (d == Direction.BACKWARD) {
            power = -1 * power;
        }

        while (currentPosition < targetEncoderValue && op.opModeIsActive()) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }

        StopAll();

    }

    public void Turn(float power, int angle, Direction d, MyBoschIMU imu, OpMode opMode) {

        Orientation startOrientation = imu.resetAndStart(d);

        float targetAngle;
        float currentAngle;
        float actualPower = power;
        float stoppingAngle = 0;

        if (d == Direction.CLOCKWISE) {
            actualPower = -(power);

            targetAngle = startOrientation.firstAngle - angle;
            currentAngle = startOrientation.firstAngle;

            while ((currentAngle - stoppingAngle) > targetAngle  && op.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v =  imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = ( 5/16 * (speed - 120)) + 30;

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }
        else {
            actualPower = power;

            targetAngle = startOrientation.firstAngle + angle;
            currentAngle = startOrientation.firstAngle;
            while ((currentAngle + stoppingAngle) < targetAngle  && op.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v =  imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = ( 5/16 * (speed - 120)) + 30;

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }


        StopAll();

    }

    public void StrafeToImage(float power, VuforiaTrackable imageTarget, OpMode opMode) {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();

        float actualPower = power;

        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            float x = pos.getColumn(3).get(0) * -1;
            float additionalpower = 0;


            while ((Math.abs(d) >= 100) && (imageListener.isVisible()) && op.opModeIsActive()) {
                pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

                Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
                OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
                Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                opMode.telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

                //Keep track the last known location
                lastKnownPosition.translation = pos.getTranslation();
                lastKnownPosition.orientation = adjustedOrientation;

                d = lastKnownPosition.translation.get(2);
                x = lastKnownPosition.translation.get(0) * -1;

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
                    blTurnAdjust = -0.2F * (Math.abs(adjustedOrientation.secondAngle) / 10);
                    // Aryan changed this; Copied Math.abs from first if branch into second branch
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

        float remainDistance = lastKnownPosition.translation.get(2) - 100;
        opMode.telemetry.addData("Remaining Distance: ", "x = %f", remainDistance);
        if (remainDistance > 30)
            this.Strafe(0.5F, remainDistance, Direction.RIGHT);
        opMode.telemetry.update();
     }

     public PositionToImage getLastKnownPosition() {
        return lastKnownPosition;
     }

    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, MyBoschIMU imu, OpMode opMode) {
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
        float turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);

        while (pos == null  && op.opModeIsActive()) {

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

    // is it ok to have a method and it is further defined in subclass, but not here, do I need virtual key word ?
    public void DriveStraight(float power, float distance, Direction d, MyBoschIMU myIMU, OpMode opMode){

    }
    // this is skeleton the detail is defined in ERIK DRIVETRAINNNNNN
    public void ProDrive(float power, float distance, Direction d, OpMode opMode ) {
    //
    }
    // this is skeleton the detail is defined in ERIK DRIVETRAINnnnnn
    public void ProTurn(float power, int angle, Direction d, MyBoschIMU imu, OpMode opMode) {

    }

    public boolean DriveUntilImageVisible (float power, Direction direction, float distanceLimit, VuforiaTrackable imageTarget, OpMode opMode) {
     //The function will move robot forward or backward until it sees the image, then stop
      // or until distanceLimit is completed but if it still can't see image, then also stop
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
        float x = (PPR * distanceLimit)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;
        opMode.telemetry.addData("init encode value",fl.getCurrentPosition());
        Log.i("init encode value", Integer.toString(fl.getCurrentPosition()));
        opMode.telemetry.addData("target encode value",targetEncoderValue);
        Log.i("distance error", Integer.toString(targetEncoderValue));

        if(direction == Direction.BACKWARD){
            power = -power;
        }

        while(pos == null && currentPosition < targetEncoderValue  && op.opModeIsActive()){
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
            pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
            currentPosition = fl.getCurrentPosition();
            opMode.telemetry.addData("current encode value",currentPosition);
            Log.i("init encode value", Integer.toString(currentPosition));
        }

        if(pos != null){
            return true;
        }
        StopAll();
        return false;
    }
}