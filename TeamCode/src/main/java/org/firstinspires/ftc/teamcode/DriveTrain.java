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
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.PositionToImage;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class DriveTrain {

    protected DcMotor fr;
    protected DcMotor fl;
    protected DcMotor br;
    protected DcMotor bl;

    protected PositionToImage lastKnownPosition;

    protected LinearOpMode op;

    // these constants can be used to proportionally control DC Motor power in response to error to target(angle, distance etc)
    public  static final double  HEADING_GAIN       =  0.018;   //was playing with 0.04, orginal 0.018, Rate at which we respond to heading error, ie angle error
    public  static final double  LATERAL_GAIN   =  0.05; //0.0027,  Rate at which we respond to off-axis error
    public  static final double  AXIAL_GAIN     =  0.05;  // 0.0017, Rate at which we respond to target distance errors

    public float PPR = 1120F;  // 560 for new robot 1120 for old robot


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

        float x = (2.0f*PPR * distance)/(4F * (float)Math.PI); // used to be a 2 at top. tried 1.5, seems ok
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int initialPosition = fl.getCurrentPosition();
        int positionDiff = 0;

        while (positionDiff < targetEncoderValue && op.opModeIsActive()) {
            /*
            op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            positionDiff = Math.abs(fl.getCurrentPosition() - initialPosition);

            //if(currentPosition < 200)
                //actualPower = .28F;

            float flPower, frPower, blPower, brPower;

            flPower = actualPower;
            frPower = -actualPower * 1.2F;
            blPower = -actualPower;
            brPower = actualPower;

            float max = Max(flPower, frPower, blPower, brPower);

            fl.setPower(flPower/max);
            fr.setPower(frPower/max);
            bl.setPower(blPower/max);
            br.setPower(brPower/max);
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

    public void Turn(float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode) {

        Orientation startOrientation = imu.resetAndStart(d);

        float targetAngle;
        float currentAngle;
        float actualPower = power;
        float stoppingAngle = 0;

        if (d == Direction.CLOCKWISE) {
            actualPower = -(power);

            targetAngle = startOrientation.firstAngle - angle;
            currentAngle = startOrientation.firstAngle;

            while ((currentAngle - stoppingAngle) > targetAngle  && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v =  imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs(( 0.25f * speed) - 7.5f);
                Log.i("[phoenix:turnTest]", String.format("StartingAngle=%f, CurrentAngle=%f, AngularVelocity=%f, StoppingAngle=%f", startOrientation.firstAngle, currentAngle, speed, stoppingAngle));

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
            while ((currentAngle + stoppingAngle) < targetAngle  && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v =  imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs(( 0.25f * speed) - 7.5f);

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }


        StopAll();

    }


    public void StrafeToImage(float power, VuforiaTrackable imageTarget, LinearOpMode opMode) {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();

        float actualPower = power;

        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            float x = pos.getColumn(3).get(0) * -1;
            float additionalpower = 0;


            while ((Math.abs(d) >= 200) && (imageListener.isVisible()) && opMode.opModeIsActive()) {
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

                float distanceAdjustment = Math.abs(d);
                if (distanceAdjustment > 1200F)
                    distanceAdjustment = 1200F;
                else if (distanceAdjustment < 700F)
                    distanceAdjustment = 0;

                if(x > 15)
                {
                    additionalpower = actualPower * 0.5F * (Math.abs(x) / 150F) * ((1200F-distanceAdjustment) / 1200F);
                }
                else if(x < -15)
                {
                    additionalpower = actualPower * -0.5F * (Math.abs(x) / 150F) * ((1200F-distanceAdjustment) / 1200F);
                }

                float flTurnAdjust = 0;
                float blTurnAdjust = 0;
                float rotationMargin = 3f * Math.abs(d) / 10f;


                if (adjustedOrientation.secondAngle < -3 && x > -rotationMargin) {
                    flTurnAdjust = actualPower * 1.25F * (Math.abs(adjustedOrientation.secondAngle) / 40F) * ((1200F-distanceAdjustment) / 1200F);
                }
                else if (adjustedOrientation.secondAngle > 3 && x < rotationMargin) {
                    blTurnAdjust = actualPower * -1.25F * (Math.abs(adjustedOrientation.secondAngle) / 40F) * ((1200F-distanceAdjustment) / 1200F);
                }

                float flPower, frPower, blPower, brPower;

                flPower = actualPower + additionalpower + flTurnAdjust;
                frPower = (-actualPower + additionalpower) * 1.2F;
                blPower = -actualPower + additionalpower + blTurnAdjust;
                brPower = actualPower + additionalpower;

                float max = Max(flPower, frPower, blPower, brPower);

                fl.setPower(flPower/max);
                fr.setPower(frPower/max);
                bl.setPower(blPower/max);
                br.setPower(brPower/max);

                Log.i("[phoenix:StrafeToImage]", String.format("x = %f, d = %f, addpower = %f, actpower = %f, distadj = %f", x, d, additionalpower, actualPower, distanceAdjustment));
                // Log.i("[phoenix:StrafeToImage]", String.format("raw x=%f, y=%f, z=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle));
                Log.i("[phoenix:StrafeToImage]", String.format("adj x=%f, y=%f, z=%f, flTurnAdjust=%f, blTurnAdjust=%f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle, flTurnAdjust, blTurnAdjust));
                opMode.telemetry.update();
            }
        }
        StopAll();

        float remainDistance = (Math.abs(lastKnownPosition.translation.get(2)) - 100) * .0254F;
        opMode.telemetry.addData("Remaining Distance: ", "x = %f", remainDistance);
        if (remainDistance > 4F)
            this.Strafe(0.5F, remainDistance, Direction.RIGHT);
        opMode.telemetry.update();
     }



    public PositionToImage getLastKnownPosition() {
        return lastKnownPosition;
     }

    // put an O there to designate old version of uncompleted method
    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, MyBoschIMU imu, OpMode opMode) {
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
        //float turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);

        while (pos == null  && op.opModeIsActive()) {
            pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
            opMode.telemetry.addData("in turn to image", "while loop");
            Log.i("[phoenix]:in turn2image", "while loop");
        }
        opMode.telemetry.addData("in turn to image", "out of while loop");
        Log.i("[phoenix]:in turn2image", "out of while loop");
    }

    // copied from ErikDriveTrain.TurnToImage in for DE match, since existing TurnToImage not working yet,

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

    // is it ok to have a method and it is further defined in subclass, but not here, do I need virtual key word ?
    public void DriveStraight(float power, float distance, Direction d, MyBoschIMU myIMU, OpMode opMode){

    }

    // this is skeleton the detail is defined in ERIK DRIVETRAINNNNNN
    public void ProDrive(float power, float distance, Direction d, LinearOpMode opMode ) {
        //        //
    }

    // this is skeleton the detail is defined in ERIK DRIVETRAINnnnnn
    public void ProTurn(float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode) {
        }

    // this is skeleton declaration, details is defined in Erik DriveTrain
    //  added here for test in DE event
    public void ProStrafe(float power, float distance, Direction d , LinearOpMode opMode) {
    }

    // skeleton declaration
    /*public OpenGLMatrix ObtainRobotCenterLocation(float initialPower, VuforiaTrackable imageTarget, MyBoschIMU imu, OpMode opMode) {
        OpenGLMatrix robotLocationTransform = null;
        return robotLocationTransform;
    }*/

    public OpenGLMatrix ObtainRobotCenterLocation(float initialPower, VuforiaTrackable imageTarget, MyBoschIMU imu, LinearOpMode opMode) {

        OpenGLMatrix lastLocation = null;

        return lastLocation;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}