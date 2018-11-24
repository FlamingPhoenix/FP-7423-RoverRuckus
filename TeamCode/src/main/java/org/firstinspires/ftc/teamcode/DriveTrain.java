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

   private float PPR = 1120F;  // 560 for new robot 1120 for old robot


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
                stoppingAngle = (( 5f/16f * (speed - 120f)) + 30f);
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
                stoppingAngle = ( 5f/16f * (speed - 120f)) + 30f;

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


            while ((Math.abs(d) >= 100) && (imageListener.isVisible()) && opMode.opModeIsActive()) {
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
                    blTurnAdjust = -0.2F * (Math.abs(adjustedOrientation.secondAngle) / 10);
                }
                else if (adjustedOrientation.secondAngle > 3) {
                    flTurnAdjust = 0.2F * (Math.abs(adjustedOrientation.secondAngle) / 10);
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


    public void StrafeToImageEric(float power, VuforiaTrackable imageTarget, LinearOpMode opMode)   {

        //super.StrafeToImage(float power, VuforiaTrackable imageTarget, OpMode opMode) // no constructor for method

        // following code is different from super class

        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
        OpenGLMatrix pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getRobotLocation();
        //pose = ((VuforiaTrackableDefaultListener) redWall.getListener()).getUpdatedRobotLocation();
        Orientation orientation = Orientation.getOrientation(pos1, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        float actualPower = power;
        float delta_x_power = 0.075f;  // at vinay, 0.075, at erik, 0.08
        float x;
        float d;
        float y;
        float camToCenter;
        float turn_power = 0.1f;  // at vinay, 0.1, at erik 0.05
        float y_angle; // control varialble for turning, instead of x
        int turn_flag = 0; // for turning based on y angle.

        String tag = "[phoenix]";
        long t = System.currentTimeMillis();
        opMode.telemetry.addData("before checking for pos in drive to image routine", 0);
        opMode.telemetry.addData("system time = ", t);
        opMode.telemetry.update();

        if (pos != null & pos1 != null) {
            //pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

            //pose = ((VuforiaTrackableDefaultListener) redWall.getListener()).getUpdatedRobotLocation();
            d = pos1.getColumn(3).get(2); //distance to the image in millimeter;
            //x = pos.getColumn(3).get(0); // distance for x, ie, is robot at the left or right of image
            x = pos1.getColumn(3).get(0);
            y = pos1.getColumn(3).get(1); // distance for Y, ie, is robot above or below center point of image
            y_angle = orientation.secondAngle;

            if (y_angle > -170.0f & y_angle < -130.0f)  // when x is positive, y angle is minus, between -90 and -180
            { turn_flag = 1;}
            else if (y_angle < 170.0f & y_angle > 130.0f) // when x is minus, y angle is positive between +90 to  + 180
            { turn_flag = 1;}  // turn flag has sign, easier to understand visually, keep this line just in case we need to change the sign
            else
            {turn_flag = 0;}

            camToCenter = (float) Math.sqrt(x*x+y*y+d*d);
            opMode.telemetry.addData("z distance: ", d);
            opMode.telemetry.addData("x distance: ", x);
            opMode.telemetry.addData("y distance: ", y);
            opMode.telemetry.addData("y Angle: ", orientation.secondAngle);
            opMode.telemetry.addData("camera->center dist: ", camToCenter);
            Log.i(tag, "z distance: " + Float.toString(d));
            Log.i(tag, "x distance: " + Float.toString(x));
            Log.i(tag, "y distance: " + Float.toString(y));
            Log.i(tag, "y Angle: " + Float.toString(y_angle));  //orientation.secondAngle));
            Log.i(tag, "camera->center dist: " + Float.toString(camToCenter));
            //opMode.telemetry.addData("y distance:", y);
            //opMode.telemetry.addData("x Angle:", orientation.firstAngle);
            //opMode.telemetry.addData("y Angle:", orientation.secondAngle);
            //opMode.telemetry.addData("z Angle:", orientation.thirdAngle);
            opMode.telemetry.update();

            while (Math.abs(d) >= 100  && opMode.opModeIsActive()) {
                //pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

                if (x > 60f) {
                    if (System.currentTimeMillis() > t + 50) {
                        tag = "within if; on right: ";
                        opMode.telemetry.addData("x distance:", x);
                        opMode.telemetry.addData("y distance", y);
                        opMode.telemetry.addData("z distance", d);
                        opMode.telemetry.addData("move to LEFT towards center", x);
                        camToCenter = (float) Math.sqrt(x * x + y * y + d * d);
                        Log.i(tag, "z distance: " + Float.toString(d));
                        Log.i(tag, "x distance(move to left): " + Float.toString(x));
                        Log.i(tag, "y distance: " + Float.toString(y));
                        Log.i(tag, "y Angle: " + Float.toString(orientation.secondAngle));
                        Log.i(tag, "camera->center dist (right): " + Float.toString(camToCenter));
                        opMode.telemetry.update();
                        t = t + 50;
                    }
                    fl.setPower(actualPower+delta_x_power); // take it off(+turn_power)this is new for experiment, see if it helps with turn
                    fr.setPower(-(actualPower)+delta_x_power);
                    bl.setPower(-(actualPower)+delta_x_power+(turn_power*turn_flag)); // to turn CW ie, more parallel to image
                    br.setPower(actualPower+delta_x_power);



                }
                else if (x < - 60f){
                    if (System.currentTimeMillis() > t + 50) {
                        tag = "within elif; on left: ";
                        opMode.telemetry.addData("x distance:", x);
                        opMode.telemetry.addData("y distance", y);
                        opMode.telemetry.addData("z distance", d);
                        opMode.telemetry.addData("move to RIGHT towards center", x);
                        camToCenter = (float) Math.sqrt(x*x+y*y+d*d);
                        Log.i(tag, "z distance: " + Float.toString(d));
                        Log.i(tag, "x distance(move to right): " + Float.toString(x));
                        Log.i(tag, "y distance: " + Float.toString(y));
                        Log.i(tag, "y Angle: " + Float.toString(y_angle)); //orientation.secondAngle));
                        Log.i(tag, "camera->center dist (left): " + Float.toString(camToCenter));
                        opMode.telemetry.update();
                        t = t + 50;
                    }

                    fl.setPower(actualPower-delta_x_power - (turn_power*turn_flag)); //to turn CCW, ie more parallel to the image
                    fr.setPower(-(actualPower+delta_x_power));
                    bl.setPower(-(actualPower+delta_x_power));// take if off +turn_power, this is new, see if it helps with turn
                    br.setPower(actualPower-delta_x_power);
                }
                else {
                    if (System.currentTimeMillis() > t + 50) {
                        tag = "within else; on center: ";
                        opMode.telemetry.addData("y distance", y);
                        opMode.telemetry.addData("z distance", d);
                        opMode.telemetry.addData("x distance:", x);
                        opMode.telemetry.addData("move to RIGHT towards center", x);
                        camToCenter = (float) Math.sqrt(x*x+y*y+d*d);
                        Log.i(tag, "z distance: " + Float.toString(d));
                        Log.i(tag, "x distance(within center): " + Float.toString(x));
                        Log.i(tag, "y distance: " + Float.toString(y));
                        Log.i(tag, "y Angle: " + Float.toString(orientation.secondAngle));
                        Log.i(tag, "camera->center dist (center): " + Float.toString(camToCenter));
                        opMode.telemetry.update();
                        t = t + 50;}

                    // think about if we need to check angle even if X is within desiable distance
                    if (y_angle > -170.0f & y_angle < -130.0f) { //y angle is minus when x is positive, should consider add delta_power flag, so formular is consistent
                        fl.setPower(actualPower); // take it off(+turn_power)this is new for experiment, see if it helps with turn
                        fr.setPower(-(actualPower));
                        bl.setPower(-(actualPower)+(turn_power*turn_flag)); // to turn CW ie, more parallel to image
                        br.setPower(actualPower);
                    }
                    else if ( y_angle < 170.0f & y_angle > 130.0f) { // y angle is positive when x is negative should consider add delta_power flag, so formular is consistent
                        fl.setPower(actualPower- (turn_power*turn_flag)); //to turn CCW, ie more parallel to the image
                        fr.setPower(-(actualPower));
                        bl.setPower(-(actualPower));// take if off +turn_power, this is new, see if it helps with turn
                        br.setPower(actualPower);
                    }
                    else {
                        fl.setPower(actualPower);
                        fr.setPower(-(actualPower));
                        bl.setPower(-(actualPower));
                        br.setPower(actualPower);
                    }
                }

                tag = "dist - before update pose and location";
                Log.i(tag, " :rigth before update");
                pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
                pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getRobotLocation();
                orientation = Orientation.getOrientation(pos1, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                Log.i(tag, ": after update");

                if (pos1 != null & pos != null ) {
                    if (System.currentTimeMillis() > t + 50) {
                        tag = "after checking x: ";
                        d = pos1.getColumn(3).get(2);
                        x = pos1.getColumn(3).get(0);
                        y = pos1.getColumn(3).get(1);
                        opMode.telemetry.addData("new z distance:", d);
                        opMode.telemetry.addData("new x distance:", x);
                        opMode.telemetry.addData("new y distance", y);
                        opMode.telemetry.addData("y Angle:", orientation.secondAngle);
                        camToCenter = (float) Math.sqrt(x * x + y * y + d * d);
                        Log.i(tag, "z distance: " + Float.toString(d));
                        Log.i(tag, "x distance(after checking x): " + Float.toString(x));
                        Log.i(tag, "y distance: " + Float.toString(y));
                        Log.i(tag, "y Angle: " + Float.toString(orientation.secondAngle));
                        Log.i(tag, "camera->center dist: " + Float.toString(camToCenter));
                        //opMode.telemetry.addData("new x Angle:", orientation.firstAngle);
                        //opMode.telemetry.addData("new y Angle:", orientation.secondAngle);
                        //opMode.telemetry.addData("new z Angle:", orientation.thirdAngle);
                        opMode.telemetry.update();
                        t = t + 50;}

                    else {
                        opMode.telemetry.addData("pos or pos1 is null, x is ", x);
                        opMode.telemetry.addData("pos or pos1 is null, z is", d);
                        opMode.telemetry.update();
                        Log.i(tag, "pos or pos1 is null" + " x is: " + Float.toString(x) + " and z is: " + Float.toString(d));
                    }

                }


            }

        }

        opMode.telemetry.update();
        StopAll();
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

    // this is skeleton declaration, details is defined in Erik DriveTrain
    public void ProStrafe(float power, float distance, Direction d , OpMode opMode) {

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