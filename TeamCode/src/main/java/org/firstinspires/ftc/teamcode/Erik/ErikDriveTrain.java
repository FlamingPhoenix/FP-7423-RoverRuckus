package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;
import android.view.ViewDebug;

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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

public class ErikDriveTrain extends DriveTrain {

    // these constants can be used to proportionally control DC Motor power in response to error to target(angle, distance etc)
    public static final double HEADING_GAIN = 0.018;   //was playing with 0.04, orginal 0.018, Rate at which we respond to heading error, ie angle error
    public static final double LATERAL_GAIN = 0.05; //0.0027,  Rate at which we respond to off-axis error
    public static final double AXIAL_GAIN = 0.05;  // 0.0017, Rate at which we respond to target distance errors
    // above numbers are not necessary for our robot, it is just a starting point..

    public ErikDriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright) {
        super(frontleft, frontright, backleft, backright);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        // is there anything else to be added here ?

    }

    public ErikDriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright, LinearOpMode op) {
        super(frontleft, frontright, backleft, backright, op);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        // is there anything else to be added here ?

    }


    // note for all other functions, which erik didnt change, will not define here, just inherit directly from supoer class, 
    // like Drive(), Strafe(), Turn()[need to add logic to avoid non-stop turning], StopAll() etc.

    // add function used to be called DriveToImageErik, now changed name to StrafeToImage and then inherit from super class

    @Override
    public void StrafeToImage(float power, VuforiaTrackable imageTarget, LinearOpMode opMode, float safetyDistance) {

        //super.StrafeToImage(float power, VuforiaTrackable imageTarget, OpMode opMode) // no constructor for method  

        // following code is different from super class

        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();
        OpenGLMatrix pos1 = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getRobotLocation();
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

        String tag = "Msg before while ";
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
            {
                turn_flag = 1;
            } else if (y_angle < 170.0f & y_angle > 130.0f) // when x is minus, y angle is positive between +90 to  + 180
            {
                turn_flag = 1;
            }  // turn flag has sign, easier to understand visually, keep this line just in case we need to change the sign
            else {
                turn_flag = 0;
            }

            camToCenter = (float) Math.sqrt(x * x + y * y + d * d);
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

            while (Math.abs(d) >= 100 && op.opModeIsActive()) {
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
                    fl.setPower(actualPower + delta_x_power); // take it off(+turn_power)this is new for experiment, see if it helps with turn
                    fr.setPower(-(actualPower) + delta_x_power);
                    bl.setPower(-(actualPower) + delta_x_power + (turn_power * turn_flag)); // to turn CW ie, more parallel to image
                    br.setPower(actualPower + delta_x_power);


                } else if (x < -60f) {
                    if (System.currentTimeMillis() > t + 50) {
                        tag = "within elif; on left: ";
                        opMode.telemetry.addData("x distance:", x);
                        opMode.telemetry.addData("y distance", y);
                        opMode.telemetry.addData("z distance", d);
                        opMode.telemetry.addData("move to RIGHT towards center", x);
                        camToCenter = (float) Math.sqrt(x * x + y * y + d * d);
                        Log.i(tag, "z distance: " + Float.toString(d));
                        Log.i(tag, "x distance(move to right): " + Float.toString(x));
                        Log.i(tag, "y distance: " + Float.toString(y));
                        Log.i(tag, "y Angle: " + Float.toString(y_angle)); //orientation.secondAngle));
                        Log.i(tag, "camera->center dist (left): " + Float.toString(camToCenter));
                        opMode.telemetry.update();
                        t = t + 50;
                    }

                    fl.setPower(actualPower - delta_x_power - (turn_power * turn_flag)); //to turn CCW, ie more parallel to the image
                    fr.setPower(-(actualPower + delta_x_power));
                    bl.setPower(-(actualPower + delta_x_power));// take if off +turn_power, this is new, see if it helps with turn
                    br.setPower(actualPower - delta_x_power);
                } else {
                    if (System.currentTimeMillis() > t + 50) {
                        tag = "within else; on center: ";
                        opMode.telemetry.addData("y distance", y);
                        opMode.telemetry.addData("z distance", d);
                        opMode.telemetry.addData("x distance:", x);
                        opMode.telemetry.addData("move to RIGHT towards center", x);
                        camToCenter = (float) Math.sqrt(x * x + y * y + d * d);
                        Log.i(tag, "z distance: " + Float.toString(d));
                        Log.i(tag, "x distance(within center): " + Float.toString(x));
                        Log.i(tag, "y distance: " + Float.toString(y));
                        Log.i(tag, "y Angle: " + Float.toString(orientation.secondAngle));
                        Log.i(tag, "camera->center dist (center): " + Float.toString(camToCenter));
                        opMode.telemetry.update();
                        t = t + 50;
                    }

                    // think about if we need to check angle even if X is within desiable distance
                    if (y_angle > -170.0f & y_angle < -130.0f) { //y angle is minus when x is positive, should consider add delta_power flag, so formular is consistent
                        fl.setPower(actualPower); // take it off(+turn_power)this is new for experiment, see if it helps with turn
                        fr.setPower(-(actualPower));
                        bl.setPower(-(actualPower) + (turn_power * turn_flag)); // to turn CW ie, more parallel to image
                        br.setPower(actualPower);
                    } else if (y_angle < 170.0f & y_angle > 130.0f) { // y angle is positive when x is negative should consider add delta_power flag, so formular is consistent
                        fl.setPower(actualPower - (turn_power * turn_flag)); //to turn CCW, ie more parallel to the image
                        fr.setPower(-(actualPower));
                        bl.setPower(-(actualPower));// take if off +turn_power, this is new, see if it helps with turn
                        br.setPower(actualPower);
                    } else {
                        fl.setPower(actualPower);
                        fr.setPower(-(actualPower));
                        bl.setPower(-(actualPower));
                        br.setPower(actualPower);
                    }
                }

                tag = "dist - before update pose and location";
                Log.i(tag, " :rigth before update");
                pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();
                pos1 = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getRobotLocation();
                orientation = Orientation.getOrientation(pos1, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                Log.i(tag, ": after update");

                if (pos1 != null & pos != null) {
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
                        t = t + 50;
                    } else {
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


    // move the method to regular drive train for DE event Nov 27, then put back here
    @Override
    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, MyBoschIMU imu, OpMode opMode) {
        //super.TurnToImage(initialPower, d, imageTarget, imu); // no contructor for method

        // in Erik's local TurnToImageErik, OpMode is needed to collect status data, question for steve
        // should modify master DriveTrain, to add OpMode variable, then inherit, or just add here in Erik's branch ?

        // pasted as is for now..
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();

        //OpenGLMatrix pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getUpdatedRobotLocation();
        float x_turningVelocity; // = Math.abs(imu.getAngularVelocity().xRotationRate);
        float y_turningVelocity; // = Math.abs(imu.getAngularVelocity().yRotationRate);
        float z_turningVelocity; // = Math.abs(imu.getAngularVelocity().zRotationRate);

        float turningMax;
        float turningstep;
        float turningRobotSpeed;
        long sleepTime;

        turningstep = 0.01F;
        turningRobotSpeed = initialPower; // turningRobotSpeed = initialPower, here is reverse turn
        // turningRobotSpeed = turningMax;
        // add following code for new robot, testing new robot.
        opMode.telemetry.addData("just entered turning loop, initial turningRobotSpeed = ", turningRobotSpeed);
        opMode.telemetry.update();
        // can increase sleep time, so that the fast turn covers more angle change,and thus reduce slow turn time..
        sleepTime = 0; // it was 1001, change to 701 still too much, only need 45 degree for redsilver, so try 401

        //bl.setPower(turningRobotSpeed);
        //fl.setPower(turningRobotSpeed);
        //br.setPower(-turningRobotSpeed);
        //fr.setPower(-turningRobotSpeed);

        //x_turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);
        //y_turningVelocity = Math.abs(imu.getAngularVelocity().yRotationRate);
        //z_turningVelocity = Math.abs(imu.getAngularVelocity().zRotationRate);
        //sleepTime = (long) Math.floor(1000*4.10f/z_turningVelocity);
        //opMode.telemetry.addData("XRotationRate", x_turningVelocity);
        //opMode.telemetry.addData("yRotationRate", y_turningVelocity);
        //opMode.telemetry.addData("zRotationRate", z_turningVelocity);
        //opMode.telemetry.addData("turning power", turningRobotSpeed);
        //opMode.telemetry.addData("sleep time", sleepTime);
        //opMode.telemetry.update();
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (d == Direction.COUNTERCLOCKWISE) {
            turningRobotSpeed = -initialPower;
        }
        //turningRobotSpeed = - 0.18f; // turningRobotSpeed = 0.15f, at vinay, 0.13 ,at erik, 0.17


        while (pos == null && op.opModeIsActive()) {
            bl.setPower(turningRobotSpeed);
            fl.setPower(turningRobotSpeed);
            br.setPower(-turningRobotSpeed);
            fr.setPower(-turningRobotSpeed);

            opMode.telemetry.addData("in slow turning phase, before IMU call. turning power", turningRobotSpeed);

            x_turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);
            y_turningVelocity = Math.abs(imu.getAngularVelocity().yRotationRate);
            z_turningVelocity = Math.abs(imu.getAngularVelocity().zRotationRate);
            opMode.telemetry.addData("XRotationRate", x_turningVelocity);
            opMode.telemetry.addData("yRotationRate", y_turningVelocity);
            opMode.telemetry.addData("zRotationRate", z_turningVelocity);
            opMode.telemetry.addData("in slow turning phase, after IMU call. turning power", turningRobotSpeed);

            opMode.telemetry.update();

            //pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getUpdatedRobotLocation();
            pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();

//            if (turningRobotSpeed > 0.12f) {
            //               turningRobotSpeed = Math.abs(turningRobotSpeed) - turningstep; }
            //          else {
            //             turningRobotSpeed = 0.12f;
            //         }

            //turningRobotSpeed = 0.15f;

            //            sleepTime = (long) Math.floor((1/turningRobotSpeed)*sleepTime);
        }

        opMode.telemetry.addData("in turn to image routine, find target, before stop", 0);
        opMode.telemetry.update();
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
        opMode.telemetry.addData("in turn to image routine, RIGHT after stop", 0);
        opMode.telemetry.update();


    }

    @Override
    public void DriveStraight(float power, float distance, Direction d, MyBoschIMU myIMU, OpMode opMode) {

        // gear box ratio is not right, float x = 25.4F*(1120F * distance)/(4F * (float)Math.PI);// here distance is in mm
        float x = 2.54f * (PPR * distance) / (4F * (float) Math.PI); // still in inch ?
        int targetEncoderValue = Math.round(x);
        //float angleError = 0;
        //float startAngle;
        final float P_Drive_COEFF = 0.15f;   // this is to control proportionally or left or right power depending on current heading

        myIMU.resetAndStart(d);
        float startAngle = myIMU.getAngularOrientation().firstAngle;
        opMode.telemetry.addData("target first angle: ", startAngle);
//        opMode.telemetry.update();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentPosition = 0;


        if (d == Direction.BACKWARD) {
            power = -1 * power;
        }
        opMode.telemetry.addData("initial position encoder: ", currentPosition);
        opMode.telemetry.addData("target position: ", targetEncoderValue);
        opMode.telemetry.update();


        while (currentPosition < targetEncoderValue && op.opModeIsActive()) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            if (myIMU.getAngularOrientation().firstAngle > 3.0f) { // heading to left, need to make right adjustment)
                fl.setPower(power * (1 + P_Drive_COEFF));
                fr.setPower(power);
                bl.setPower(power * (1 + P_Drive_COEFF));
                br.setPower(power);
            } else if (myIMU.getAngularOrientation().firstAngle < -3.0f) { // heading to right
                fl.setPower(power);
                fr.setPower(power * (1 + P_Drive_COEFF));
                bl.setPower(power);
                br.setPower(power * (1 + P_Drive_COEFF));
            } else {
                fl.setPower(power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(power);
            }
            opMode.telemetry.addData("current position encoder: ", currentPosition);
            opMode.telemetry.addData("target position: ", targetEncoderValue);
            opMode.telemetry.addData("current angle: ", myIMU.getAngularOrientation().firstAngle);
            opMode.telemetry.update();
        }

        StopAll();

    }

    // can have proportional turn, proportional drive, p

    // below is proportional drive, ie, turn at normal speed and then slow down towards the end as we get closer to the target angle
    // consider using cliprange function, Range.clip(calculated Motor Power, -1, 1)

    @Override
    public void ProTurn(float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode) {

        Orientation startOrientation = imu.resetAndStart(d);

        float propower = power;
        float angle_Error = 0.0f;
        float targetAngle;
        float currentAngle;
        float startAngle;
        //float actualPower = power;

        if (d == Direction.CLOCKWISE) {
            //actualPower = -(power);

            targetAngle = startOrientation.firstAngle - angle;
            currentAngle = startOrientation.firstAngle;
            startAngle = currentAngle;
            while (currentAngle > targetAngle && op.opModeIsActive()) {

                //if ((currentPosition < 400)) {  // first 1/3 turn, slow start up
                //        actualPower = -.20F; //  min value for caprt = 0.18, value for dr. warner 0.14
                //    }


                angle_Error = Math.min(Math.abs(currentAngle - startAngle), Math.abs(currentAngle - targetAngle));
                propower = -(Math.max(0.17f, power * Range.clip(((float) HEADING_GAIN) * (angle_Error), -1, 1)));
                opMode.telemetry.addData("CW propower", propower);
                Log.i("CW ProTurn propower is ", Float.toString(propower));
                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                fl.setPower(-(propower));
                fr.setPower(propower);
                bl.setPower(-(propower));
                br.setPower(propower);
            }
        } else {
            //actualPower = power;

            targetAngle = startOrientation.firstAngle + angle;
            currentAngle = startOrientation.firstAngle;
            startAngle = currentAngle;

            while (currentAngle < targetAngle && op.opModeIsActive()) {

                //angle_Error = targetAngle - currentAngle;
                angle_Error = Math.min(Math.abs(currentAngle - startAngle), Math.abs(currentAngle - targetAngle));
                propower = (Math.max(0.17f, power * Range.clip(((float) HEADING_GAIN) * Math.abs(angle_Error), -1, 1)));
                opMode.telemetry.addData("CCW propower", propower);
                Log.i("CCWProTurn propower is ", Float.toString(propower));
                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                fl.setPower(-(propower));
                fr.setPower(propower);
                bl.setPower(-(propower));
                br.setPower(propower);
            }
        }


        StopAll();

    }

    // below is proportional drive, ie, drive normal speed and then slow down towards the end as we get closer to the target distance
    @Override
    public void ProDrive(float power, float distance, Direction d, LinearOpMode opMode) {
        float distance_Error = 0.0f;
        //float end_Ref = 0f; // end_Ref will be the smaller of distance or distance_Error, which controls starting or ending speed
        float pro_power = power;
        float x = (PPR * distance) / (4F * (float) Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        //added code below to support reverse driving, tested Oct 29, Erik did ofc this

        while (currentPosition < targetEncoderValue && opMode.opModeIsActive()) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            if (currentPosition < 200) {  // the goal is to start slow and also stop slow..
                if (d == Direction.FORWARD) {
                    pro_power = .18F;
                }  // was 0.2F
                else {
                    pro_power = -.18F;
                }
            } else {
                // using mm as unit for measuring distance error and pairing with axial-gain
                distance_Error = Math.abs(25.4f * (distance - currentPosition * (4F * (float) Math.PI) /PPR));
                opMode.telemetry.addData("distance_error", distance_Error);
                Log.i("distance error", Float.toString(distance_Error));
                pro_power = Math.max(0.15f, power * Range.clip(distance_Error * ((float) AXIAL_GAIN), -1, 1));
                opMode.telemetry.addData("propower, after clip ", pro_power);
                Log.i("aft-clip-prodrive-pwris", Float.toString(pro_power));
                if (d == Direction.BACKWARD) {
                    pro_power = -1 * pro_power;
                }
            }

            fl.setPower(pro_power);
            fr.setPower(pro_power);
            bl.setPower(pro_power);
            br.setPower(pro_power);
        }

        StopAll();

    }

    // will implement slow ramping up in the beginning and gradually slow down at the end


    //add N at end of Method, since added to regular drivetrain

    @Override
    public void ProStrafe(float power, float distance, Direction d, LinearOpMode opMode) {

        float distance_Error = 0.0f;
        float pro_power = power;
        float x = (PPR* 2 * distance) / (4F * (float) Math.PI);
        int targetEncoderValue = Math.round(x);

        //float actualPower = power;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && op.opModeIsActive()) {


            currentPosition = (Math.abs(fl.getCurrentPosition()));


            if ((currentPosition < 350) || ((targetEncoderValue - currentPosition) < 350)) {  // the goal is to start slow and also stop slow..
                if (d == Direction.LEFT) {
                    pro_power = -.25F; // was 0.20f
                }  // was 0.28F
                else {
                    pro_power = .25F;
                }
            } else {
                if (d == Direction.LEFT) {
                    pro_power = -(power);
                } else {
                    pro_power = power;
                }

            }
            //if(currentPosition < 200)
            //actualPower = .28F;
            fl.setPower(pro_power);
            fr.setPower(-(pro_power));
            bl.setPower(-(pro_power));
            br.setPower(pro_power);
        }

        StopAll();
    }

    @Override
    public OpenGLMatrix ObtainRobotCenterLocation(float initialPower, VuforiaTrackable imageTarget, MyBoschIMU imu, LinearOpMode opMode) {

        OpenGLMatrix lastLocation = null;
     //   final float mmPerInch = 25.4f;
       // final float mmBotWidth = 18 * mmPerInch;            // may need to change for new robot, 457.2

       // final float mmCamera_Forward_Displacement = 0.5f * mmBotWidth;//6.0f * mmPerInch;   // 229Camera is 110 mm in front of robot center
        //final float mmCamera_Height_Off_Ground = 4.25f * mmPerInch;   // Camera is 4.25 inch above ground, 108
       // final float mmCamera_Right_Displacement = 6.0f * mmPerInch; //-0.5f * mmBotWidth;// 152, Camera is ON the robots right handside,

        float robotX;         // X displacement from target center, which is distance to target along X axis(projection on X axis), negative
        float robotY;         // Y displacement from target center, which is lateral distance, negative if on right hand side of target.
        float robotOwnAngle;   // Robot's rotation around the Z axis (CCW is positive）
        float robotToImageDiagonalDistance;    // distance from robot's center to image in mm
        float angleToImage;  // Heading of the target , relative to the robot's unrotated center
        float totalRobotToImageAngle; //Heading to the image from the robot's current angle.

        // create an image translation/rotation matrix to be used for specified image
        // put image centers 6" (6*2.54 = 150) above the 0:0:0 origin, rotate it so it is along the -X axis.
        OpenGLMatrix robotLocationTransform = null;

        /* opMode.telemetry.addData("before image translation", "fisrt call to opGL");
        Log.i("[pheonix]:begin of call", "before openGL");
        OpenGLMatrix imageOrientation = OpenGLMatrix

                .translation(0, 0, 110) // 150

                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XYZ,

                        AngleUnit.DEGREES, 90, 0, -90));// third angle: -90));




         * Create a transformation matrix describing where the phone is on the robot.
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  Camera and screen will be
         * in "portrait Mode" with screen closest to right handside of robot, camera at bottom(upside down)
        opMode.telemetry.addData("before phone translation & rotation", "second call to opGL");
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix

                .translation(mmCamera_Forward_Displacement, mmCamera_Right_Displacement, mmCamera_Height_Off_Ground) //mmCamera_Forward_Displacement, -mmCamera_Right_Displacement, mmCamera_Height_Off_Ground

                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES,
                        -90, 0, 0));//third angel-90));  //
// x = -90, y, z = 0, forward = 6 inch, rigth = 9 inches..


        // Set image target to have the same location and camera orientation

        imageTarget.setLocation(imageOrientation);
        opMode.telemetry.addData("after image set location", "image set location");
        Log.i("[pheonix]:after image", "set location");
        ((VuforiaTrackableDefaultListener) imageTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.BACK);//param.cameraDirection);
        opMode.telemetry.addData("after phone set location", "phone set location");
        Log.i("[pheonix]:after phone", "phone set location");
        */

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) imageTarget.getListener();
        opMode.telemetry.addData("after set listener", "set listener");
        Log.i("[pheonix]:aft setlisten", "set listener");

        while (opMode.opModeIsActive()) {

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if ((listener != null) && listener.isVisible()) {

                robotLocationTransform = ((VuforiaTrackableDefaultListener) (VuforiaTrackableDefaultListener) imageTarget.getListener()).getUpdatedRobotLocation(); // change to get robotlocation, .getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                /**
                 * Provide feedback as to where the robot was last located (if we know).
                 */
                if (lastLocation != null) {
                    //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                    VectorF robotTranslation = lastLocation.getTranslation();
                    Orientation robotOrientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    robotX = robotTranslation.get(0);

                    robotY = robotTranslation.get(1);

                    // Robot angle (in +vc CCW cartesian system) is defined by the standard Matrix z rotation

                    robotOwnAngle = robotOrientation.thirdAngle;

                    // diagonal distance is based on distance from robot center position to image origin.

                    robotToImageDiagonalDistance = (float) Math.hypot(robotX, robotY);

                    // image angle is based on angle formed between the X axis to the diagonal line between robot center and image

                    angleToImage = (float) Math.toDegrees(-Math.asin(robotY / robotToImageDiagonalDistance));//(Math.atan(robotY / robotX));

                    // Target relative bearing is the target Heading relative to the direction the robot is pointing.

                    totalRobotToImageAngle = angleToImage - robotOwnAngle;

                    opMode.telemetry.addData("robot x = %f", robotX);
                    opMode.telemetry.addData("robot y = %f", robotY);
                    opMode.telemetry.addData("robot z = %f", robotTranslation.get(2));
                    opMode.telemetry.addData("diagonal distance = %f", robotToImageDiagonalDistance);
                    opMode.telemetry.addData("Robot angle to image = %f", angleToImage);
                    opMode.telemetry.addData("Robot own angle = %f", robotOwnAngle);
                    opMode.telemetry.addData("Robot total angle = %f", totalRobotToImageAngle);
                    opMode.telemetry.update();
                    Log.i("[phoenix]:Rob last Pos=", format(lastLocation));
                    Log.i("[phoenix]:get-trans x= ", Float.toString(robotX));
                    Log.i("[phoenix]:get-trans y=", Float.toString(robotY));
                    Log.i("[phoenix]:get-trans z=", Float.toString(robotTranslation.get(2)));
                    Log.i("[phoenix]:diag dist= ", Float.toString(robotToImageDiagonalDistance));
                    Log.i("[phoenix]:rob tot ang=", Float.toString(totalRobotToImageAngle));
                    Log.i("[phoenix]:ima angle2rob", Float.toString(angleToImage));
                    Log.i("[phoenix]:rob own-angle", Float.toString(robotOwnAngle));
                } else {
                    opMode.telemetry.addData("Pos", "Unknown");
                    Log.i("Robot last Pos =", "unknown");
                }


            }
        }
        return lastLocation;

    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}

