package org.firstinspires.ftc.teamcode.Eric;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;

public class EricDriveTrain extends DriveTrain {

    public EricDriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright)  {
        super(frontleft, frontright, backleft, backright);
        
      // is there anything else to be added here ?     
        
    }
    
    
    // note for all other functions, which erik didnt change, will not define here, just inherit directly from supoer class, 
    // like Drive(), Strafe(), Turn()[need to add logic to avoid non-stop turning], StopAll() etc.

    // add function used to be called DriveToImageErik, now changed name to StrafeToImage and then inherit from super class
    
    
    @Override
    
    public void StrafeToImage(float power, VuforiaTrackable imageTarget, OpMode opMode)   {
        
        //super.StrafeToImage(float power, VuforiaTrackable imageTarget, OpMode opMode) // no constructor for method  
            
            // following code is different from super class
        
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
       OpenGLMatrix pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getRobotLocation();
        //pose = ((VuforiaTrackableDefaultListener) redWall.getListener()).getUpdatedRobotLocation();
        Orientation orientation = Orientation.getOrientation(pos1, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        float actualPower = power;
        float delta_x_power = 0.08f;  // at vinay, 0.075, at erik, 0.08
        float x;
        float d;
        float y;
        float camToCenter;
        float turn_power = 0.05f;  // at vinay, 0.1, at erik 0.05
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

            while (Math.abs(d) >= 150) {
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
    
    
    @Override
    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, BNO055IMU imu) {
        //super.TurnToImage(initialPower, d, imageTarget, imu); // no contructor for method
        
          // in Erik's local TurnToImageErik, OpMode is needed to collect status data, question for steve
        // should modify master DriveTrain, to add OpMode variable, then inherit, or just add here in Erik's branch ? 
        
        // pasted as is for now..
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

        //OpenGLMatrix pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getUpdatedRobotLocation();
        float x_turningVelocity; // = Math.abs(imu.getAngularVelocity().xRotationRate);
        float y_turningVelocity; // = Math.abs(imu.getAngularVelocity().yRotationRate);
        float z_turningVelocity; // = Math.abs(imu.getAngularVelocity().zRotationRate);

        float turningMax;
        float turningstep;
        float turningRobotSpeed;
        long sleepTime;

        turningstep = 0.01F;
        turningRobotSpeed = initialPower;
       // turningRobotSpeed = turningMax;
        //opMode.telemetry.addData("XRotationRate", turningVelocity);
        //opMode.telemetry.update();
       sleepTime = 1001;

        bl.setPower(turningRobotSpeed);
        fl.setPower(turningRobotSpeed);
        br.setPower(-turningRobotSpeed);
        fr.setPower(-turningRobotSpeed);

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

        turningRobotSpeed = 0.17f; // at vinay, 0.13 ,at erik, 0.17


        while (pos == null) {
            bl.setPower(turningRobotSpeed);
            fl.setPower(turningRobotSpeed);
            br.setPower(-turningRobotSpeed);
            fr.setPower(-turningRobotSpeed);

            x_turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);
            y_turningVelocity = Math.abs(imu.getAngularVelocity().yRotationRate);
            z_turningVelocity = Math.abs(imu.getAngularVelocity().zRotationRate);

            opMode.telemetry.addData("XRotationRate", x_turningVelocity);
            opMode.telemetry.addData("yRotationRate", y_turningVelocity);
            opMode.telemetry.addData("zRotationRate", z_turningVelocity);
            opMode.telemetry.addData("turning power", turningRobotSpeed);

            opMode.telemetry.update();

            //pos1 = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getUpdatedRobotLocation();
            pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

//            if (turningRobotSpeed > 0.12f) {
 //               turningRobotSpeed = Math.abs(turningRobotSpeed) - turningstep; }
  //          else {
   //             turningRobotSpeed = 0.12f;
   //         }

            //turningRobotSpeed = 0.15f;

            //            sleepTime = (long) Math.floor((1/turningRobotSpeed)*sleepTime);
        }
 
        opMode.telemetry.addData("in turn to image routing, find target, before stop", 0);
        opMode.telemetry.update();
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
        opMode.telemetry.addData("in turn to image routing, RIGHT after stop", 0);
        opMode.telemetry.update();
 
        
    }
}
