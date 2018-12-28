package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name="Plot Angular Velocity", group="none")
public class PlotAngularVelocity extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain drivetrain;
    MyBoschIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //drivetrain = new DriveTrain(fl, fr, bl, br);
        imu = new MyBoschIMU(hardwareMap); // erik and Aryan made changes using myBoschIMU
        imu.initialize(new BNO055IMU.Parameters());

        waitForStart();

        //Log.i("[phoenix:PlotAngVel]", String.format("Initiating loop"));
        for (float power = 1; power >= .40; power-=.05){

            float diffAverage = 0;
            float velocityAverage = 0;

            for (int i = 0; i < 4; i++){

                Orientation startOrientation = imu.resetAndStart(Direction.COUNTERCLOCKWISE);
                float startAngle = 0;
                if (startOrientation != null)
                    startAngle = startOrientation.firstAngle;

                float v = Turn(power);
                sleep (3000);
                float currentAngle = imu.getAngularOrientation().firstAngle;

                velocityAverage += v;
                diffAverage += (currentAngle - startAngle);


            }

            diffAverage = diffAverage / 4f;
            velocityAverage = velocityAverage / 4f;

            Log.i("[phoenix:PlotAngVel]", String.format("%f: %f: %f",
                    power, diffAverage, velocityAverage));
        }

        sleep(300000);

    }

    float Turn(float power) {



        //Log.i("[phoenix:PlotAngVel]", String.format("Beginning Turn at %f power.", power))
        Orientation startOrientation = imu.resetAndStart(Direction.COUNTERCLOCKWISE);
         float startAngle = 0;
         if (startOrientation != null)
            startAngle = startOrientation.firstAngle;

         float endingAngle = startAngle - 45;
         float currentAngle = startAngle;

         while (currentAngle > endingAngle) {
             fl.setPower(power);
             bl.setPower(power);
             fr.setPower(-power);
             br.setPower(-power);
             currentAngle = imu.getAngularOrientation().firstAngle;
         }

        //Log.i("[phoenix:PlotAngVel]", String.format("Getting angular velocity"));
        AngularVelocity v =  imu.getAngularVelocity();
        float speed = v.xRotationRate;

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        //Log.i("[phoenix:PlotAngVel]", String.format("speed = %f", speed));
        //Log.i("[phoenix:PlotAngVel)", String.format("additionalAngle = %f", (currentAngle + 90)));
        return speed;
    }
}
