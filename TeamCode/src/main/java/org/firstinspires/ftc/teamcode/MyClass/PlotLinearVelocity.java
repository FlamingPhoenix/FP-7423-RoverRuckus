package org.firstinspires.ftc.teamcode.MyClass;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name="Plot Linear Velocity", group="none")
public class PlotLinearVelocity extends LinearOpMode{
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

        imu = new MyBoschIMU(hardwareMap); // erik and Aryan made changes using myBoschIMU
        imu.initialize(new BNO055IMU.Parameters());

        waitForStart();


        for (float power = 1; power >= .15; power-=.05){
            float diffAverage = 0;
            float velocityAverage = 0;

            for (int i = 0; i < 4; i++){
                double startPosition = imu.getPosition().x;
                Drive(power);

                double v = Drive(power);
                sleep (8000);
                double currentPosition = imu.getPosition().x;

                velocityAverage += v;
                diffAverage += (currentPosition - startPosition);
            }

            diffAverage = diffAverage / 4;
            velocityAverage = velocityAverage / 4;

            Log.i("[phoenix:PlotLineVel]", String.format("%f: %f: %f",
                    power, diffAverage, velocityAverage));
        }

    }

    double Drive (float power) {
        int startPosition = 0;

        float x = (1120F * 10)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;
        currentPosition = startPosition;


        while (currentPosition < targetEncoderValue) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }

        Velocity v = imu.getVelocity();
        double speed = v.xVeloc;

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        return speed;
    }

}
