package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.PositionToImage;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="My First Auto", group="none")
public class MyFirstAuto extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        //releaseFromLander();
       drivetrain.Turn(.5F, 90, Direction.CLOCKWISE, imu, this);
       sleep(1000);
       drivetrain.Turn(.5F, 45, Direction.CLOCKWISE, imu, this);
       sleep(1000);
       drivetrain.Turn(.5F, 90, Direction.COUNTERCLOCKWISE, imu, this);
    }
}
