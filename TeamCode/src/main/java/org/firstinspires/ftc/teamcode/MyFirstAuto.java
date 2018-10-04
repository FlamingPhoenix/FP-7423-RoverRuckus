package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="My First Auto", group="none")
public class MyFirstAuto extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain drivetrain;
    BNO055IMU imu;

    public void initialize() {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        drivetrain = new DriveTrain(fl, fr, bl, br);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        drivetrain.Turn(0.5F,60,Direction.CLOCKWISE, imu);
        drivetrain.Drive(0.5F, 5, Direction.FORWARD);
        drivetrain.Turn(0.5F,60,Direction.COUNTERCLOCKWISE, imu);
        
    }
}
