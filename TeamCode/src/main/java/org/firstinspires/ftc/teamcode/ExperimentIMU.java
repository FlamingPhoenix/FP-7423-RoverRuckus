package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@TeleOp(name = "Expirement", group = "none")
public class ExperimentIMU extends LinearOpMode {


    MyBoschIMU imu;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain driveTrain;

    public void initialize() {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new DriveTrain(fl, fr, bl, br);


        imu = new MyBoschIMU(this.hardwareMap);   //Use our new MYBoscheIMU, instead of the original IMU; our MYBoscheIMU should make our code simpler
        imu.initialize(new BNO055IMU.Parameters());
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();


        //Call driveTrain to make a few turns here, see what happens
    }
}
