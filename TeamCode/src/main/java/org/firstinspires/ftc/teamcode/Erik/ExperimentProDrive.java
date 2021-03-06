package org.firstinspires.ftc.teamcode.Erik;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Disabled
@Autonomous(name = "Erik Test ProDrive", group = "none")
public class ExperimentProDrive extends LinearOpMode {


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

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new ErikDriveTrain(fl, fr, bl, br, this);


        imu = new MyBoschIMU(this.hardwareMap);   //Use our new MYBoscheIMU, instead of the original IMU; our MYBoscheIMU should make our code simpler
        imu.initialize(new BNO055IMU.Parameters());
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        /*driveTrain.Turn(0.7f, 150, Direction.CLOCKWISE, imu, this);
        sleep(2000);
        driveTrain.Turn(0.7f, 150, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(2000);
        driveTrain.ProTurn(.7F, 150, Direction.CLOCKWISE, imu, this);
        sleep(2000);
        driveTrain.ProTurn(.7F, 150, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(2000); */
        driveTrain.Strafe(.4F, 25, Direction.LEFT);
        sleep(2000);
        driveTrain.Strafe(.4F, 25, Direction.RIGHT);
        sleep(2000);
        driveTrain.ProStrafe(.4F, 25, Direction.LEFT, this);
        sleep(2000);
        driveTrain.ProStrafe(.4F, 25, Direction.RIGHT, this);
        sleep(2000);
        sleep(2000);
        driveTrain.ProStrafe(.4F, 25, Direction.LEFT, this);
        sleep(2000);
        driveTrain.ProStrafe(.4F, 25, Direction.RIGHT, this);
        sleep(2000);
        driveTrain.Drive(.4F, 25, Direction.FORWARD);
        sleep(2000);
        driveTrain.Drive(.4F, 25, Direction.BACKWARD);
        sleep(2000);
        driveTrain.ProDrive(.4F, 25, Direction.FORWARD, this);
        sleep(2000);
        driveTrain.ProDrive(.4F, 25, Direction.BACKWARD, this);
        sleep(2000);
        driveTrain.ProDrive(.4F, 25, Direction.FORWARD, this);
        sleep(2000);
        driveTrain.ProDrive(.4F, 25, Direction.BACKWARD, this);
        sleep(2000);
    }
}
