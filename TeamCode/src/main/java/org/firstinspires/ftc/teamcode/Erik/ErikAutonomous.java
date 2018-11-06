package org.firstinspires.ftc.teamcode.Erik;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name="Erik Auto Subclass", group="none")
public class ErikAutonomous extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DriveTrain drivetrain;  // why not ErikDriveTrain vs DriveTrain type ?
    MyBoschIMU imu;

    VuforiaLocalizer vuforia;

    public void initialize() {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        //*******************************************************/////
        ///Use Erik's Drive Train to experiment Erik's change
        drivetrain = new ErikDriveTrain(fl, fr, bl, br);

        // following code replaced by MyBoschIMU and its class

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled = true;
        //parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = new MyBoschIMU(hardwareMap);
        //imu.initialize(parameters);
        imu.initialize(new BNO055IMU.Parameters());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        vuforia = ClassFactory.getInstance().createVuforia(param);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();

        VuforiaTrackable backTarget = rover.get(2);
        backTarget.setName("front");

        waitForStart();

        drivetrain.Drive(0.2F, 17F, Direction.FORWARD);
        drivetrain.Strafe(0.4F, 17F, Direction.LEFT);

        /*
        drivetrain.Turn(0.15F,90,Direction.CLOCKWISE, imu, this);
        drivetrain.Drive(0.25F, 5, Direction.FORWARD);
        drivetrain.Turn(
                0.15F,90,Direction.COUNTERCLOCKWISE, imu, this);
*/
        //OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)backTarget.getListener()).getPose();

        //while (pose == null) {
        //    pose = ((VuforiaTrackableDefaultListener)backTarget.getListener()).getPose();
        drivetrain.TurnToImage(0.4F, Direction.CLOCKWISE, backTarget, imu, this); // at vinay, 0.4, at erik, can 0.4 or 0.5
            //bl.setPower(0.13);
            //fl.setPower(0.13);
            //br.setPower(-0.13);
            //fr.setPower(-0.13);
        //


        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);

        drivetrain.StrafeToImage(0.3F, backTarget, this);  //

        // drive backward for certain distance.
        drivetrain.Drive(0.2f, 39f, Direction.BACKWARD);

        // can try drive straight..a method defined in subclass and just a skeleton at super class(DriveTrain)
        //drivetrain.DriveStraight(0.2f, 40f, Direction.FORWARD, imu, this);

    }
}
