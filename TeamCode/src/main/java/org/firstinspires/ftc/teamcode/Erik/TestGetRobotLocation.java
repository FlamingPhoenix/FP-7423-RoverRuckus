package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.MyClass.MineralPositionViewModel;

import java.util.List;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="Erik Where is Robot", group="none")
public class TestGetRobotLocation extends ErikAutoBase {
    private ElapsedTime runtime = new ElapsedTime();
    private static final long  firstHitTime = 1250; // this is from calibration, it is time to detect first object
    private static final long secondHitTime = 5300; // this is time to hit 2nd object..need to calibrate
    private static final long thirdHitTime = 12000;

    private static final float  firstHitDistance = 3.5f; // this is from calibration, it is time to detect first object
    private static final float secondHitDistance = 15.5f; // this is time to hit 2nd object..need to calibrate
    private static final float thirdHitDistance = 27.5f;


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        telemetry.addData(">", "Press Start to get Robot location.");
        telemetry.update();
        // Lower the robot

        // Detach from the lander

        // Move forward 3 inches
        while (opModeIsActive()) {
            sleep(3000);
            erikdrivetrain.ObtainRobotCenterLocation(0.1f, frontTarget, imu, this);
        }
        //drivetrain.Drive(0.25F, 3F, Direction.FORWARD);
        //sleep(8000);

        // Turn counter clockwise to image

        //drivetrain.Turn(0.35F, 15, Direction.COUNTERCLOCKWISE, imu, this);
        //sleep(15000);


        //  Turn angle to mineral and drive forward certain amount to hit the mineral

        //  Drive backward a certain amount

        //  Turn counter clockwise to find next image

        //  Strafe to image

        //  Drive forward to depot

        //  Drop to market

        //  Park in crater







        //scanGold(0.12f);
        //sleep(300);
        //drivetrain.Turn(), first turn 100 - 120 degree, can test proturn
        //drivetrain.Turn(0.35f, 95, Direction.COUNTERCLOCKWISE, imu, this);
        // then turn to image
        //sleep(1000);
        //drivetrain.TurnToImage(0.13f, Direction.COUNTERCLOCKWISE, redTarget, imu, this);
        //strafe to image
        //drivetrain.StrafeToImage(0.25f, redTarget, this);
        //sleep(3000);

        //if (tfod != null) { // now it is ok to shutdown tfod/vuforia
        //    tfod.deactivate();
        //    tfod.shutdown();
        //}

        // drive backward for certain distance. here can also test prodrive
        //drivetrain.Drive(.4f, 58f, Direction.FORWARD);
        //sleep(300);
        //drivetrain.Drive(1.0f, 5, Direction.BACKWARD);//  drop marker
        //drivetrain.Drive(.65f, 70, Direction.BACKWARD); // continue to drive
    }

}
