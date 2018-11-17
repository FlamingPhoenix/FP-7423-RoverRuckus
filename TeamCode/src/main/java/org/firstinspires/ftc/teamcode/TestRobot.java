package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="VerifyBot", group="none")
public class TestRobot extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        drivetrain.Drive(0.5F, 10, Direction.FORWARD);
        sleep(2000);
        drivetrain.Drive( 0.5F, 10, Direction.BACKWARD);
        sleep(2000);
        drivetrain.Strafe(1F, 10, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(1F, 10, Direction.LEFT);
        sleep(2000);
        drivetrain.Turn(0.7F, 45, Direction.CLOCKWISE, imu, this);
        sleep(2000);
        drivetrain.Turn(0.7F, 45, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(2000);
        drivetrain.StopAll();



    }
}
