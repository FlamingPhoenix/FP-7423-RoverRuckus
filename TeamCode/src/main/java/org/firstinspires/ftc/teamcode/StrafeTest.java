package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Steve on 7/22/2018.
 */
//@Disabled
@Autonomous(name="StrafeTest", group="none")
public class StrafeTest extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

//        drivetrain.Strafe(1F, 5, Direction.RIGHT);
////        sleep(2000);
////        drivetrain.Strafe(1F, 5, Direction.LEFT);
////        sleep(2000);
////        drivetrain.Strafe(1F, 10, Direction.RIGHT);
////        sleep(2000);
////        drivetrain.Strafe(1F, 10, Direction.LEFT);
////        sleep(2000);
        drivetrain.Strafe(0.5F, 20, Direction.RIGHT);
//        sleep(2000);
//        drivetrain.Strafe(1F, 20, Direction.LEFT);
//
//        sleep(6000);
//
//        drivetrain.Strafe(.75F, 5, Direction.RIGHT);
//        sleep(2000);
//        drivetrain.Strafe(.75F, 5, Direction.LEFT);
//        sleep(2000);
//        drivetrain.Strafe(.75F, 10, Direction.RIGHT);
//        sleep(2000);
//        drivetrain.Strafe(.75F, 10, Direction.LEFT);
//        sleep(2000);
//        drivetrain.Strafe(.75F, 20, Direction.RIGHT);
//        sleep(2000);
//        drivetrain.Strafe(.75F, 20, Direction.LEFT);
//
//        sleep(6000);
//
//        drivetrain.Strafe(.5F, 5, Direction.RIGHT);
//        sleep(2000);
//        drivetrain.Strafe(.5F, 5, Direction.LEFT);
//        sleep(2000);
//        drivetrain.Strafe(.5F, 10, Direction.RIGHT);
//        sleep(2000);

        drivetrain.StopAll();



    }
}
