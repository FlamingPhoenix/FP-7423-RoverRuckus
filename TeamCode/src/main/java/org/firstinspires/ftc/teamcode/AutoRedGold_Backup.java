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

@Disabled
@Autonomous(name="Auto Red Gold-Backup", group="none")
public class AutoRedGold_Backup extends AutoBase {
    // just in case for future reference
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();

        VuforiaTrackable backTarget = rover.get(3);
        // 3 = Nebula Picture
        backTarget.setName("back");

        waitForStart();

        /*
        drivetrain.Turn(0.15F,90,Direction.CLOCKWISE, imu, this);
        drivetrain.Drive(0.25F, 5, Direction.FORWARD);
        drivetrain.Turn(
                0.15F,90,Direction.COUNTERCLOCKWISE, imu, this);
*/
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)backTarget.getListener()).getPose();

        while (pose == null) {
            pose = ((VuforiaTrackableDefaultListener)backTarget.getListener()).getPose();
            bl.setPower(0.1);
            fl.setPower(0.1);
            br.setPower(-0.1);
            fr.setPower(-0.1);
        }

        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);

        drivetrain.StrafeToImage(.8F, backTarget, this);

    }
}
