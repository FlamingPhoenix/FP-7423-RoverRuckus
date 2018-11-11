package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="Auto Red Silver", group="none")
public class AutoRedSilver extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        VuforiaTrackables rover = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        rover.activate();

        VuforiaTrackable backTarget = rover.get(2);
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

        /*
        while (pose == null) {
            pose = ((VuforiaTrackableDefaultListener)backTarget.getListener()).getPose();
            bl.setPower(0.15);
            fl.setPower(0.15);
            br.setPower(-0.15);
            fr.setPower(-0.15);
        }

        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);

        */

        drivetrain.Turn(.3F, 10, Direction.COUNTERCLOCKWISE, imu, this);
        sleep(20000);
       /* drivetrain.Strafe(.4F, 6, Direction.LEFT);

        while (pose == null)
        {
            drivetrain.Turn(.4F, 10, Direction.CLOCKWISE,imu,this);
        }

        drivetrain.StrafeToImage(.4F, backTarget,this);
        drivetrain.Drive(.4F,7,Direction.FORWARD);*/
    }
}
