package org.firstinspires.ftc.teamcode.Erik;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;

/**
 * Created by Steve on 7/22/2018.
 */
//@Disabled
@Autonomous(name="Weight_Balance_StrafeTest", group="none")
public class StrafeTest_WeightBalance extends AutoBase {

    private float PPR = 1120F;  // 560 for new robot 1120 for old robot

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

       /* drivetrain.Strafe(1F, 5, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(1F, 5, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(1F, 10, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(1F, 10, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(1F, 20, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(1F, 20, Direction.LEFT); */

        sleep(6000);

        drivetrain.Strafe(.5F, 5, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.5F, 5, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.5F, 10, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.5F, 10, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.5F, 20, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.5F, 20, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.5F, 30, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.5F, 30, Direction.LEFT);

        Balanced_Strafe(.5F, 5, Direction.RIGHT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 5, Direction.LEFT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.RIGHT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.LEFT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.RIGHT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.LEFT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.RIGHT, 1.1f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.LEFT, 1.1f, this);

        Balanced_Strafe(.5F, 5, Direction.RIGHT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 5, Direction.LEFT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.RIGHT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.LEFT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.RIGHT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.LEFT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.RIGHT, 1.3f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.LEFT, 1.3f, this);

        Balanced_Strafe(.5F, 5, Direction.RIGHT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 5, Direction.LEFT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.RIGHT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.LEFT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.RIGHT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.LEFT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.RIGHT, 1.5f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.LEFT, 1.5f, this);

        Balanced_Strafe(.5F, 5, Direction.RIGHT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 5, Direction.LEFT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.RIGHT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 10, Direction.LEFT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.RIGHT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 20, Direction.LEFT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.RIGHT, 2f, this);
        sleep(2000);
        Balanced_Strafe(.5F, 30, Direction.LEFT, 2f, this);


        /*drivetrain.Strafe(.75F, 5, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.75F, 5, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.75F, 10, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.75F, 10, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.75F, 20, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.75F, 20, Direction.LEFT);
        sleep(6000);

        drivetrain.Strafe(.5F, 5, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.5F, 5, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.5F, 10, Direction.RIGHT);
        sleep(2000);


        drivetrain.Strafe(.5F, 10, Direction.LEFT);
        sleep(2000);
        drivetrain.Strafe(.5F, 20, Direction.RIGHT);
        sleep(2000);
        drivetrain.Strafe(.5F, 20, Direction.LEFT);
        sleep(2000); */

        drivetrain.StopAll();



    }

    public void Balanced_Strafe(float power, float distance, Direction d, float power_Ratio, LinearOpMode opMode) {

        float x = (2.0f*PPR * distance)/(4F * (float)Math.PI); // used to be a 2 at top. tried 1.5, seems ok
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue && opMode.opModeIsActive()) {
            /*
            op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            currentPosition = (Math.abs(fr.getCurrentPosition()));

            //if(currentPosition < 200)
            //actualPower = .28F;

            fl.setPower(actualPower);
            fr.setPower(-(actualPower));
            bl.setPower(-(Range.clip(actualPower*power_Ratio, -1.0f, 1.0f)));
            br.setPower(Range.clip(actualPower*power_Ratio, -1.0f, 1.0f));
        }

        drivetrain.StopAll();
    }


}
