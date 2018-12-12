package org.firstinspires.ftc.teamcode.Erik;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

import java.util.List;

@Disabled
@Autonomous(name="Marcus Test", group="none") //used to be called Red Gold, messed up Gold/Silver
//@TeleOp(name="Erik Auto Subclass", group="none")

public class TestStrafeToImage extends AutoBase {

    protected TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() <= 3) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            int goldleft = (int) recognition.getLeft();
                            int goldright = (int) recognition.getRight();
                            float goldcenter = (goldleft + goldright) / 2f;

                            telemetry.addData("[phoenix]", "gl: %f, gr: %f, gc: %f", goldleft, goldright, goldcenter);
                            telemetry.update();
                        }
                    }
                }
            }
            else {
                telemetry.addData("[phoenix]", "updatedRecognitions is null");
            }
        } else {
            telemetry.addData("[phoenix]", "TFOD is null");
        }

        //drivetrain.StrafeToImage(.3F, redTarget, this);

    }



}
