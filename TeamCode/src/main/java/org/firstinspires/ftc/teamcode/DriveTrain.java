package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

public class DriveTrain {

    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    public DriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright) {

        fr = frontright;
        fl = frontleft;
        br = backright;
        bl = backleft;


    }
    public void Strafe(float power, float distance, Direction d /*, OpMode op*/) {

        float x = (2240F * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue) {
            /*
            op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(actualPower);
            fr.setPower(-(actualPower));
            bl.setPower(-(actualPower));
            br.setPower(actualPower);
        }

        StopAll();
    }
    public void Drive(float power, float distance, Direction d) {

        float x = (1120F * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }

        StopAll();

    }

    public void Turn(float power, int angle, Direction d, BNO055IMU imu, OpMode opMode) {
        int startAngle = ((Math.round(imu.getAngularOrientation().firstAngle))+180);

        int targetAngle = startAngle + angle;

        int currentAngle = startAngle;

        float actualPower = power;
        if (d == Direction.CLOCKWISE) {
            actualPower = -(power);
        }
            while (currentAngle < targetAngle) {

                opMode.telemetry.addData("start:", startAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.addData("actual:", (currentAngle-180));
                opMode.telemetry.update();

                currentAngle = ((Math.round(imu.getAngularOrientation().firstAngle))+180);
                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }

            StopAll();

    }

    public void DriveToImage(float power, VuforiaTrackable imageTarget, OpMode opMode) {
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();

        float actualPower = power;

        if (pos != null) {
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            opMode.telemetry.addData("z distance:", d);

            while (Math.abs(d) >= 100) {
                pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
                d = pos.getColumn(3).get(2);
                opMode.telemetry.addData("z distance:", d);

                fl.setPower(actualPower);
                fr.setPower(-(actualPower));
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }
        opMode.telemetry.update();
        StopAll();
    }

    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, BNO055IMU imu) {
        OpenGLMatrix pos = ((VuforiaTrackableDefaultListener)imageTarget.getListener()).getPose();
        float turningVelocity = Math.abs(imu.getAngularVelocity().xRotationRate);

        while (pos == null) {
            
        }
    }

    public void StopAll(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

}