package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

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

    public void Drive(float power, float distance, Direction d) {

        float x = (1120F * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int currentPosition = 0;

        while (currentPosition < targetEncoderValue) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

}
