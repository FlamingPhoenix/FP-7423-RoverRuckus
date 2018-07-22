package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by Steve on 7/22/2018.
 */

@Autonomous(name="My First Auto", group="none")
public class MyFirstAuto extends LinearOpMode {
    DcMotor frontLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;

    public void initialize() {
        frontLeftWheel = hardwareMap.dcMotor.get("frontleft");
        frontRightWheel = hardwareMap.dcMotor.get("frontright");
        backLeftWheel = hardwareMap.dcMotor.get("backleft");
        backRightWheel = hardwareMap.dcMotor.get("backright");

        frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        int encoderCount;
        encoderCount = (int) (1440D * 10D / (4 * Math.PI));

        backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int currentCount = backLeftWheel.getCurrentPosition();

        while (currentCount < encoderCount) {
            frontLeftWheel.setPower(1);
            frontRightWheel.setPower(1);
            backLeftWheel.setPower(1);
            backRightWheel.setPower(1);
            currentCount = backLeftWheel.getCurrentPosition();
        }

        frontLeftWheel.setPower(0);
        frontRightWheel.setPower(0);
        backLeftWheel.setPower(0);
        backRightWheel.setPower(0);

    }
}
