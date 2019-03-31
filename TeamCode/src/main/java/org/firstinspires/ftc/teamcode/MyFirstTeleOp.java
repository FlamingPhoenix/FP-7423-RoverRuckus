package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "MyFirstTele", group = "none")
public class MyFirstTeleOp extends OpMode {
    DcMotor frontLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;

    DriveTrain driveTrain;
    ///Initializing variables (motors, servos, sensors, and other common variables.
    ///Remark: Initialization takes place when user hit "Initialize" on Driver Station
    @Override
    public void init() {
        frontLeftWheel = hardwareMap.dcMotor.get("frontleft");
        frontRightWheel = hardwareMap.dcMotor.get("frontright");
        backLeftWheel = hardwareMap.dcMotor.get("backleft");
        backRightWheel = hardwareMap.dcMotor.get("backright");

        frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    @Override
    public void loop() {
        drive(gamepad1.left_stick_x * -1, gamepad1.left_stick_y, gamepad1.right_stick_x);

    }
    public void drive(float x1, float y1, float x2) {
        float frontLeft = y1 + x1 + x2;
        float frontRight = y1 - x1 - x2;
        float backLeft = y1 - x1 + x2;
        float backRight = y1 + x1 - x2;

        frontLeft = Range.clip(frontLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        backLeft = Range.clip(backLeft, -1, 1);
        backRight = Range.clip(backRight, -1, 1);

        frontLeftWheel.setPower(frontLeft);
        frontRightWheel.setPower(frontRight);
        backLeftWheel.setPower(backLeft);
        backRightWheel.setPower(backRight);
    }
}
