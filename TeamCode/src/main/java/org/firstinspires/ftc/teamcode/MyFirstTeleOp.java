package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "MyFirstTele", group = "none")
public class MyFirstTeleOp extends OpMode {
    DcMotor frontLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;

    ///Initializing variables (motors, servos, sensors, and other common variables.
    ///Remark: Initialization takes plance when user hit "Initialize" on Driver Station
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
        frontLeftWheel.setPower(gamepad1.left_stick_y);
        backLeftWheel.setPower(gamepad1.left_stick_y);
        frontRightWheel.setPower(gamepad1.right_stick_y);
        backRightWheel.setPower(gamepad1.right_stick_y);
    }
}


