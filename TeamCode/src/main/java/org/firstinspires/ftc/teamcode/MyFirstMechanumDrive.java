package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mechanum", group = "none")
public class MyFirstMechanumDrive extends OpMode {

    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    DcMotor rightLift;
    DcMotor leftLift;
    Servo hook;
    boolean isHookOpen = false;
    DigitalChannel liftSensor;

    public void drive(float x1, float y1, float x2) {
        float frontLeft = y1 + x1 + x2;
        float frontRight = y1 - x1 - x2;
        float backLeft = y1 - x1 + x2;
        float backRight = y1 + x1 - x2;

        frontLeft = Range.clip(frontLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        backLeft = Range.clip(backLeft, -1, 1);
        backRight = Range.clip(backRight, -1, 1);

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("frontright");
        fl = hardwareMap.dcMotor.get("frontleft");
        br = hardwareMap.dcMotor.get("backright");
        bl = hardwareMap.dcMotor.get("backleft");
        rightLift = hardwareMap.dcMotor.get("rightlift");
        leftLift = hardwareMap.dcMotor.get("leftlift");
        hook = hardwareMap.servo.get("hook");
        liftSensor = hardwareMap.get(DigitalChannel.class, "liftsensor");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y * -1, gamepad1.right_stick_x);
        telemetry.addData("y1;", gamepad1.left_stick_y);
        telemetry.addData("x1;", gamepad1.left_stick_x);
        telemetry.update();

        float power = gamepad1.right_stick_y;

        // lift motors
        if (gamepad1.right_stick_y < -0.2)
        {
            rightLift.setPower(power);
            leftLift.setPower(power);
        }
        else if (gamepad1.right_stick_y > 0.2)
        {
            rightLift.setPower(power);
            leftLift.setPower(power);
        }
        else
        {
            rightLift.setPower(0);
            leftLift.setPower(0);
        }

        // servo motors
        if (gamepad1.left_bumper)
        {
            if (isHookOpen)
            {
                hook.setPosition(1);
                isHookOpen = false;
            }
            else
            {
                hook.setPosition(.5);
                isHookOpen = true;
            }
        }

        telemetry.addData("lift sensor:", liftSensor.getState());
        telemetry.update(); //graffiti
    }
}
