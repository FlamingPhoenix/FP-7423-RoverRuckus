package org.firstinspires.ftc.teamcode.MyClass;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Adjustin", group = "none")
public class Adjustin extends OpMode{
    DcMotor rightLift;
    DcMotor leftLift;
    Servo hook;
    boolean isHookOpen = false;
    DigitalChannel liftSensor;

    @Override
    public void init()
    {
        rightLift = hardwareMap.dcMotor.get("rightlift");
        leftLift = hardwareMap.dcMotor.get("leftlift");
        hook = hardwareMap.servo.get("hook");
        liftSensor = hardwareMap.get(DigitalChannel.class, "liftsensor");
    }

    @Override
    public void loop()
    {
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
