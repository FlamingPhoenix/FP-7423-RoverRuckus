package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
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
    int magZero = 0;

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
        ServoControllerEx primaryController = (ServoControllerEx) hook.getController();
        int hookServoPort = hook.getPortNumber();
        PwmControl.PwmRange hookPwmRange = new PwmControl.PwmRange(899, 2000);
        primaryController.setServoPwmRange(hookServoPort, hookPwmRange);

        liftSensor = hardwareMap.get(DigitalChannel.class, "liftsensor");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive(gamepad1.left_stick_x, gamepad1.left_stick_y * -1, gamepad1.right_stick_x);
        //telemetry.addData("y1;", gamepad1.left_stick_y);
        //telemetry.addData("x1;", gamepad1.left_stick_x);
        //telemetry.update();

        float power = gamepad2.right_stick_y;


        if (liftSensor.getState() == false)
        {
            magZero = rightLift.getCurrentPosition();
        }


        if (power < -0.2)
        {
            if ((rightLift.getCurrentPosition() - magZero) < -7000)
            {
                if (power < -0.5)
                    power = -0.5f;
            }

            leftLift.setPower(power);
            rightLift.setPower(power);
        }
        else if (power > 0.2)
        {
            if ((rightLift.getCurrentPosition() - magZero) > 2500)
            {
                if (power > 0.5)
                    power = 0.5f;
            }

            leftLift.setPower(power);
            rightLift.setPower(power);
        }
        else
        {
            rightLift.setPower(0);
            leftLift.setPower(0);
        }


        // servo motors
        if (gamepad2.left_bumper)
        {
            if (isHookOpen)
            {
                hook.setPosition(1);
                isHookOpen = false;
            }
            else
            {
                hook.setPosition(.1);
                isHookOpen = true;
            }
        }

        telemetry.addData("lift sensor:", liftSensor.getState());
        telemetry.addData("encoder value: ", rightLift.getCurrentPosition());
        telemetry.addData("power: ", power);
        telemetry.addData("magZero: ", magZero);
        telemetry.update(); //graffiti
    }
}
