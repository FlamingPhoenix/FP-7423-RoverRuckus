package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
    boolean isAPressed = false;
    boolean isLatchenabled = true;
    float x1, x2, y1;
    DcMotor intakeMotor;


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
        ServoControllerEx hookController = (ServoControllerEx) hook.getController();
        int hookServoPort = hook.getPortNumber();
        PwmControl.PwmRange hookPwmRange = new PwmControl.PwmRange(899, 2000);
        hookController.setServoPwmRange(hookServoPort, hookPwmRange);

        liftSensor = hardwareMap.get(DigitalChannel.class, "liftsensor");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor = hardwareMap.dcMotor.get("intaketh");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        x1 = gamepad1.left_stick_x;
        y1 = gamepad1.left_stick_y;
        x2 = gamepad1.right_stick_x;
        if (gamepad1.left_bumper){
            x1 = x1/2f;
            y1 = y1/2f;
            x2 = x2/2f;
        }
        drive(x1,  y1 * -1, x2);
        //telemetry.addData("y1;", gamepad1.left_stick_y);
        //telemetry.addData("x1;", gamepad1.left_stick_x);
        //telemetry.update();

        float power = gamepad2.right_stick_y;

        telemetry.addData("lift power: ", power);
        telemetry.addData("lift encoder: ", rightLift.getCurrentPosition());
        telemetry.update();
        Log.i("lift power: ", Float.toString(power));
        Log.i("lift encoder: ", Integer.toString(rightLift.getCurrentPosition()));


        if (!liftSensor.getState())
        {
            magZero = rightLift.getCurrentPosition();
        }

        //driver 2 control lift
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
            if ((rightLift.getCurrentPosition() - magZero) > 2500)// why positive 2500 ? while above is -7000 ?
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
        if ((gamepad2.left_bumper) && isLatchenabled)
        {
            isLatchenabled = false;
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
        else if (!gamepad2.left_bumper)
            isLatchenabled = true;

        if (gamepad2.a && liftSensor.getState())
        {
            if(rightLift.getCurrentPosition() < (magZero + 200))
            {
                rightLift.setPower(0.4);
                leftLift.setPower(0.4);
            }
            else if ((rightLift.getCurrentPosition() > (magZero - 200)))
            {
                rightLift.setPower(-0.4);
                leftLift.setPower(-0.4);
            }
        }

        if (gamepad1.right_trigger > 0.2 && gamepad1.right_trigger < 0.8) {
            if (intakeMotor.getCurrentPosition() > -800) {
                intakeMotor.setPower(-0.5);
            } else {
                intakeMotor.setPower(-0.1);
            }
        }
        else if (gamepad1.right_trigger > 0.8) {
            if (intakeMotor.getCurrentPosition() > -800) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(-0.1);
            }
        }
        else if (gamepad1.right_bumper) {
            if (intakeMotor.getCurrentPosition() > 0)
                intakeMotor.setPower(0);
            else
                intakeMotor.setPower(0.4);
        } else {
            intakeMotor.setPower(0);
        }

        telemetry.addData("encoder: ", intakeMotor.getCurrentPosition());
        telemetry.update(); //graffiti
    }
}
