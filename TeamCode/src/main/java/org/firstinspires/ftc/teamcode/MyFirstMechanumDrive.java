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
    Servo arm;
    boolean isArmInitiliazed = false;
    Servo hopper;
    Servo bucket;
    boolean isAPressed = false;
    boolean isReadyToDropMineral = false;


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

        arm = hardwareMap.servo.get("arm");
        ServoControllerEx armController = (ServoControllerEx) arm.getController();
        int armServoPort = arm.getPortNumber();
        PwmControl.PwmRange armPwmRange = new PwmControl.PwmRange(1480, 1700);
        armController.setServoPwmRange(armServoPort, armPwmRange);

        hook = hardwareMap.servo.get("hook");
        ServoControllerEx hookController = (ServoControllerEx) hook.getController();
        int hookServoPort = hook.getPortNumber();
        PwmControl.PwmRange hookPwmRange = new PwmControl.PwmRange(899, 2000);
        hookController.setServoPwmRange(hookServoPort, hookPwmRange);

        hopper = hardwareMap.servo.get("hopper");
        ServoControllerEx hopperController = (ServoControllerEx) hopper.getController();
        int hopperServoPort = hopper.getPortNumber();
        PwmControl.PwmRange hopperPwmRange = new PwmControl.PwmRange(899, 2105);
        hopperController.setServoPwmRange(hopperServoPort, hopperPwmRange);
        hopper.setPosition(0.2);

        bucket = hardwareMap.servo.get("bucket");
        ServoControllerEx bucketController = (ServoControllerEx) bucket.getController();
        int bucketServoPort = bucket.getPortNumber();
        PwmControl.PwmRange bucketPwmRange = new PwmControl.PwmRange(899, 2105);
        bucketController.setServoPwmRange(bucketServoPort, hopperPwmRange);
        bucket.setPosition(1);

        liftSensor = hardwareMap.get(DigitalChannel.class, "liftsensor");
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {


        if (!isArmInitiliazed)
        {
            arm.setPosition(0.3);
            isArmInitiliazed = true;
        }

        drive(gamepad1.left_stick_x,  gamepad1.left_stick_y * -1, gamepad1.right_stick_x);
        //telemetry.addData("y1;", gamepad1.left_stick_y);
        //telemetry.addData("x1;", gamepad1.left_stick_x);
        //telemetry.update();

        float power = gamepad2.right_stick_y;


        if (liftSensor.getState() == false)
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

            //prevent arm to get in harms way

                if (arm.getPosition() < 0.3d) {
                    arm.setPosition(0.3d);
                }

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

        if (gamepad2.a && liftSensor.getState() == true)
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

        //driver 2 control buckets
        if (gamepad2.right_trigger > 0.5) {
            bucket.setPosition(0);
        }

        else if (gamepad2.right_bumper) {
            bucket.setPosition(1);
        }

        //driver 1 control arm position
        if (gamepad1.right_trigger > 0.5)
        {
            arm.setPosition(1);
            isReadyToDropMineral = false;
        }
        else if (gamepad1.right_bumper)
        {
            arm.setPosition(0.7);
            isReadyToDropMineral = false;
        }
        else if (gamepad1.y)
        {
            if (rightLift.getCurrentPosition() > magZero) {
                double newArmPosition = arm.getPosition() - 0.05d;
                if (newArmPosition < 0.05)
                    newArmPosition = 0.05d;
                arm.setPosition(newArmPosition);
                isReadyToDropMineral = true;
            }
        }


        if (gamepad1.a)
        {
            hopper.setPosition(0.1);
            isAPressed = true;
        }
        else if (!isAPressed)
        {
            double armAngle = arm.getPosition();

            telemetry.addData("arm position", armAngle);
            double hopperPosition = 0.13d + (120.0d * (1d - armAngle)) * 0.0078d;
            if (hopperPosition > 1.0d) {
                hopperPosition = 1.0d;
            }
            hopper.setPosition(hopperPosition);
        }
        else if (isAPressed)
        {
            if (arm.getPosition() > 0.5)
            {
                isAPressed = false;
            }
        }

        telemetry.update(); //graffiti
    }
}
