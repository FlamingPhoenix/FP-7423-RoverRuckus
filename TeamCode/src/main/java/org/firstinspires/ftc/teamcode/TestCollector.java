package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "TestCollector", group = "none")
public class TestCollector extends OpMode {

    DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.dcMotor.get("intaketh");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.right_stick_y < -0.5)
            intakeMotor.setPower(0.2);

        else if (gamepad1.right_stick_y > 0.5)
            intakeMotor.setPower(-0.2);

        else
            intakeMotor.setPower(0);

        telemetry.addData("encoder:", intakeMotor.getCurrentPosition());
        telemetry.update();
    }
}
