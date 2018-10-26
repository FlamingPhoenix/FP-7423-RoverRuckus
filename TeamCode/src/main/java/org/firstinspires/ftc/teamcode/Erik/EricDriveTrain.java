package org.firstinspires.ftc.teamcode.Erik;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveTrain;

public class EricDriveTrain extends DriveTrain {

    public EricDriveTrain(DcMotor frontleft, DcMotor frontright, DcMotor backleft, DcMotor backright)  {
        super(frontleft, frontright, backleft, backright);
    }

    @Override
    public void TurnToImage(float initialPower, Direction d, VuforiaTrackable imageTarget, BNO055IMU imu) {
        super.TurnToImage(initialPower, d, imageTarget, imu);
    }
}
