package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorWrapper {

    private DcMotor motor;
    private double previousPower;



    public  DcMotorWrapper(DcMotor m) {
        motor = m;
    }

    public void setPower(double power) {
        if (previousPower != power) {
            motor.setPower(power);
            previousPower = power;
        }
    }
}
