package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "AUTOTEST", group = "Test")

public class autotest extends  LinearOpMode {

    DcMotor m1, m2, m3, m4;
    BNO055IMU imu;
    DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    ColorSensor colorSensor;

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        //commands to run in auto starts

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 2200) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, 0.5);
        while (m1.getCurrentPosition() > 130) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 2400) {
        }
        setPower(0, 0, 0);

        sleep(2000);

        setPower(0, -1, 0);
        while (m1.getCurrentPosition() > 130) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, -0.5);
        while (m1.getCurrentPosition() < 2200) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 10500) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, 0.5);
        while (m1.getCurrentPosition() > 8470) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 10750) {
        }
        setPower(0, 0, 0);

        sleep(2000);

        setPower(0, -1, 0);
        while (m1.getCurrentPosition() > 8470) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, 0.5);
        while (m1.getCurrentPosition() > 6380) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 17000) {
        }
        setPower(0, 0, 0);

        //commands to run in auto ends

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    }

    void setPower(double px, double py, double pa) {
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }

    void runEnc() {
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetEnc() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
