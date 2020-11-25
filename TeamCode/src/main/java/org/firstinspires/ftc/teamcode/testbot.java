package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name = "test bot", group = "Mechanum")
public class testbot extends LinearOpMode {

    public void runOpMode() {
        Gyroscope imu = hardwareMap.get(Gyroscope.class, "imu");
        DigitalChannel digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        DistanceSensor sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor m5 = hardwareMap.dcMotor.get("5");
        DcMotor m6 = hardwareMap.dcMotor.get("6");
        DcMotor m7 = hardwareMap.dcMotor.get("7");
        DcMotor m8 = hardwareMap.dcMotor.get("8");
        DcMotor m9 = hardwareMap.dcMotor.get("9");
        DcMotor m10 = hardwareMap.dcMotor.get("10");
        DcMotor m11 = hardwareMap.dcMotor.get("11");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Status", "Running");
            telemetry.update();

            double px = gamepad1.left_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            double py = -gamepad1.left_stick_y;
            if (Math.abs(py) < 0.05) py = 0;
            double pa = -gamepad1.right_stick_x;
            if (Math.abs(pa) < 0.05) pa = 0;
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

            if (gamepad1.a == true) {
                m5.setPower(100);
                m6.setPower(100);
                m7.setPower(100);
                m8.setPower(100);
                m9.setPower(100);
                m10.setPower(100);
                m11.setPower(100);
            }

            if (gamepad1.b == true) {
                m5.setPower(-100);
                m6.setPower(-100);
                m7.setPower(-100);
                m8.setPower(-100);
                m9.setPower(-100);
                m10.setPower(-100);
                m11.setPower(-100);
            }

        }

    }

}