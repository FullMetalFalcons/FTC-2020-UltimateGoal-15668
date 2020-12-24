package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name = "Mech Test Code 2", group = "Test")

public class specialtest extends LinearOpMode {

    public void runOpMode() {

        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor m5 = hardwareMap.dcMotor.get("shooter");


        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            double px = -gamepad1.left_stick_x;
            double py = gamepad1.left_stick_y;
            double pa = gamepad1.right_stick_x;
            boolean halfp;
            double m1p;
            double m2p;
            double m3p;
            double m4p;
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

            halfp = true;

            if (halfp == true) {
                m1p = p1 * 0.5;
                m2p = p2 * 0.5;
                m3p = p3 * 0.5;
                m4p = p4 * 0.5;
            } else {
                m1p = p1;
                m2p = p2;
                m3p = p3;
                m4p = p4;
            }

            m1.setPower(m1p);
            m2.setPower(m2p);
            m3.setPower(m3p);
            m4.setPower(m4p);

            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(), m3.getCurrentPosition(), m4.getCurrentPosition());

            if (gamepad1.left_trigger >= 0.1) {
                m5.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1. right_trigger >= 0.1) {
                m5.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.a == true) {
                m5.setPower(0);
            }

        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);

    }

}
