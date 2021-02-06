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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name = "Mech Test Code", group = "Test")

public class specialtest extends LinearOpMode {

    public void runOpMode() {

        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        DcMotor m5 = hardwareMap.dcMotor.get("shooter");
        DcMotor m6 = hardwareMap.dcMotor.get("intake");
        DcMotor m7 = hardwareMap.dcMotor.get("arm");
        DcMotor m8 = hardwareMap.dcMotor.get("belt");
        Servo m9 = hardwareMap.servo.get("gate");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            m1.setPower(y - x + rx);
            m2.setPower(y + x + rx);
            m3.setPower(y - x - rx);
            m4.setPower(y + x - rx);

            if (gamepad1.left_trigger >= 0.1) {
                m5.setPower(-gamepad1.left_trigger);
            } else if (gamepad1. right_trigger >= 0.1) {
                m5.setPower(gamepad1.right_trigger);
            } else {
                m5.setPower(0);
            }

            if (gamepad1.left_bumper == true) {
                m6.setPower(-1);
            } else if (gamepad1.right_bumper == true) {
                m6.setPower(1);
            } else {
                m6.setPower(0);
            }

            double minarm = 0;
            double maxarm = 999999999;
            if ((gamepad1.y == true) && (m7.getCurrentPosition() <= maxarm)) {
                m7.setPower(0.3);
            } else if ((gamepad1.b == true) && (m7.getCurrentPosition() >= minarm)) {
                m7.setPower(-0.3);
            } else {
                m7.setPower(0);
            }

            if (gamepad1.x == true) {
                m8.setPower(1);
            } else if (gamepad1.a == true) {
                m8.setPower(-1);
            } else {
                m8.setPower(0);
            }

            if (gamepad1.dpad_up) {
                m9.setPosition(0);
            } else if (gamepad1.dpad_right) {
                m9.setPosition(0.62);
            }

            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(), m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.addData("arm: ", m7.getCurrentPosition());
            telemetry.update();

        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);
        m6.setPower(0);
        m7.setPower(0);
        m8.setPower(0);

    }

}
