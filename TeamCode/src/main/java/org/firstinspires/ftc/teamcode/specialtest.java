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
        Servo m7 =  hardwareMap.servo.get("arm1");
        Servo m8 =  hardwareMap.servo.get("arm2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()){

            m7.setPosition(0);
            m8.setPosition(0);

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            m2.setPower(y + x + rx);
            m1.setPower(y - x + rx);
            m3.setPower(y - x - rx);
            m4.setPower(y + x - rx);


            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(), m3.getCurrentPosition(), m4.getCurrentPosition());

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


            if(gamepad1.y) {
                // move to 0 degrees.
                m7.setPosition(0);
                m8.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                m7.setPosition(0.5);
                m8.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                m7.setPosition (1);
                m8.setPosition (1);
            }
            telemetry.addData("Servo Position", m7.getPosition());
            telemetry.addData("Servo Position", m8.getPosition());
            telemetry.update();

        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);
        m6.setPower(0);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
