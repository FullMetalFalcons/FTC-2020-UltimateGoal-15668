package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 */

@TeleOp(name = "mechbot demo", group = "Mechanum")
@Disabled
public class MechBotDemo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo Arm = null;

    public void runOpMode() {
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor m5 = hardwareMap.dcMotor.get("extend");
        DcMotor m6 = hardwareMap.dcMotor.get("lift");
        Servo LClam = hardwareMap.servo.get("LClam");
        Servo Wrist = hardwareMap.servo.get("Wrist");
        Servo RClam = hardwareMap.servo.get("RClam");


        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LClam.setPosition(1);
        RClam.setPosition(0);
        Wrist.setPosition(.85);
        //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        //  Servo backServo = hardwareMap.servo.get("arm");
        /*DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance"); */
        //gyro.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);


        telemetry.addData("Press Start When Ready", "");
        telemetry.update();
        waitForStart();
        Wrist.setPosition(.4);
        while (opModeIsActive()) {
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

            boolean leftBump = gamepad1.left_bumper;
            boolean rightBump = gamepad1.right_bumper;
            double exPos = m5.getCurrentPosition();
            extender(leftBump, rightBump, exPos, m5);
//            if (leftBump == true) {
//                m5.setPower(0.2);
//            }
//            if (rightBump == true) {
//                m5.setPower(-0.2);
//            }
            double extendpos = m5.getCurrentPosition();
            telemetry.addData("Extend", extendpos);
            telemetry.update();

            float leftTrig = gamepad1.left_trigger;
            float rightTrig = gamepad1.right_trigger;
            int currentPosition = m6.getCurrentPosition();
            ProcessArm(leftTrig, rightTrig, currentPosition, m6);

            double LData = LClam.getPosition();
            double RData = RClam.getPosition();
            telemetry.addData("LClamp", LData);
            Grabber(LClam, RClam, Wrist);
//            if (gamepad1.a) {
//
//            }
//
//
//            if (gamepad1.dpad_up == true) {
//                m6.setTargetPosition(350);
//                m6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                m6.setPower(.6);
//            }


            // telemetry.addData("Heading"," %.1f", gyro.getHeading());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("trigger data", leftTrig);
            telemetry.addData("right trigger", rightTrig);
            telemetry.addData("Current arm position", currentPosition);
//            telemetry.addData("Arm" , AData);
            telemetry.update();

            telemetry.addData("Status", "Initialized");
            telemetry.update();


        }


    }

    private void ProcessArm(double ltrig, double rtrig, double pos, DcMotor m) {
        if (ltrig == 0 && rtrig == 0) {
            m.setPower(0);
        } else if (ltrig > 0) {
            if (pos > 120) {
                m.setPower(-.3 * ltrig);
            } else {
                m.setPower(0);
            }
        } else {
            if (rtrig > 0) {
                if (pos < 1200) {
                    m.setPower(.3 * rtrig);
                } else {
                    m.setPower(0);
                }
            }
//jamie
        }
    }

    private void HoldArmPosition() {

    }

    private void extender(boolean lBump, boolean rBump, double exPos, DcMotor backforth) {
        if (lBump == false && rBump == false) {
            backforth.setPower(0);
        } else if (lBump == true) {
            if (exPos < 10800) {
                backforth.setPower(.2);
            } else {
                backforth.setPower(0);
            }
        } else if (rBump == true) {
            if (exPos > 0) {
                backforth.setPower(-.2);
            }
        } else {
            backforth.setPower(0);
        }

    }
    private void Grabber (Servo LeftC, Servo RightC, Servo Wrist) {
        if (gamepad1.a) {
            LeftC.setPosition(.25);
            RightC.setPosition(.75);
        } else if (gamepad1.b)
        {
            LeftC.setPosition(.75);
            RightC.setPosition(.25);
        }
    }

//    private void grab(Servo left, Servo right, Servo wrist) {
//        if (gamepad1.y);
//    }

}