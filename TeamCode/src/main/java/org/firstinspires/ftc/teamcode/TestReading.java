package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 */
@TeleOp(name = "test", group = "Mechanum")
public class TestReading extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

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
//        Arm = hardwareMap.get(Servo.class, "Arm");
//        Arm.setPosition(0.5);
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

        Wrist.setPosition(.4);


        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setTargetPosition(100);
        m6.setPower(0.2);
        waitForStart();

    while (opModeIsActive()) {
        double LPosition = LClam.getPosition();
        double RPosition = RClam.getPosition();

            if (gamepad1.a) {
                LClam.setPosition(.25);
                RClam.setPosition(.75);
            } else if (gamepad1.b)
            {
                LClam.setPosition(.75);
                RClam.setPosition(.25);
            }


        m6.setPower(0);





        telemetry.addData("left clamp", LPosition);
        telemetry.addData("right clamp", RPosition);
        telemetry.addData("Status", "Initialized");
            telemetry.update();







        }


    }


}