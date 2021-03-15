package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

@Autonomous(name = "AUTOTEST", group = "Test")

public class FinalAuto extends  LinearOpMode {

    DcMotor m1, m2, m3, m4, m6, m7, m8;
    DcMotorEx m5;
    Servo m9;
    BNO055IMU imu;

    private double[] distances;

    double startHeading = 0;
    boolean previouspress = false;
    boolean previouspressy = false;

    public void runOpMode() throws InterruptedException {

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m5 = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        m6 = hardwareMap.dcMotor.get("intake");
        m7 = hardwareMap.dcMotor.get("arm");
        m8 = hardwareMap.dcMotor.get("belt");
        m9 = hardwareMap.servo.get("gate");

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m7.setTargetPosition(0);
        m7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m8.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);
        Orientation orientation;

        m9.setPosition(0.22);

        waitForStart();

        //commands to run in auto starts

        driveToValueE(-0.3, -2100);


        shooterRun(1300,5000);


        //commands to run in auto ends

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);
        m6.setPower(0);
        m7.setPower(0);
        m8.setPower(0);

    }

    void gyroTurn (double degrees, double speed) {

    }

    void driveToValueE(double speed, int tick) {
        m1.setTargetPosition(tick);
        m2.setTargetPosition(tick);
        m3.setTargetPosition(tick);
        m4.setTargetPosition(tick);
        m1.setPower(speed);
        m2.setPower(speed);
        m3.setPower(speed);
        m4.setPower(speed);
        boolean notInPosition = true;
        do {
            int m1Pos = m1.getCurrentPosition();
            int m2Pos = m2.getCurrentPosition();
            int m3Pos = m3.getCurrentPosition();
            int m4Pos = m4.getCurrentPosition();
            int avgPos = (m1Pos + m2Pos + m3Pos + m4Pos)/4;
            notInPosition = Math.abs(tick-avgPos) > 30;
            sleep(20);
        } while (notInPosition);
        sleep(100);
        resetEncValuesEnc();
    }

    void driveToValueT(double speed, long time) {
        m1.setPower(speed);
        m2.setPower(speed);
        m3.setPower(speed);
        m4.setPower(speed);
        sleep(time);
    }

    void driveToValueO(double x, double y, double rx, double position) {
        while (abs(m1.getCurrentPosition()) < abs(position)) {
            setPower(x,y,rx);
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(100);
        resetEncValuesEnc();
    }

    void shooterRun (double rpm, long time) {
        while (m5.getVelocity() <= rpm - 10) {
            m5.setVelocity(rpm);
            m9.setPosition(0.22);
        }
        while (m5.getVelocity() >= rpm - 10) {
            m9.setPosition(0.05);
        }
        sleep(200);
        m8.setPower(-0.8);
        m5.setVelocity(rpm);
        sleep(time);
        m9.setPosition(0.9);
        sleep(200);
    }

    void resetEncValuesEnc() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetEncValuesPos() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m1.setTargetPosition(0);
        m2.setTargetPosition(0);
        m3.setTargetPosition(0);
        m4.setTargetPosition(0);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setPower(double x, double y, double rx) {
        m1.setPower(y - x + rx);
        m2.setPower(y + x + rx);
        m3.setPower(y - x - rx);
        m4.setPower(y + x - rx);
    }






}
