// package defined
package org.firstinspires.ftc.teamcode;

// import classes
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

// auto is defined
@Autonomous(name = "AUTOTEST", group = "Test")

// builds on LinearOpMode
public class FinalAuto extends  LinearOpMode {

    // hardware is set
    DcMotor m1, m2, m3, m4, m5, m6, m7, m8;
    Servo m9;

    public void runOpMode(){

        // hardware is defined
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        DcMotorEx m5 = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        m6 = hardwareMap.dcMotor.get("intake");
        m7 = hardwareMap.dcMotor.get("arm");
        m8 = hardwareMap.dcMotor.get("belt");
        m9 = hardwareMap.servo.get("gate");

        //  reset encoders - stop
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset encoders - start
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m7.setTargetPosition(0);
        m7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m8.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //commands to run in auto starts

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 2200) {
        }
        setPower(0, 0, 0);

        //commands to run in auto ends

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setVelocity(0);
        m6.setPower(0);
        m7.setTargetPosition(0);
        m8.setPower(0);

    }

    void setPower(double x, double y, double rx) {
        m1.setPower(y - x + rx);
        m2.setPower(y + x + rx);
        m3.setPower(y - x - rx);
        m4.setPower(y + x - rx);
    }



}
