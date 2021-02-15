// package defined
package org.firstinspires.ftc.teamcode;

// import classes
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// teleop is defined
@TeleOp(name = "TeleOp", group = "Final")

// builds on LinearOpMode
public class FinalTeleOp extends LinearOpMode {

    // test boolean for toggle
    boolean changed = false;

    // teleop code
    public void runOpMode() {

        // defines hardware
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

        // reset encoders - stop
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
        m7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m8.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set arm behavior when no power
        m7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // start telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // opmode code
        while (opModeIsActive()){

            // driving code
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            boolean slow = false;
            if ( (gamepad1.left_stick_button) || (gamepad1.right_stick_button) ) {
                slow = true;
            }
            if (slow == true) {
                m1.setPower( 0.5 * (y - x + rx) );
                m2.setPower( 0.5 * (y + x + rx) );
                m3.setPower( 0.5 * (y - x - rx) );
                m4.setPower( 0.5 * (y + x - rx) );
            } else {
                m1.setPower(y - x + rx);
                m2.setPower(y + x + rx);
                m3.setPower(y - x - rx);
                m4.setPower(y + x - rx);
            }

            // code for shooter and gate for high goal
            if (gamepad1.right_bumper) {
                m9.setPosition(0.62);
                m5.setPower(gamepad1.right_trigger * 0.9);
            } else {
                m9.setPosition(0);
                m5.setPower(0);
            }

            // code for shooter and gate for power shots
            if (gamepad1.left_bumper) {
                m9.setPosition(0.62);
                m5.setPower(gamepad1.right_trigger * 0.67);
            } else {
                m9.setPosition(0);
                m5.setPower(0);
            }

            // code for intake and belt
            if (gamepad1.right_trigger >= 0.1) {
                m6.setPower(-gamepad1.right_trigger);
                m8.setPower(1);
            } else {
                m6.setPower(0);
                m8.setPower(0);
            }

            // code for intake
            if (gamepad1.left_trigger >= 0.1) {
                m6.setPower(gamepad1.right_trigger);
            } else {
                m6.setPower(0);
            }

            // arm to out position
            if (gamepad1.b) {
                m7.setTargetPosition(1418);
                m7.setPower(0.3);
            }

            // arm to start position
            if (gamepad1.x) {
                m7.setTargetPosition(0);
                m7.setPower(0.3);
            }

            // toggle shooter test
            if(gamepad1.a && !changed) {
                if(m5.getPower() == 0) m5.setPower(1);
                else m5.setPower(0);
                changed = true;
            } else if(!gamepad1.right_bumper) changed = false;

            // other telemetry
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(), m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.addData("arm: ", m7.getCurrentPosition());
            telemetry.update();

        }

        // set motor power to 0 when finished
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
