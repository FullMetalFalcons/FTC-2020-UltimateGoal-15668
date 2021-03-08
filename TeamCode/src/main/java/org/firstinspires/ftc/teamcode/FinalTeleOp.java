// package defined
package org.firstinspires.ftc.teamcode;

// import classes
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// teleop is defined
@TeleOp(name = "TeleOp", group = "Final")

// builds on LinearOpMode
public class FinalTeleOp extends LinearOpMode {

    // defines hardware
    DcMotor m1, m2, m3, m4, m6, m7, m8;
    DcMotorEx m5;
    Servo m9, m10;

    // teleop code
    public void runOpMode() {

        // defines hardware
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
        m10 = hardwareMap.servo.get("agate");

        // reset encoders - stop
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        // set arm behavior when no power
        m7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // sets up variables for all toggles
        boolean halfPower = false;
        boolean stickPressed = false;
        boolean shooterStarted = false;
        boolean rbPressed = false;
        boolean shooterStartedLow = false;
        boolean lbPressed = false;
        boolean armOut = false;
        boolean yPressed = false;
        boolean armIn = false;
        boolean bPressed = false;
        boolean gateOpen = false;
        boolean aPressed = false;
        boolean armDrop = false;
        boolean xPressed = false;

        // start telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // opmode code
        while (opModeIsActive()){

            // code for driving and toggling slow mode
            double totalpowervalue = 1;
            if (!stickPressed && ( (gamepad1.left_stick_button) || (gamepad1.right_stick_button) ) ) {
                halfPower = !halfPower;
                stickPressed = true;
            } else if (stickPressed && !( (gamepad1.left_stick_button) || (gamepad1.right_stick_button) ) ) {
                stickPressed = false;
            }
            if (halfPower == true) {
                totalpowervalue = 0.25;
            } else if (halfPower == false) {
                totalpowervalue = 1;
            }
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            m1.setPower(totalpowervalue * (y - x + rx));
            m2.setPower(totalpowervalue * (y + x + rx));
            m3.setPower(totalpowervalue * (y - x - rx));
            m4.setPower(totalpowervalue * (y + x - rx));

            // code for toggling shooter and opening gate
            if (!rbPressed && gamepad1.right_bumper) {
                shooterStarted = !shooterStarted;
                rbPressed = true;
            } else if (rbPressed && !gamepad1.right_bumper) {
                rbPressed = false;
            }
            if (shooterStarted == true) {
                shooterStartedLow = false;
                double highpower = 1500;
                m5.setVelocity(highpower);
                if (m5.getVelocity() >= highpower - 10) {
                    m9.setPosition(0);
                }
            }
            if (!lbPressed && gamepad1.left_bumper) {
                shooterStartedLow = !shooterStartedLow;
                lbPressed = true;
            } else if (lbPressed && !gamepad1.left_bumper) {
                lbPressed = false;
            }
            if (shooterStartedLow == true) {
                shooterStarted = false;
                double lowpower = 1300;
                m5.setVelocity(lowpower);
                if (m5.getVelocity() >= lowpower - 10) {
                    m9.setPosition(0);
                }
            }
            if (shooterStartedLow == false && shooterStarted == false) {
                m5.setVelocity(0);
                m9.setPosition(0.62);
            }

            // code for intake and belt
            if (gamepad1.right_trigger > 0.1) {
                m6.setPower(gamepad1.right_trigger * -1);
                m8.setPower(-1);
            } else if (gamepad1.left_trigger > 0.1) {
                m6.setPower(gamepad1.left_trigger);
            } else {
                m6.setPower(0);
                m8.setPower(0);
            }

            // code for operating arm
            if (!yPressed && gamepad1.y) {
                armOut = !armOut;
                yPressed = true;
            } else if (yPressed && !gamepad1.y) {
                yPressed = false;
            }
            if (armOut == true) {
                armIn = false;
                armDrop = false;
                m7.setTargetPosition(150);
                m7.setPower(0.25);
            }
            if (!bPressed && gamepad1.b) {
                armIn = !armIn;
                bPressed = true;
            } else if (bPressed && !gamepad1.b) {
                bPressed = false;
            }
            if (armIn == true) {
                armOut = false;
                armDrop = false;
                m7.setTargetPosition(1050);
                m7.setPower(0.25);
            }
            if (!xPressed && gamepad1.x) {
                armDrop = !armDrop;
                xPressed = true;
            } else if (xPressed && !gamepad1.x) {
                xPressed = false;
            }
            if (armDrop == true) {
                armOut = false;
                armIn = false;
                m7.setTargetPosition(650);
                m7.setPower(0.25);
            }
            if (armOut == false && armIn == false && armDrop == false) {
                m7.setTargetPosition(0);
                m7.setPower(0.25);
            }

            // code for operating gate arm
            if (!aPressed && gamepad1.a) {
                gateOpen = !gateOpen;
                aPressed = true;
            } else if (aPressed && !gamepad1.a) {
                aPressed = false;
            }
            if (gateOpen == true) {
                m10.setPosition(-1);
            } else if (gateOpen == false) {
                m10.setPosition(1);
            }

            // code for belt outtake
            if (gamepad1.dpad_left == true) {
                m8.setPower(1);
            }

            // telemetry data adding and updating
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(), m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.addData("shooter ", m5.getVelocity());
            telemetry.addData("gate: ", m9.getPosition());
            telemetry.addData("intake: ", m6.getPower());
            telemetry.addData("belt: ", m8.getPower());
            telemetry.addData("arm: ", m7.getCurrentPosition());
            telemetry.addData("arm gate:", m10.getPosition());
            telemetry.update();

        }

        // set motor power to 0 when finished
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setVelocity(0);
        m6.setPower(0);
        m7.setPower(0);
        m8.setPower(0);

    }

}


