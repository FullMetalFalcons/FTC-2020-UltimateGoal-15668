package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 */
@TeleOp(name = "ArmMotor", group = "test")
@Disabled
public class TestArmMotor extends LinearOpMode {

    //variable
    private ElapsedTime runtime = new ElapsedTime();
    //running state
    public void runOpMode() {

        robot sam = new robot();
        sam.RobotInit(hardwareMap);

        //telemetry
        telemetry.addData("Press Start When Ready", "");
        telemetry.update();

        //while running
        waitForStart();

        sam.AdjustRisk();
        while (opModeIsActive()) {

            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;



            float ltrig = gamepad1.left_trigger;
            float rtrig = gamepad1.right_trigger;

            if (rtrig > 0) sam.LiftArm();
            if (ltrig > 0) sam.DropArm();

            boolean lbump = gamepad1.left_bumper;
            boolean rbump = gamepad1.right_bumper;

            sam.RobotDriveFieldRelative(telemetry, px, py, pa);
            //sam.ProcessArm(ltrig, rtrig);
            sam.RobotExtend(lbump, rbump);
            sam.DebugPrint(telemetry);

            if (gamepad1.a) sam.GrabberOpen();
            if (gamepad1.b) sam.GrabberClose();
            telemetry.update();
        }

        sam.RobotStop();

        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

}