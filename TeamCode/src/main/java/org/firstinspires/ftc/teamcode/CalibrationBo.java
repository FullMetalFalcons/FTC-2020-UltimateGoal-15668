//start code
package org.firstinspires.ftc.teamcode;

//import packages

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "Sambot-Calibration", group = "Mechanum")
//Mechbot Class
public class CalibrationBo extends LinearOpMode {

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


        int m6pos = 0;
        sam.AdjustRisk();
        while (opModeIsActive()) {

            double px = -gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;


            float ltrig = gamepad1.left_trigger;
            float rtrig = gamepad1.right_trigger;

            if (rtrig > 0) sam.LiftArm();
            if (ltrig > 0) sam.DropArm();

            boolean lbump = gamepad1.left_bumper;
            boolean rbump = gamepad1.right_bumper;

            sam.RobotDrive(px, py, pa);
            sam.ProcessArm(ltrig, rtrig);
            sam.RobotExtendNoSafety(lbump, rbump);
            sam.DebugPrint(telemetry);

            if (gamepad1.a) sam.GrabberOpen();
            if (gamepad1.b) sam.GrabberClose();

            double LPosition = sam.LPinch.getPosition();
            double RPosition = sam.RPinch.getPosition();
            double WristPosition = sam.Wrist.getPosition();
            double step  = 0.02;
            if (gamepad1.dpad_up) {

             sam.LPinch.setPosition(step+LPosition);
            }
            if (gamepad1.dpad_down) {

             sam.LPinch.setPosition(-1 * step+LPosition);
            }


            if (gamepad1.dpad_left) {

                sam.RPinch.setPosition(step+RPosition);
                sam.Wrist.setPosition(WristPosition - step);

            }
            if (gamepad1.dpad_right) {
                sam.Wrist.setPosition(WristPosition + step);
                sam.RPinch.setPosition(-1 * step +RPosition);
            }


            if (gamepad1.a && gamepad1.b)
                sam.initExtender();

            if (gamepad1.y) {
                m6pos += 50;
                sam.MotorMoveToPosition(sam.m6, m6pos, .3);
            }
            if (gamepad1.x) {
                sam.DropArm();
                m6pos = 0;
            }


            if (gamepad2.x) {
           //     sam.StackBlock();
            }
            sam.DebugPrint(telemetry);
            telemetry.addData("LPinch Pos", sam.LPinch.getPosition());
            telemetry.addData("RPinch Pos", sam.RPinch.getPosition());
            telemetry.addData("RPinch Pos", sam.Wrist.getPosition());
            telemetry.update();
        }

        sam.RobotStop();

        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

}