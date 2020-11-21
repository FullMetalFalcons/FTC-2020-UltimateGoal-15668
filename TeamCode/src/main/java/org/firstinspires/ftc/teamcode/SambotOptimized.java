//start code
package org.firstinspires.ftc.teamcode;

//import packages

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "SambotOptimized", group = "Mechanum")
//Mechbot Class
public class SambotOptimized extends LinearOpMode {

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

            double px = -gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;

            float ltrig = gamepad1.left_trigger;
            float rtrig = gamepad1.right_trigger;

            //if (rtrig > 0) sam.LiftArm();
            //if (ltrig > 0) sam.DropArm();

            boolean lbump = gamepad1.left_bumper;
            boolean rbump = gamepad1.right_bumper;

            sam.RobotDriveOptimzed(px, py, pa);
            sam.ProcessArm(ltrig, rtrig);
            sam.RobotExtend(lbump, rbump);
            sam.DebugPrint(telemetry);

            if (gamepad1.a) sam.GrabberOpen();
            if (gamepad1.b) sam.GrabberClose();

            if (gamepad1.dpad_up) sam.Pinchin();
            if (gamepad1.dpad_down) sam.PinchOut();

            telemetry.update();
        }

        sam.RobotStop();

        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

}