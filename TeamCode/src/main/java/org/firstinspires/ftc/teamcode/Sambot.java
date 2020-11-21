//start code
package org.firstinspires.ftc.teamcode;

//import packages

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.util.Log;
@TeleOp(name = "Sambot", group = "Mechanum")
//Mechbot Class
public class Sambot extends LinearOpMode {

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

            sam.RobotDrive(px, py, pa);
            sam.ProcessArm(ltrig, rtrig);
            sam.RobotExtend(lbump, rbump);
            sam.DebugPrint(telemetry);


            if (gamepad1.a) sam.GrabberOpen();
            if (gamepad1.b) sam.GrabberClose();

            if (gamepad1.dpad_up || gamepad2.dpad_up) sam.Pinchin();
            if (gamepad1.dpad_down || gamepad2.dpad_down) sam.PinchOut();

            if (gamepad2.right_stick_y < -0.1) sam.WristFoldIn();
            if (gamepad2.right_stick_y > 0.1) sam.WristFoldOut();
            if (gamepad2.x) sam.StackBlock(1);
            if (gamepad2.y)sam.StackBlock(2);
            if (gamepad2.b) sam.StackBlock(3);
            if (gamepad2.a) sam.StackBlock(4);
            if (gamepad2.left_bumper && gamepad2.right_bumper)
                sam.DropCapStone();
            telemetry.update();



        }

        sam.RobotStop();

        //telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

}