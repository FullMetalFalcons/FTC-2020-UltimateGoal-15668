package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Park Center - Straight/turn right", group="SamBot")

public class AutoCommonParkAtCenter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        robot robot = new robot();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.RobotInit(hardwareMap);

        telemetry.addData("Path", "Simple Park");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        try {

            int ticks = robot.InchesToTick(27);
            robot.DriveToTick(ticks, .5);
            robot.RobotStop();
            robot.resetDriveUsingEncoder();
            robot.TurnToAngle(-90,1);
            robot.ResetDriveEncoder();
            robot.DriveToTick(1440,0.8);
        } catch (Exception ex) {
            telemetry.addData("Path", "Exception thrown");
        } finally {
            robot.RobotStop();
            robot.DebugPrint(telemetry);
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }


    }
}
