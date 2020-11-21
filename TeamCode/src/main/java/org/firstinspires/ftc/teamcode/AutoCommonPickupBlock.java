package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoCommonPickupBlock extends LinearOpMode {


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
            robot.PinchOut();
            sleep(100);
            robot.DriveToTick(1440,0.3);
            robot.RobotExtendToPosition(5400);
            sleep(100);
            robot.DriveToTick(-5760,0.3);
            robot.resetDriveUsingEncoder();
            robot.TurnToAngle(-45, 1);
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
