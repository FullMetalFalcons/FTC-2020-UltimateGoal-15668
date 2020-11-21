package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Park Side Wall", group="SamBot")
public class AutoParkSideWall extends LinearOpMode {
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

            int ticks = robot.InchesToTick(10);
            robot.DriveToTick(ticks, .5);
            robot.RobotStop();

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
