package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

@Autonomous(name = "Final Auto Vision", group = "Concept")

public class TensorFlowFinal extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AW4Kjyf/////AAABmXIr/SXbv02xp0txPYNnhm0n1cDgB6aMDCKParQ2uA0v/QtUkGssXl5ck9XB4uIGe6O7g6l511DOFM85owy9jYanoHy+fVF9adBCoF28B/E7TGx/4YDfakBltEG7fzRMt+dBBwEQ13WbNEoUtNx5+HUFI3RaKcxVa0e+0kb+FGvjh9+0Wvy4E2zfPzxrNHFnhF436L48+Z7bz116uAk5lRlpluKY303A3fW5bqh85ze2elNNMLE2SKXjpk9u4hALYdkKCMnc1Oos9kAok7k41I/4gPo4IjqwUot/wXdUE7znU7nP/4QB0cVe83x8VqnmG7sEhxmfk5tIyknYJ/QEXiL0+iucqKw1oGL4q041vFF5";

    DcMotor m1, m2, m3, m4, m6, m7, m8;
    DcMotorEx m5;
    Servo m9, m10;
    BNO055IMU imu;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    int detect;

    @Override
    public void runOpMode() {

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

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m7.setTargetPosition(0);
        m7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m8.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);
        Orientation orientation;

        m9.setPosition(1);
        m10.setPosition(1);

        String understood = "";

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        while (opModeIsActive() != true) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0) {
                        //empty list, no objects recognized
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                    } else {
                        //list is not empty
                        // step through the list of recognitions and display boundary Logger.info
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format(" left,top (%d)", i), "%.03f, %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("right, bottom(%d)", i), "%.03f,%.03f",
                                    recognition.getRight(), recognition.getBottom());
                            //check label to see which target zone to go after
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                            understood = recognition.getLabel();
                        }
                    }
                    telemetry.update();
                }
            }
        }

        waitForStart();

        if (understood == "Quad") {

            driveToValueE(-0.25, -2100);

            shooterRun(1400,3500);
            m6.setPower(0);
            m5.setVelocity(0);
            m8.setPower(0);


            driveToValueE(-0.2, -1700);
            sleep(1000);
            driveToValueE(0.2, 1500);


            setPower(0, 0, -0.5f);
            while (opModeIsActive()){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (orientation.firstAngle <= -123) break;
            }
            setPower(0, 0, 0);
            sleep(100);

            driveToValueE(0.2, 1800);


            m7.setTargetPosition(1050);
            m7.setPower(0.25);
            sleep(1500);
            m10.setPosition(0.5);
            sleep(700);
            m7.setTargetPosition(0);
            m7.setPower(0.25);
            sleep(1500);
            m10.setPosition(1);
            sleep(200);

            driveToValueE(-0.2, -800);

        } else if  (understood == "Single") {

            driveToValueE(-0.25, -2100);

            shooterRun(1400,3500);
            m6.setPower(0);
            m5.setVelocity(0);
            m8.setPower(0);

            setPower(0, 0, -0.5f);
            while (opModeIsActive()){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (orientation.firstAngle <= -140) break;
            }
            setPower(0, 0, 0);
            sleep(100);

            driveToValueE(0.2, 600);


            m7.setTargetPosition(1050);
            m7.setPower(0.25);
            sleep(1500);
            m10.setPosition(0.5);
            sleep(700);
            m7.setTargetPosition(0);
            m7.setPower(0.25);
            sleep(1500);
            m10.setPosition(1);
            sleep(200);

            driveToValueE(-0.2, -300);

        } else {

            driveToValueE(-0.25, -2100);

            shooterRun(1400,3500);
            m6.setPower(0);
            m5.setVelocity(0);
            m8.setPower(0);

            driveToValueE(-0.2, -500);

            setPower(0, 0, -0.5f);
            while (opModeIsActive()){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (orientation.firstAngle <= -65) break;
            }
            setPower(0, 0, 0);
            sleep(100);

            driveToValueE(0.2, 350);

            m7.setTargetPosition(1050);
            m7.setPower(0.25);
            sleep(1500);
            m10.setPosition(0.5);
            sleep(700);
            m7.setTargetPosition(0);
            m7.setPower(0.25);
            sleep(1500);
            m10.setPosition(1);
            sleep(200);

            driveToValueE(-0.2, -400);

        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    void driveToValueE(double speed, int tick) {
        m1.setTargetPosition(tick);
        m2.setTargetPosition(tick);
        m3.setTargetPosition(tick);
        m4.setTargetPosition(tick);
        m1.setPower(speed);
        m2.setPower(speed);
        m3.setPower(speed);
        m4.setPower(speed);
        boolean notInPosition = true;
        do {
            int m1Pos = m1.getCurrentPosition();
            int m2Pos = m2.getCurrentPosition();
            int m3Pos = m3.getCurrentPosition();
            int m4Pos = m4.getCurrentPosition();
            int avgPos = (m1Pos + m2Pos + m3Pos + m4Pos) / 4;
            notInPosition = Math.abs(tick - avgPos) > 30;
            sleep(20);
        } while (notInPosition);
        sleep(100);
        resetEncValuesEnc();
    }

    void driveToValueT(double speed, long time) {
        m1.setPower(speed);
        m2.setPower(speed);
        m3.setPower(speed);
        m4.setPower(speed);
        sleep(time);
    }

    void driveToValueO(double x, double y, double rx, double position) {
        while (abs(m1.getCurrentPosition()) < abs(position)) {
            setPower(x, y, rx);
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(100);
        resetEncValuesEnc();
    }

    void shooterRun(double rpm, long time) {
        while (m5.getVelocity() <= rpm - 10) {
            m5.setVelocity(rpm);
            m9.setPosition(0.22);
            m6.setPower(-1);
        }
        while (m5.getVelocity() >= rpm - 10) {
            m9.setPosition(0.05);
            m6.setPower(-1);
        }
        sleep(200);
        m8.setPower(-0.8);
        m5.setVelocity(rpm);
        sleep(time);
        m9.setPosition(0.9);
        sleep(200);
    }

    void resetEncValuesEnc() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetEncValuesPos() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m1.setTargetPosition(0);
        m2.setTargetPosition(0);
        m3.setTargetPosition(0);
        m4.setTargetPosition(0);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setPower(double x, double y, double rx) {
        m1.setPower(y - x + rx);
        m2.setPower(y + x + rx);
        m3.setPower(y - x - rx);
        m4.setPower(y + x - rx);
    }

}