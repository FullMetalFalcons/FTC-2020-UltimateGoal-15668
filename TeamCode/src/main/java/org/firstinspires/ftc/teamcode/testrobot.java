//start code
package org.firstinspires.ftc.teamcode;

//import packages

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class testrobot {

    //variable
    private ElapsedTime runtime = new ElapsedTime();
    private Servo Arm = null;

    protected DcMotor m1 = null;
    protected DcMotor m2 = null;
    protected DcMotor m3 = null;
    protected DcMotor m4 = null;
    protected DcMotor m5 = null;
    protected DcMotor m6 = null;

    private DcMotorWrapper m1opt = null;
    private DcMotorWrapper m2opt = null;
    private DcMotorWrapper m3opt = null;
    private DcMotorWrapper m4opt = null;
    private DcMotorWrapper m5opt = null;

    protected Servo LClam = null;
    protected Servo Wrist = null;
    protected Servo RClam = null;
    protected Servo LPinch = null;
    protected Servo RPinch = null;
    protected Servo CapStoneGate = null;

    private boolean isArmHolding = true;
    private int ArmLastPosition = 0;
    private BNO055IMU imu;

    private boolean isStacking  = false;

    private Orientation lastAngles    = new Orientation();
    private double globalAngle;
    private PIDController           pidRotate;
    private final  double LPinch_START_POSITION= 0.55;
    private final double  RPinch_START_POSITION= 0.75;

    private final double RPinch_DROP_POSITION = 0.56;
    private final double LPinch_DROP_POSITION = 0.73;

    private final double RPinch_LIFT_POSITION = 0.97;
    private final double LPinch_LIFT_POSITION = 0.35;

    private final double inchesPerRev = 12.5;

    private final int Extender_LIMIT = 5500;

    public void RobotInit(HardwareMap map)
    {
        PIDController pidDrive;

        m1 = map.dcMotor.get("back_left_motor");
        m2 = map.dcMotor.get("front_left_motor");
        m3 = map.dcMotor.get("front_right_motor");
        m4 = map.dcMotor.get("back_right_motor");

        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m1opt = new DcMotorWrapper(m1);
        m2opt = new DcMotorWrapper(m2);
        m3opt = new DcMotorWrapper(m3);
        m4opt = new DcMotorWrapper(m4);
        m5opt = new DcMotorWrapper(m5);

        m6 = map.dcMotor.get("lift");
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m5 = map.dcMotor.get("extend");
        m5.setDirection(DcMotor.Direction.REVERSE);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LClam = map.servo.get("LClam");
        Wrist = map.servo.get("Wrist");
        RClam = map.servo.get("RClam");
        LPinch = map.servo.get("LPinch");
        RPinch = map.servo.get("RPinch");
        CapStoneGate = map.servo.get("CapStoneGate");

        LClam.setPosition(1);
        RClam.setPosition(0);
        Wrist.setPosition(.85);
        CapStoneGate.setPosition(0.5);

        LPinch.setPosition(LPinch_START_POSITION);
        RPinch.setPosition(RPinch_START_POSITION);
        imu = map.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        pidRotate = new PIDController(.004, .00003, 0);

        //HoldArm(200);
        initExtender();
        resetAngle();

    }


    public int InchesToTick(double inches) {
        double revValue = 1440/inchesPerRev * inches;
        return (int)Math.round(revValue);
    }

    public void resetDriveUsingEncoder() {
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public  void initExtender() {
        //Move Extender slowly until it cannot move forward
        int pos, previous;
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pos = 0;
        try {
            do  {
                m5.setPower(-.1);
                Thread.sleep(200);
                m5.setPower(0);
                previous = pos;
                pos = m5.getCurrentPosition();
            } while(Math.abs(pos - previous)  > 10);

        }
        catch (Exception e)
        {

        }

        m5.setPower(0);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void RobotDrive(double px, double py, double pa)
    {
        if (Math.abs(px) < 0.05) px = 0;
        if (Math.abs(py) < 0.05) py = 0;
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);

    }
    public void RobotDriveOptimzed(double px, double py, double pa)
    {
        if (Math.abs(px) < 0.05) px = 0;
        if (Math.abs(py) < 0.05) py = 0;
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1opt.setPower(p1);
        m2opt.setPower(p2);
        m3opt.setPower(p3);
        m4opt.setPower(p4);

    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    double getHeadingRadians() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -angles.firstAngle;   // Not sure why this is negative, could be the simulator

        /*
        angles.firstAngle is the heading, measured COUNTER-CLOCKWISE, from the orientation the bot was in when the
        op mode was started (by default, straight upward). In the driveFieldRelative method, the X axis is the robot's
        starting forward direction and the Y axis is 90 degrees CLOCKWISE from the X axis. Likewise, in the
        driveMechanum method, the strafe direction (robot-right) is 90 degrees CLOCKWISE from the forward direction.
        It is these choices of coordinate systems that necessitate using the negative of angles.firstAngle.
         */

    }

    void RobotDriveFieldRelative(Telemetry telemetry, double x, double y, double rotate) {
        //Convert x, y to theta, r

        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);

        //Get modified theta, r based off gyro heading
        double heading = getHeadingRadians();

        double modifiedTheta = theta - heading;

        //Convert theta and r back to a forward and strafe
        double forward = r * Math.cos(modifiedTheta);
        double strafe = r * Math.sin(modifiedTheta);

        this.RobotDrive(forward, strafe, rotate);
    }



    public void TurnToAngle(double degrees, double power) throws Exception{
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);


        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            //On right turn we have to get off zero first.
            //while (getAngle() == 0)
            //{
            //    RobotDrive(0,0, 0.5);
            //    Thread.sleep(50);
            //}

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.

                Log.i("Power:", Double.toString(power));
                RobotDrive(0,0, power);

            } while (!pidRotate.onTarget());
        }
        else {   // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                RobotDrive(0, 0, power);
                Log.i("Power:", Double.toString(power));
            } while (!pidRotate.onTarget());
        }
        RobotDrive(0,0, 0);

    }

    public void ResetDriveEncoder(){
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }



    public void DriveToTick(int tick, double power) throws Exception {


        m1.setTargetPosition(tick);
        m2.setTargetPosition(tick);
        m3.setTargetPosition(tick);
        m4.setTargetPosition(tick);

        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);


        Thread.sleep(10);
        boolean notInPosition = true;
        do {
            int m1Pos = m1.getCurrentPosition();
            int m2Pos = m2.getCurrentPosition();
            int m3Pos = m3.getCurrentPosition();
            int m4Pos = m4.getCurrentPosition();
            int avgPos = (m1Pos + m2Pos + m3Pos + m4Pos)/4;
//            Log.d("Avg Tic:",Integer.toString(avgPos));
//            Log.d("DriveToTick:m1",Integer.toString(m1Pos));
//            Log.d("DriveToTick:m2",Integer.toString(m2Pos ));
//            Log.d("DriveToTick:m3",Integer.toString(m3Pos));
//            Log.d("DriveToTick:m4",Integer.toString(m4Pos ));
//            Log.d("NotInPosition", Boolean.toString(notInPosition));

            notInPosition = Math.abs(tick-avgPos) > 30;
            Thread.sleep(20);

        } while (notInPosition);



        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    };



    public void RobotStop()
    {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);
        m6.setPower(0);


    }


    public void ProcessArm(float ltrig,float rtrig) {

        // Before moving arm... Make use the neutral position do not conflict with preprogram macro

        if (ltrig == 0 && rtrig == 0 && CheckStacking())
            return;

        int pos = m6.getCurrentPosition();

        // if motor is not moving and trigger is not pressed then do nothing
        if (ltrig ==0 && rtrig == 0)
        {
            if (!isArmHolding) {
                isArmHolding = true;
                ArmLastPosition = pos;
            }
            else {

                return;
            }
            if (pos > 10) {

                HoldMotorPosition(m6, ArmLastPosition,0.4);
            } else {
                m6.setPower(0);
            }
            return;
        }
        else
            isArmHolding = false;

        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // ltrig will lower the arm
        if (ltrig > 0) {

            m6.setPower(-.2 * ltrig);
            return;
        }

        // rtrig will raise the arm
        if (rtrig > 0) {

            if (pos < 1000) {
                double multiplier = 0.4;
                if (pos < 500)
                    multiplier = 0.1;

                m6.setPower(multiplier * rtrig);
            } else {
                HoldMotorPosition(m6, 1000,0.8);
                isArmHolding = true;
                ArmLastPosition = 1000;

            }
            return;
        }

    }


    public void HoldArm(int pos)
    {
        MotorMoveToPosition(m6,pos,.2);
    }
    public void LiftArm() {
        MotorMoveToPosition(m6,1000,.5);
    }

    public void DropArm() {
        MotorMoveToPosition(m6,0, 0.3);
    }

    private void HoldMotorPosition(DcMotor motor, int pos) {
        MotorMoveToPosition(motor, pos, 0.5);

    }
    private void HoldMotorPosition(DcMotor motor, int pos, double power) {
        MotorMoveToPosition(motor, pos, power);

    }
    protected void MotorMoveToPosition(DcMotor m, int pos, double power) {
        m.setTargetPosition(pos);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(power);
    }




    public void RobotExtendToPosition(int pos) {

        MotorMoveToPosition(m5, pos,.5);
    }


    private void ImplRobotExtend (boolean lBump, boolean rBump, boolean safetyOn ) {


        if (!lBump && !rBump && CheckStacking())
            return;

        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int pos  = m5.getCurrentPosition();

        if (lBump == false && rBump == false) {
            m5.setPower(0);
        } else if (lBump == true) {
            if (pos < Extender_LIMIT -100 && safetyOn)
                m5.setPower(.7);
            else
                m5.setPower(0);
        } else if (rBump == true) {
            if (pos > 0 + 100 && safetyOn)
                m5.setPower(-.7);
            else
                m5.setPower(0);
        }
    }

    public void RobotExtend (boolean lBump, boolean rBump) {
        ImplRobotExtend(lBump,rBump,true);
    }


    public void RobotExtendNoSafety (boolean lBump, boolean rBump) {
        ImplRobotExtend(lBump,rBump,false);
    }

    public void RobotExtendOptimized (boolean lBump, boolean rBump) {
        int pos  = m5.getCurrentPosition();

        if (lBump == false && rBump == false) {
            m5opt.setPower(0);
        } else if (lBump == true) {
            if (pos < Extender_LIMIT -100)
                m5opt.setPower(.6);
            else
                m5opt.setPower(0);
        } else if (rBump == true) {
            if (pos > 0 + 100)
                m5opt.setPower(-.6);
            else
                m5opt.setPower(0);
        }
    }

    public void AdjustRisk() {
        Wrist.setPosition(.4);
    }
    public void WristFoldOut() {
        double pos = Wrist.getPosition();
        Wrist.setPosition(pos + 0.02);
    }
    public void WristFoldIn() {
        double pos = Wrist.getPosition();
        Wrist.setPosition(pos - 0.02);
    }

    public void PickupPosition(){

        HoldMotorPosition(m6,250, 0.3);
        GrabberOpen();

    }

    public void GrabberOpen () {
        LClam.setPosition(0);
        RClam.setPosition(1);

    }

    public void GrabberClose() {
        LClam.setPosition(.75);
        RClam.setPosition(.25);
    }

    public void DebugPrint (Telemetry t)
    {
        t.addData("Extender", m5.getCurrentPosition());
        t.addData("Arm", m6.getCurrentPosition());
        t.addData("Wrist",Wrist.getPosition());
        t.addData("m1", m1.getCurrentPosition());
        t.addData("m2", m2.getCurrentPosition());
        t.addData("m3", m3.getCurrentPosition());
        t.addData("m4", m4.getCurrentPosition());

    }

    public void DropCapStone() {
        CapStoneGate.setPosition(1);
    }
    public void Pinchin () {
        Pinchin(true,true);
    }
    public void PinchOut () {
        LPinch.setPosition(LPinch_LIFT_POSITION);
        RPinch.setPosition(RPinch_LIFT_POSITION);
    }
    public void Pinchin(boolean right, boolean left) {
        if (right)
            RPinch.setPosition(RPinch_DROP_POSITION);
        if (left)
            LPinch.setPosition(LPinch_DROP_POSITION);
    }

    public void PinchOut (boolean right, boolean left) {
        if (left)
            LPinch.setPosition(LPinch_LIFT_POSITION);
        if (right)
            RPinch.setPosition(RPinch_LIFT_POSITION);
    }

    public boolean isArmBusy() {
        return m6.isBusy();
    }


    public boolean CheckStacking(){
        if (!isStacking)
            return false;

        if (!m5.isBusy() && !m6.isBusy()){
            isStacking = false;
            return false;
        }
        else
        {
            return true;
        }
    }
    public void StackBlock(int i) {
        int ArmPos =0;
        int ExtensionPos=0;
        double RiskPos=0;

        switch(i)
        {
            case 0:
                ArmPos = 550;
                ExtensionPos = 5000;
                RiskPos =0.52;
                break;
            case 1:
                ArmPos = 732;
                ExtensionPos = 2400;
                RiskPos =0.52;
                break;
            case 2:
                ArmPos = 732;
                ExtensionPos = 1964;
                RiskPos =0.52;
                break;
            case 3:
                ArmPos = 732;
                ExtensionPos = 2779;
                RiskPos =0.62;
                break;
            case 4:
                ArmPos = 1200;
                ExtensionPos = 5800;
                RiskPos =0.73;
                break;
            default:
                return;

        }

        MotorMoveToPosition(m5,ExtensionPos,1);
        MotorMoveToPosition(m6, ArmPos,0.8);
        Wrist.setPosition(RiskPos);

        isStacking = true;
//        while (m5.isBusy() || m6.isBusy()  )
//        {
//
//            if ((Math.abs(ArmPos - m6.getCurrentPosition()) < 25) && (Math.abs(ExtensionPos-m5.getCurrentPosition()) < 10))
//                    break;
//            try {
//                Thread.sleep(50);
//                Log.i("Stack", "waiting");
//            } catch (Exception ex) {
//            }
//        }

    }


}