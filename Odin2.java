package org.firstinspires.ftc.teamcode.mechanisms;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.mechanisms.ShooterConstants.flywheelSpeed;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.lang.Math;
@TeleOp
public class Odin2 extends LinearOpMode {
    private CRServo turrell;
    private CRServo turrelr;

    private Servo stopperr;
    private Servo stopperl;


    private double kP = 0.015;
    private double kI = 0.0001;
    private double kD = 0.002;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean posLow = false;
    private boolean pos2Low = false;
    private double turouput=0;


    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    private Limelight3A limelight;
    private IMU imu;
    private double distance;


    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(8); //id 20
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot  revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor minimecanum = hardwareMap.dcMotor.get("minimecanum");

        DcMotorEx flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo servo = hardwareMap.get(Servo.class,"servo");

        turrell = hardwareMap.get(CRServo.class, "turrell");
        turrelr = hardwareMap.get(CRServo.class, "turrelr");


        turrell.setDirection(CRServo.Direction.FORWARD);


        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        servo.setDirection(Servo.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        minimecanum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        minimecanum.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopperr = hardwareMap.get(Servo.class, "stopperr");
        stopperl = hardwareMap.get(Servo.class, "stopperl");

        stopperr.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        limelight.start();
        double targetRPM=1000;
        double hoodPos = 0.5;


        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x / 0.5;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double brPower = (y + x - rx) / denominator;


            leftFront.setPower(flPower);
            leftBack.setPower(blPower);
            rightFront.setPower(frPower);
            rightBack.setPower(brPower);



            //flywheel dynamice ins
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult llResult = limelight.getLatestResult();


            double tur_integral = 0;
            double tur_lastError = 0;
            double tur_error = 0;

            if(llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                distance = getDistanceFromTags(llResult.getTa());
                tur_error = -llResult.getTx();
            }

            //intake
            float in= gamepad2.left_trigger;
            float out= gamepad2.right_trigger;

            if(in> 0.2){
                minimecanum.setPower(1);
            }
            else if(out > 0.2){
                minimecanum.setPower(-1);
            }
            else{
                minimecanum.setPower(0);
            }
            boolean in2= gamepad2.left_bumper;
            boolean out2= gamepad2.right_bumper;

            if(in2){
                intake.setPower(1);
            }
            else if(out2){
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }


            double fw_kP = 0.02;
            double fw_kI = 0.00000;
            double fw_kD = 0.000;
            double fw_kF = 0.0000; //наш поуер на таргемтики окей
            double fw_integral = 0;
            double fw_lastError = 0;


            ElapsedTime fwTimer = new ElapsedTime();
            fwTimer.reset();
            ElapsedTime turTimer = new ElapsedTime();
            turTimer.reset();



           //stopper
            boolean pos1 = gamepad2.a;

            if (pos1 && !prevA) {
                posLow = !posLow;
            }

            stopperr.setPosition(posLow ? 0.25 : 0.1);
            stopperl.setPosition(posLow ? 0.55 : 0.4);

            prevA = pos1;
            double borespeed = leftFront.getCurrentPosition()  ;


                double TPR = 28.0;
                targetRPM = (0.0000528302 * (Math.pow(distance, 3)) - (0.0227224 * Math.pow(distance, 2)) + (5.11456 * distance + 864.15094));
                servo.setPosition(0.0000215611 * Math.pow(distance, 2) - 0.00860677 * distance + 1.38557);

                double targetTicksPerSec = (targetRPM / 30.0) * TPR;   //if rpm 3000 the ticks are 1650

                double currentVelocity = flywheel2.getVelocity();
                double fw_error = targetTicksPerSec - currentVelocity;

                double fw_dt = fwTimer.seconds();
                fwTimer.reset();

                fw_lastError = fw_error;
                double fw_derivative = (fw_error - fw_lastError) / Math.max(fw_dt, 1e-6);


                double output = fw_kP * fw_error + fw_kI * fw_integral + fw_kD * fw_derivative + fw_kF * targetTicksPerSec;
                output = Math.max(-1, Math.min(1, output));

                if (gamepad2.y) {
                    flywheel1.setPower(output);
                    flywheel2.setPower(output);
                } else {
                    flywheel1.setPower(0);
                    flywheel2.setPower(0);
                    fw_integral = 0;
                }
                fw_integral += fw_error * fw_dt;


                double turadjustment = gamepad2.left_stick_x;
                if(turadjustment > 0.5){
                    turrell.setPower(0.8);
                    turrelr.setPower(0.8);
                }else{
                    turrell.setPower(0);
                    turrelr.setPower(0);
                }



                double tur_dt = turTimer.seconds();
                turTimer.reset();

                double tur_derivative = (tur_error - tur_lastError) / Math.max(tur_dt, 1e-6);
                tur_lastError = tur_error;

                double tur_kP = 0.02;
                double tur_kD = 0.000;

                turouput = tur_kP * tur_error + tur_kD * tur_derivative;
                turouput = Math.max(-1, Math.min(1, turouput));

                if(leftFront.getCurrentPosition() < -5000){
                    turouput = Math.max(turouput,0);
                }else if(leftFront.getCurrentPosition() > 5000){
                    turouput = Math.min(turouput,0);
                }





            turrell.setPower(turouput);
            turrelr.setPower(turouput);

            telemetry.addData("turretpos", borespeed);

            telemetry.addData("Target (ticks/s)", targetTicksPerSec);
            telemetry.addData("Hood Pos", hoodPos);
            telemetry.addData("RPM", (currentVelocity / 28.0) * 30.0);
            telemetry.addData("flywheel 1 velocity", flywheel1.getVelocity());
            telemetry.addData("Distance", distance);
            telemetry.addData("turouput", turouput);
            telemetry.addData("tur_error", tur_error);

            telemetry.addData("bore", leftFront.getCurrentPosition());
            telemetry.addData("ser_pos", servo.getPosition());

            telemetry.update();


            idle();

        }

        stop();
    }



    public static double getDistanceFromTags(double ta){
        return (174.188 * Math.pow(ta, -0.5479));
    }
}

