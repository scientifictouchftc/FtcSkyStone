
 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;


 public class Moverobot {


     private DcMotor FR = null;
     private DcMotor FL = null;
     private DcMotor BL = null;
     private DcMotor BR = null;
     static final double TICKS_PER_REV = 537.6;    // eg: TETRIX Motor Encoder
     static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
     static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
     static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
             (WHEEL_DIAMETER_INCHES * 3.1415);

     public enum DriveDirection {
         FORWARD,BACKWARD,RIGHT,LEFT
     }

     static final double DRIVE_SPEED = 0.6;
     static final double TURN_SPEED = 0.5;
     private ElapsedTime runtime = new ElapsedTime();

     private double rfLastEncoder;
     private double rbLastEncoder;
     private double lfLastEncoder;
     private double lbLastEncoder;
     private LinearOpMode op = null;
     private Telemetry telemetry = null;


     public Moverobot(DcMotor rf, DcMotor rb, DcMotor lf, DcMotor lb, Telemetry telemetry, LinearOpMode op) {

         this.FR = rf;
         this.BR = rb;
         this.FL = lf;
         this.BL = lb;
         this.telemetry = telemetry;
         this.op = op;

     }


     public void move(double speed, double leftInches, double rightInches) {

         int newLeftFrontTarget;
         int newRightFrontTarget;
         int newLeftBackTarget;
         int newRightBackTarget;


         // Determine new target position, and pass to motor controller

         newLeftFrontTarget = FL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
         newRightFrontTarget = FR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
         newLeftBackTarget = BL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
         newRightBackTarget = BR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
         //FL.setDirection(DcMotor.Direction.REVERSE);
        // BL.setDirection(DcMotor.Direction.REVERSE);


         FL.setTargetPosition(newLeftFrontTarget);
         FR.setTargetPosition(newRightFrontTarget);
         BL.setTargetPosition(newLeftBackTarget);
         BR.setTargetPosition(newRightBackTarget);

         // reset the timeout time and start motion.
         runtime.reset();
         setPowerAll(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

         setRunToPosition();

         // keep looping while we are still active, and there is time left, and both motors are running.
         // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
         // its target position, the motion will stop.  This is "safer" in the event that the robot will
         // always end the motion as soon as possible.
         // However, if you require that BOTH motors have finished their moves before the robot continues
         // onto the next step, use (isBusy() || isBusy()) in the loop test.
         while (op.opModeIsActive() &&
                 (runtime.seconds() < 31.0) &&
                         (FR.isBusy() && FL.isBusy() && (BR.isBusy() && (BL.isBusy())))) {

             // Display it for the driver.
             telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
             telemetry.addData("Path2", "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


             // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

             telemetry.update();
         }


// Stop all motion;
         stoprobot();



     }


     public void move_ticks(double speed, double distCm, DriveDirection direction) {

         int newLeftFrontTarget;
         int newRightFrontTarget;
         int newLeftBackTarget;
         int newRightBackTarget;

         // Determine new target position, and pass to motor controllerj6dmju6

         newLeftFrontTarget = FL.getCurrentPosition() ;
         newRightFrontTarget = FR.getCurrentPosition() ;
         newLeftBackTarget = BL.getCurrentPosition() ;
         newRightBackTarget = BR.getCurrentPosition();

         //int ticks = (int)(distCm / Math.cos(Math.PI/4) * COUNTS_PER_INCH);
         int ticks= 538;

         switch (direction){
             case FORWARD:
                 FR.setDirection(DcMotor.Direction.REVERSE);
                 BR.setDirection(DcMotor.Direction.REVERSE);
                 newLeftFrontTarget += ticks ;
                 newRightFrontTarget += ticks;
                 newLeftBackTarget += ticks;
                 newRightBackTarget += ticks;
                 break;
             case BACKWARD:
                 newLeftFrontTarget -= ticks ;
                 newRightFrontTarget -= ticks;
                 newLeftBackTarget -= ticks;
                 newRightBackTarget -= ticks;
                 break;
             case RIGHT:
                 newLeftFrontTarget -= ticks ;
                 newRightFrontTarget += ticks;
                 newLeftBackTarget += ticks;
                 newRightBackTarget -= ticks;
                 break;
             case LEFT:
                 newLeftFrontTarget += ticks ;
                 newRightFrontTarget -= ticks;
                 newLeftBackTarget -= ticks;
                 newRightBackTarget += ticks;
                 break;
         }

         FL.setTargetPosition(newLeftFrontTarget);
         FR.setTargetPosition(newRightFrontTarget);
         BL.setTargetPosition(newLeftBackTarget);
         BR.setTargetPosition(newRightBackTarget);

         FL.setDirection(DcMotor.Direction.REVERSE);
         BL.setDirection(DcMotor.Direction.REVERSE);

        // setRunToPosition();
         // reset the timeout time and start motion.
         runtime.reset();
         setPowerAll(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));
         setRunToPosition();

         // keep looping while we are still active, and there is time left, and both motors are running.
         // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
         // its target position, the motion will stop.  This is "safer" in the event that the robot will
         // always end the motion as soon as possible.
         // However, if you require that BOTH motors have finished their moves before the robot continues
         // onto the next step, use (isBusy() || isBusy()) in the loop test.
         while (op.opModeIsActive() &&
                 (runtime.seconds() < 31.0) &&
                 (FR.isBusy() && FL.isBusy() && (BR.isBusy() && (BL.isBusy())))) {

             // Display it for the driver.
             telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
             telemetry.addData("Path2", "Running at %7d :%7d", newLeftBackTarget, newRightBackTarget);


             // FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
             // FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

             telemetry.update();
         }


// Stop all motion;


     }


     public void runusingEncoder() {
         // Turn OFF RUN_TO_POSITION
         FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

     }

     public void resetEncoder() {
         // Turn OFF RUN_TO_POSITION
         FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

     }

     public void setRunToPosition() {
         // Turn ON RUN_TO_POSITION

         FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

     }

     public void setPowerAll(double rfSpeed, double lfSpeed, double rbSpeed, double lbSpeed) {
         FR.setPower(rfSpeed);
         FL.setPower(lfSpeed);
         BR.setPower(rbSpeed);
         BL.setPower(lbSpeed);
     }

     public void stoprobot() {
         FR.setPower(0.0);
         FL.setPower(0.0);
         BR.setPower(0.0);
         BL.setPower(0.0);
     }



     public double[] getEncoderPositions() {
         double[] encoders = {FR.getCurrentPosition() - rfLastEncoder, FL.getCurrentPosition() - lfLastEncoder, BL.getCurrentPosition() - lbLastEncoder, BR.getCurrentPosition() - rbLastEncoder};
         return encoders;
     }



 }








