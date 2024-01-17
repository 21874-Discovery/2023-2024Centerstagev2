/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="random auto", group="Robot")
public class RoBoBoTowPoGoAuto extends LinearOpMode {

   // declare OpMode members


   private DcMotor topLeft;
   private DcMotor topRight;
   private DcMotor bottomLeft;
   private DcMotor bottomRight;

   private DcMotor intakeMotor;

   private DistanceSensor distanceSensor;

   int step=1;
   boolean toggle=false;


   private final double ppr=537.7;
   private final double     dgr    = 1.0 ;     // No External Gearing.
   private final double     wd   = 3.77953 ;     // For figuring circumference
   private final double     tpi         = (ppr * dgr)/(wd * Math.PI);

   private enum Direction{FORWARD,RIGHT,BACKWARD,LEFT,CW,CCW};


   public boolean distanceCalc(double list[]){
      double sum=0;

      //get the sum
      for(int x=0;x<list.length;x+=1) {

         sum+=list[x];
      }

      //get the average
      double average=sum/list.length;
      double stdev=0;

      //standard deviation step 3
      for(int x=0;x<list.length;x+=1) {
         stdev+=Math.pow(list[x]-average,2);
      }

      //divide and square root
      stdev=Math.sqrt(stdev/list.length);

      //if average larger than 315, not detecting
      if(stdev<=1&&average>=315){
         return false;
      }

        /*
        if standard deviation is larger than 20, so dataset very spread apart,
        then nothing is detected
         */
      else if(stdev>=20&&average>=100){
         return false;
      }
      else if(stdev>=50){
         return false;
      }
      else{
         return true;
      }


   }

   public boolean distanceDetector(){
      double list[]=new double[10];

      for(int x=0;x<list.length;x+=1) {
         list[x]=distanceSensor.getDistance(DistanceUnit.INCH);
      }
      return distanceCalc(list);
   }
   
   
   public void drive(Direction dir,double distance,double power){

      topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      topLeft.setPower(power);
      topRight.setPower(power);
      bottomLeft.setPower(power);
      bottomRight.setPower(power);


      switch(dir) {
         case FORWARD:
            topLeft.setTargetPosition((int) (distance * tpi));
            topRight.setTargetPosition((int) (distance * tpi));
            bottomLeft.setTargetPosition((int) (distance * tpi));
            bottomRight.setTargetPosition((int) (distance * tpi));
         case RIGHT:
            topLeft.setTargetPosition((int) (distance * tpi));
            topRight.setTargetPosition(-(int) (distance * tpi));
            bottomLeft.setTargetPosition(-(int) (distance * tpi));
            bottomRight.setTargetPosition((int) (distance * tpi));
         case BACKWARD:
            topLeft.setTargetPosition(-(int) (distance * tpi));
            topRight.setTargetPosition(-(int) (distance * tpi));
            bottomLeft.setTargetPosition(-(int) (distance * tpi));
            bottomRight.setTargetPosition(-(int) (distance * tpi));
         case LEFT:
            topLeft.setTargetPosition(-(int) (distance * tpi));
            topRight.setTargetPosition((int) (distance * tpi));
            bottomLeft.setTargetPosition((int) (distance * tpi));
            bottomRight.setTargetPosition(-(int) (distance * tpi));
         case CW:
            topLeft.setTargetPosition((int) (distance * tpi));
            topRight.setTargetPosition(-(int) (distance * tpi));
            bottomLeft.setTargetPosition((int) (distance * tpi));
            bottomRight.setTargetPosition(-(int) (distance * tpi));
         case CCW:
            topLeft.setTargetPosition(-(int) (distance * tpi));
            topRight.setTargetPosition((int) (distance * tpi));
            bottomLeft.setTargetPosition(-(int) (distance * tpi));
            bottomRight.setTargetPosition((int) (distance * tpi));
      }
      
      topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   }
   
   public boolean isDriving(){
      return isDriving()||topRight.isBusy()||bottomLeft.isBusy()||bottomRight.isBusy();
   }

   public void outTake(){
      intakeMotor.setPower(0.1);
      sleep(1500);
      intakeMotor.setPower(0);
   }


   @Override
   public void runOpMode() {

      topLeft=hardwareMap.get(DcMotor.class,"topLeft");
      topRight=hardwareMap.get(DcMotor.class,"topRight");
      bottomLeft=hardwareMap.get(DcMotor.class,"bottomLeft");
      bottomRight=hardwareMap.get(DcMotor.class,"bottomRight");
      
      intakeMotor=hardwareMap.get(DcMotor.class,"intakeMotor");

      distanceSensor=hardwareMap.get(DistanceSensor.class,"distanceSensor");


      //zero power behav

      //set position of wanted inches



      topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


      topLeft.setDirection(DcMotor.Direction.FORWARD);
      topRight.setDirection(DcMotor.Direction.REVERSE);
      bottomLeft.setDirection(DcMotor.Direction.FORWARD);
      bottomRight.setDirection(DcMotor.Direction.REVERSE);

      intakeMotor.setDirection(DcMotor.Direction.REVERSE);

      topLeft.setPower(0.1);
      topRight.setPower(0.1);
      bottomLeft.setPower(0.1);
      bottomRight.setPower(0.1);


      telemetry.addData("Status", "Initialized");
      telemetry.update();


      // init intake motors
      // init servo

      // Wait for the game to start (driver presses PLAY)
      waitForStart();



      // run until the end of the match (driver presses STOP)

      while (opModeIsActive()) {
         /* READ INPUTS */
         // read current values of joystick

         //STEP 1: DRIVE FORWARD
         if(step==1){
            if(distanceDetector()){
               step=4;
            }
            else{
               step=2;
            }
            //TODO: Figure out this logic. Maybe add a while opModeIsActive && topLeft.isBusy??
         }
         
         
         else if(step==2){
            drive(Direction.RIGHT,12,0.1);
            step=3;
         }
         
         else if(step==3){
            if(distanceDetector()){
               step=5;
            }
            else{
               if(!isDriving()){
                  step=6;
               }
            }
         }

         else if(step==4){
            drive(Direction.FORWARD,30,0.1);
            step=9;
         }
         
         else if(step==5){
            drive(Direction.FORWARD,20,0.1);
            step=9;
         }

         else if(step==6){
            drive(Direction.FORWARD, 32, 0.1);
            step=7;
         }

         else if(step==7){
            if(!isDriving()) {
               drive(Direction.CW, 17.28, 0.1);
               step=8;
            }
         }

         else if(step==8){
            if(!isDriving()) {
               drive(Direction.BACKWARD, 16, 0.1);
               step=10;
            }
         }


         else if(step==9){
            if(!isDriving()){
               drive(Direction.CW,34.56,0.1);
               step=10;
            }
         }


         else if(step==10){
            if(!isDriving()){
               outTake();
            }
         }


         else {
            topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }




         telemetry.addData("topLeft: ", topLeft.getDirection());
         telemetry.addData("topRight: ", topRight.getDirection());
         telemetry.addData("bottomLeft: ", bottomLeft.getDirection());
         telemetry.addData("bottomRight: ", bottomRight.getDirection());
         telemetry.addData("step: ", step);
         telemetry.addData("detecting: ", distanceDetector());
         telemetry.update();

      }
      topLeft.setPower(0.0);
      topRight.setPower(0.0);
      bottomLeft.setPower(0.0);
      bottomRight.setPower(0.0);
      intakeMotor.setPower(0.0);
   }


}

