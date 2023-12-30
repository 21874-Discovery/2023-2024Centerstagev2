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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="armDemo", group="Linear Opmode")
public class armDemo extends LinearOpMode {

   // declare OpMode members


   // intake
   private DcMotor         armMotor        = null;
   private Servo           clawServo           = null;


   private double prevArm = 0.0;

   private double ppr = 4.0;
   //ticks per revolution
   private double gr = 72.0;
   //gear ratio

   private boolean clawToggle=true;

   private boolean justStopped=true;

   @Override
   public void runOpMode() {

      telemetry.addData("💪 Status", "Initialized 😎");
      telemetry.update();
      // init intake motors
      // init servo
      armMotor       = hardwareMap.get(DcMotor.class, "armMotor");

      clawServo       = hardwareMap.get(Servo.class, "clawServo");


      armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //reset encoder position

      armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

      armMotor.setTargetPosition(0);

      clawServo.setPosition(0);
      // Wait for the game to start (driver presses PLAY)
      waitForStart();

      // run until the end of the match (driver presses STOP)

      while (opModeIsActive()) {
         /* READ INPUTS */
         // read current values of joystick
         double right_y=gamepad1.right_stick_y;
         boolean a = gamepad1.a;


         armMotor.setPower(right_y);

         // if no joystick, hold position
         /*if(right_y<=0.05){
            if(justStopped) {
               armMotor.setTargetPosition(armMotor.getCurrentPosition());
               justStopped=false;
            }
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }
         else {
            justStopped=true;
            armMotor.setPower(right_y);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         }*/

         // if joystick, then command the motor proportionally with the input

      /*
         if (right_y < Math.abs(prevArm)) {
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         }
         else {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setTargetPosition(armMotor.getCurrentPosition());
            prevArm = right_y;
         }

*/
         if(a){
            if(clawToggle) {
               if(clawServo.getPosition()==0||clawServo.getPosition()==-1) {
                  clawServo.setPosition(1);
               }
               else{
                  clawServo.setPosition(-1);
               }
               clawToggle=false;
            }
         }
         else{
            clawToggle=true;
         }

         telemetry.addData("Current Power 👍", armMotor.getPower());
         telemetry.addData("Current Y 👍", right_y);
         telemetry.addData("Current Position 👍", armMotor.getCurrentPosition());
         telemetry.addData("Current Target 👍", armMotor.getTargetPosition());
         telemetry.addData("Current Previous POWER NEWEST 👎", armMotor.isBusy());
         telemetry.update();
      }

      armMotor.setPower(0);

   }
}
