 //These define's must be placed at the beginning before #include "TimerInterrupt.h"
#define TIMER_INTERRUPT_DEBUG      0

#define USE_TIMER_1     false
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include "TimerInterrupt.h"
#include "TimerOne.h"
#include <PID_v1.h>




// ----------------------------------------------
//PWM constant
#define PWM_Frequency_uS 5000
#define PWM_OUTPUT_DUTY_MAX_LIMIT  100 //the max output duty
// ------ PIN and HW definition  hardwire connect pin 5&6&9, pin 7&4
#define PWM_SPEED_PIN 9
#define PWM_direction_PIN 4
#define PWM_direction_PIN_B 7
volatile float PWM_duty_cycle = 0;
volatile int PWM_direction = 0;


volatile double delta_angle_omega_global=0;
volatile double delta_angle_omega_threahold=0;


// ----------------------------------------------
// system control
// Angle measurement constant
#define Angle_POTENTIAL_METER_PIN A0
#define Angle_MAPPING_VALUE1  180.0 // Resistance value corresponding to start Angle
#define Angle_MAPPING_VALUE2  900.0 // Resistance value corresponding to end Angle
#define Angle_MAPPING_RESULT1  4.4123  // adjust RESULT1 and RESULT 2 to mapping the potential meter value to 0.5Pi and 1.5Pi
#define Angle_MAPPING_RESULT2   1.2707 //
volatile double Angle = 0.0, Angle_real=0.0, Angle_previous=0.0, Angle_omega=0.0;
volatile float Angle_calibrate_delta=0;


// ----------------
volatile float acce_angle_k=210.0;
volatile float acce_omega_k=14.0;


long Encoder_previous_counter;

// for debug
int Print_data_flag=0;
int Print_data_type=0;
String Print_data_type_info[]={
  "Ang,Acc,S_set,S_msd",
  "S_ref,PID,S_set,S_msd",
  "PID,Acc,S_set,S_msd",
  "PID,Acc,Ang,S_msd"
};

// ----------------------------------------------
// system
#define Sync_TIMER_INTERVAL_MS    2  //sampling period
#define Sync_TIMER_INTERVAL_COUNTER_LIMIT    5  //sampling period
// system control virables
volatile int Sync_TIMER_INTERVAL_COUNTER=0;

volatile bool FLAG_system_start = false; // flag to turn on everything.  configured by serial command START / STOP
volatile bool FLAG_update_action = false;
volatile bool FLAG_Angle_calibrate_status=false;
volatile bool FLAG_System_Break = false;





// ----------------------------------------------
// Other facilitating variables
#define Meter2MilliMeterRatio 1000.0   // I use this ratio to convert sonar distance MM to equation unit of meter
#define MyMotor100pctPWMDrivingForce 11.0 // this number is calculated out basing on my motor spec under 11V condition.



// ----------------------------------------------
// Car speed testing
int Encoder_counter =0;
int Encoder_last_counter=0;
double Encoder_speed=0, Encoder_last_speed=0, Encoder_acceleration=0;
int Encoder_last_status=0;
int encoder_A_status =0,encoder_B_status=0;
bool Flag_encoder_change=0;
bool Serial_interruption_handler_flag=0;

//speed PID loop parameters
double Speed_measured=0,Speed_measured_last=0, Car_actual_acc_measured=0, Speed_error=0, Speed_ref=0,Speed_set=0, Speed_acc=0;
double Speed_pid_output=0, speed_pid_p=0, speed_pid_i=0,speed_pid_d=0; //speed_pid_i=2.79
long PID_MAPPING_INDEX_RANGE = 174;  // in fact the real speed is  between -/+ 1.4m/s . but here let's mapp it to duty / k_s2d gain

double angle_history_buffer_threshold=0.01;
int angle_history_buffer_all_below_threshold_counter=0;

//other control parameters
float car_wheel_diameter =0.075;
double Ks2d=1.0, Ks2d_offset=0.0;
int car_pwm_abs_duty_index=0,car_pwm_dir=1;

// ----------------------------------------------
// PWM Duty linear mapping
long pwm_duty_index =0;

// i put 3.0% as the min duty for motor in order to avoid totally brake effect.
float duty_cycle_mapping_array[] ={30,
85,
91,
93,
93,
93,
95,
96,
97,
98,
99,
100,
100,
101,
102,
103,
104,
105,
106,
107,
108,
109,
110,
111,
112,
113,
114,
115,
116,
117,
119,
120,
121.3,
122.0,
124.0,
125.0,
126.0,
127.0,
128.0,
128.4,
128.8,
129.3,
129.7,
131.0,
132.3,
133.0,
133.6,
134.4,
135.1,
136.0,
137.0,
138.0,
138.6,
139.3,
140.0,
140.6,
141.3,
142.0,
144.0,
144.7,
145.5,
146.5,
147.5,
149.0,
150.5,
151.5,
152.5,
153.5,
154.8,
156.4,
158.0,
160.0,
160.6,
161.3,
162.0,
163.0,
164.0,
166.6,
168.6,
170.0,
171.3,
172.5,
173.5,
174.6,
175.9,
177.3,
178.5,
179.5,
182.0,
184.0,
184.6,
185.3,
186.0,
188.0,
192.0,
194.0,
198.0,
202.0,
203.7,
205.4,
207.1,
209.0,
211.0,
212.9,
215.0,
218.0,
221.3,
224.0,
226.0,
228.0,
230.0,
232.0,
234.0,
236.0,
238.0,
244.0,
245.0,
246.0,
252.9,
255.0,
258.0,
261.5,
264.4,
268.0,
272.0,
275.4,
278.4,
284.0,
287.9,
292.0,
298.0,
301.0,
304.0,
308.0,
312.0,
318.0,
324.0,
331.0,
337.0,
340.9,
347.0,
352.9,
359.0,
367.0,
376.0,
384.0,
390.0,
400.0,
410.0,
414.0,
426.0,
438.0,
446.0,
454.0,
464.0,
476.0,
492.0,
515.9,
532.0,
538.0,
544.0,
555.9,
575.9,
595.9,
610.0,
628.0,
660.0,
678.0,
706.0,
728.0,
742.0,
768.0,
804.0,
822.0,
872.0,
898.0,
1000.0
};


// ----------------------------------------
// -----------PID control -----------------


PID myPID(&Speed_measured, &Speed_pid_output, &Speed_ref,speed_pid_p,speed_pid_i,speed_pid_d,DIRECT);







// ---- PID control func ---

void PID_initial()
{

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits((-1*PID_MAPPING_INDEX_RANGE),PID_MAPPING_INDEX_RANGE);
  myPID.SetSampleTime(Sync_TIMER_INTERVAL_MS*Sync_TIMER_INTERVAL_COUNTER_LIMIT); //milli second

}


//--------------------------
void Angle_initial()
{

}


void Angle_calibration()
{
  long k,k1,k2,k3,k4;
  k1=analogRead(Angle_POTENTIAL_METER_PIN);  // the reading will be between 0 -- 1023
  k2=analogRead(Angle_POTENTIAL_METER_PIN);  // the reading will be between 0 -- 1023
  k3=analogRead(Angle_POTENTIAL_METER_PIN);  // the reading will be between 0 -- 1023
  k4=analogRead(Angle_POTENTIAL_METER_PIN);  // the reading will be between 0 -- 1023
  k=long((k1+k2+k3+k4)/4);
  Angle_calibrate_delta = 2*3.1415926-float(map(k, Angle_MAPPING_VALUE1, Angle_MAPPING_VALUE2, Angle_MAPPING_RESULT1*10000, Angle_MAPPING_RESULT2*10000))/10000.0-PI;
  Angle_real=PI;
  Angle_previous=Angle_real;
  Angle=Angle_real;
}

void Angle_checking()
{
  long k=0;
  k=analogRead(Angle_POTENTIAL_METER_PIN);  // the reading will be between 0 -- 1023
  double k_angle=0;
  k_angle = 2*3.1415926-float(map(k, Angle_MAPPING_VALUE1, Angle_MAPPING_VALUE2, Angle_MAPPING_RESULT1*10000, Angle_MAPPING_RESULT2*10000))/10000.0-Angle_calibrate_delta;
  Angle_real = 0.2*k_angle + 0.8*Angle_real;

  Angle=Angle_real;

  if ( abs(Angle-PI) < angle_history_buffer_threshold){
     angle_history_buffer_all_below_threshold_counter ++;
     if (angle_history_buffer_all_below_threshold_counter > 1000)
     {
       angle_history_buffer_all_below_threshold_counter = 1000;
     }
  }
  else{
     angle_history_buffer_all_below_threshold_counter=0;
  }

}

void Angle_calculate_omega()
{

  delta_angle_omega_global=Angle-Angle_previous;


  if (abs(delta_angle_omega_global)<delta_angle_omega_threahold)
  {
    delta_angle_omega_global=0;
  }

  Angle_omega= delta_angle_omega_global /Sync_TIMER_INTERVAL_MS/Sync_TIMER_INTERVAL_COUNTER_LIMIT*1000.0;

  Angle_previous=Angle;

}






// ---------------------------
// ---------------------------
// ----- Speed sensing    ----
// ---------------------------
// ---------------------------


// handle encoder Sync_interrupt_handler
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3


void  Encoder_interrupt_initial()
{

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), Encoder_phase_change_trigger, FALLING  );

}

void Encoder_phase_change_trigger()
{

//  Flag_encoder_change=0;

  int b_status=0;
  b_status= digitalRead(ENCODER_B_PIN) ;

  if (b_status){
    Encoder_counter++;
  }
  else{
    Encoder_counter--;

  }
}


void Encoder_speed_update()
{
  //if serial communication interrupt the counting , then we use this way to handle it
 if(Serial_interruption_handler_flag)
 {
   Serial_interruption_handler_flag=false;
   Encoder_counter=Encoder_previous_counter;
 }

 // I dont know why yet, but we see some time the output jump to 256 - the number, thus if some weird number appear, I want to bypass it as a temp workaround
 if (abs(Encoder_counter)>100)
 {
   Encoder_counter=Encoder_previous_counter;
 }

 long temp_speed =0;
 temp_speed=Encoder_counter*(1000/(Sync_TIMER_INTERVAL_MS*Sync_TIMER_INTERVAL_COUNTER_LIMIT));
 Encoder_speed=0.2*temp_speed+0.8*Encoder_speed;
 Encoder_previous_counter=Encoder_counter;
 Encoder_counter=0;

}

void Speed_measured_cal()
{
  // the hall sensor in motor is 12 line per rotaion
  // the gear ratio for the motor is 1:34
  Speed_measured_last = Speed_measured;
  Speed_measured = car_wheel_diameter*PI*Encoder_speed/34/12;
  Car_actual_acc_measured =(Speed_measured - Speed_measured_last)*(1000/(Sync_TIMER_INTERVAL_MS*Sync_TIMER_INTERVAL_COUNTER_LIMIT));
}


void Speed_set_2_Duty_mapping()
{
  double temp_duty;
  temp_duty = abs(Speed_set) * Ks2d + Ks2d_offset;

  int array_length = 176;

  if ( temp_duty > array_length )
  {
    car_pwm_abs_duty_index = array_length;
    Speed_set=Speed_set/abs(Speed_set)*(array_length-Ks2d_offset)/Ks2d;
  }
  else{
    car_pwm_abs_duty_index=int(temp_duty);
  }

  if (Speed_set>0)
  {
    car_pwm_dir=1;
  }
  else
  {
    car_pwm_dir=0;
  }

}



void Car_calculate_speed_acceleration()
{
  double delta_angle=Angle-PI;
  double temp_omega=0;
  if (abs(delta_angle)<0.003) //action to stop infinite accumulation
  {
    delta_angle=0;
  }

  Speed_acc=acce_angle_k*delta_angle+acce_omega_k*Angle_omega;
/*
  if (abs(Angle_omega)>0.5){    //to gate out the shakking effect due to angle meter accuracy
    Speed_acc=acce_angle_k*delta_angle+acce_omega_k*Angle_omega;
  }
  else{
    Speed_acc=acce_angle_k*delta_angle;
  }
*/

}
















// -------------------------------------------
//-------Timer 1 PWM sub functions-----------------------
//
void PWM_initial()
{

  Timer1.initialize(PWM_Frequency_uS); // initialize timer1
  // put your setup code here, to run once:
  pinMode(PWM_SPEED_PIN, OUTPUT);
  pinMode(PWM_direction_PIN, OUTPUT);
  pinMode(PWM_direction_PIN_B, OUTPUT);

}

void PWM_set(float dutyCycle, int dir)
{
   digitalWrite(PWM_direction_PIN, dir);
   digitalWrite(PWM_direction_PIN_B, dir);
   Timer1.pwm(PWM_SPEED_PIN,(int)(dutyCycle * 1023 /100));
}




// -------------------------------------------
//------- Serial port Printing format -----------------------
//
void Print_info(String info2print)
{
  Serial.println("i"+info2print);
}

void Print_4data(float data1, float data2, float data3, float data4)
{
  Serial.println("d"+String(data1,4)+" "+String(data2,4)+" "+String(data3,4)+" "+String(data4,4)+" ");
}



//-------------------------------------------------
//----------- System periodical sampling and execution sub functions
//-------------------------------------------------

void Sync_initial()
{
  ITimer2.init();

  if (ITimer2.attachInterruptInterval(Sync_TIMER_INTERVAL_MS,Sync_interrupt_handler))  //add paramter here for call back func if need, follow original code
    Print_info("Sync sampling timer setup is done");
  else
    Print_info("Can not set up timer for sampling");

}


void Sync_interrupt_handler()
{
      FLAG_update_action = true;
}


void Sync_processing()
{

  if(FLAG_system_start)
  {
    Sync_TIMER_INTERVAL_COUNTER++;
    Angle_checking();



    if (Sync_TIMER_INTERVAL_COUNTER>=Sync_TIMER_INTERVAL_COUNTER_LIMIT)
    {


        Sync_TIMER_INTERVAL_COUNTER=0;

        //sampling data for PID process
        Angle_calculate_omega();
        Encoder_speed_update();
        Speed_measured_cal();   // here measured both speed and acceleration
        Car_calculate_speed_acceleration();

        // use Kangle = 200, komega = 14 for angle control only case.
        // use other testing number for speed and angle combined loop.

        //PID process and set target speed
        myPID.Compute();

        //Speed_pid_output =0;
        //for testing purpose we set Speed_pid_output to zero temprary
        //Speed_set = (Speed_acc - Speed_pid_output);  //we use positive feed back here to control the speed loop

        //Speed_set = Speed_set+(Speed_acc - Speed_pid_output)*(Sync_TIMER_INTERVAL_COUNTER_LIMIT*Sync_TIMER_INTERVAL_MS)/1000;

        double convert_pwm_duty_speed_ratio =0.008046;
        double check_info =0 ;
        check_info=(Speed_acc - Speed_pid_output)*(Sync_TIMER_INTERVAL_COUNTER_LIMIT*Sync_TIMER_INTERVAL_MS)/1000/ convert_pwm_duty_speed_ratio;

        double temp_last_Speed_set ;
        temp_last_Speed_set = Speed_set;

        // 2020.01.10 according to real speed to decide whether start accumulation or reset it
        if (abs(Speed_measured)<0.005){
          Speed_set=0;
        }

        Speed_set = Speed_set + check_info ;

        //  prevent change direction in stable status.
        if (angle_history_buffer_all_below_threshold_counter>(30*Sync_TIMER_INTERVAL_COUNTER_LIMIT)){
          if (( temp_last_Speed_set * Speed_set < 0 ) && (abs(Speed_set) < 5)) {
            Speed_set = temp_last_Speed_set;
          }
        }

        //drive output basing on this pid output
        if(!FLAG_System_Break)
        {
          if(FLAG_system_start)
          {
             Speed_set_2_Duty_mapping();
             PWM_set(duty_cycle_mapping_array[car_pwm_abs_duty_index]/10,car_pwm_dir);
          }
        }
        else{
          PWM_set(0,0);
        }

        // print for debug and mornitor
        if(Print_data_flag)
        {

          if (Print_data_type==0){
            Print_4data(Angle-PI,Speed_acc,Speed_set,Speed_measured);
          }

          if (Print_data_type==1)
          {
            Print_4data(Speed_ref,Speed_pid_output,Speed_set,Speed_measured);
          }

          if (Print_data_type==2){
            Print_4data(Speed_pid_output,Speed_acc,Speed_set,Speed_measured);
          }


          if (Print_data_type==3){
            Print_4data(Speed_pid_output,Speed_acc,Angle-PI,Speed_measured);
          }

          // real code
          //Print_4data(Angle,Angle_omega,Speed_acc,Speed_measured);
        }


    //  Print_4data(Encoder_counter);

    }

    //Fall down protection

    if (abs(Angle-PI)>0.7){
       FLAG_system_start = false;
       FLAG_Angle_calibrate_status=false;
       FLAG_System_Break=true;
       PWM_set(0,0);
       Print_info("system fall down");
    }


  }

  //Print_info(String(Angle,5));

 /*


  if(FLAG_system_start)
  {
    //Drive Motor with PWM
    float motor_duty ;
    int calculated_pwm_index=0;
    calculated_pwm_index=int(abs(PID_angle_blk_output));
    if (calculated_pwm_index >122){
      calculated_pwm_index = 122;
    }


    motor_duty= float(duty_cycle_mapping_array[calculated_pwm_index]/10.0);
    if (PID_angle_blk_output>0){
        PWM_set(motor_duty,0);
    }
    else{
        PWM_set(motor_duty,1);
    }

    //for test
    float k1=0.0;
    if (PID_angle_blk_output>0){
        k1=motor_duty;
    }
    else{
         k1=motor_duty*(-1);
    }


    Print_4data(String(Angle-PI,5)+" "+String(k1)+" "+String(Angle_kp*Angle_error)+" "+String(Angle_ki*Angle_error_sum)+" "+String(Angle_kd*Angle_error_diff*Angle_kd_switch));

  }

*/
}



//-------------------------------------------------
//----------- Serial command communicaiton subfuncs
//-------------------------------------------------



void serial_com_setup()
{

  //Serial.begin(9600);
  Serial.begin(115200);
  //Print_info("System Start ...");

}

void Upper_Stream_Comman_handler()
{

  String content=Serial.readString();

  //everytime recieve data, we need to reset encoder counter, otherwise the speed is not correct
  Serial_interruption_handler_flag=true;

  content.trim();

  String cmd=content.substring(0,1);
  float cmd_content=content.substring(1).toFloat();
  String cmd_content_str=content.substring(1);

  //system command
  if (cmd.equals("g")){
    if (FLAG_Angle_calibrate_status){
      Print_info("System Start");
      FLAG_system_start = true;
    }
    else{
      Print_info("Please calibrate Vertical Angle PI first, with command 'PI'");
    }
  }


  if (cmd.equals("s")){
      PWM_set(0, 0);
      FLAG_Angle_calibrate_status = false;
      FLAG_system_start = false;
      Print_info("System Stopped");
    }


  if (cmd.equals("v")){
      FLAG_Angle_calibrate_status = true;
      Angle_calibration();
      Print_info("Calibration: "+String(Angle_calibrate_delta,3));
    }


      if (cmd.equals("t")){
        Print_info("Motor Start Running");
        PWM_set(0,1);
        delay(200);
        Speed_set_2_Duty_mapping();
        PWM_set(duty_cycle_mapping_array[car_pwm_abs_duty_index]/10,car_pwm_dir);
      }


      if (cmd.equals("p")){
        //Print_4data(String(Speed_set)+" "+String(Speed_measured)+" "+String(Encoder_speed)+" "+String(Encoder_previous_counter));
        if (Print_data_flag)
        {
          Print_data_flag=false;
          Print_info("plot stop");
        }
        else{
          Print_data_flag=true;
          Print_info("plot enable");
        }
      }

      if (cmd.equals("k"))
      {
        Print_data_type = int(cmd_content);
        Print_info(Print_data_type_info[Print_data_type]);
      }

//-- updaet k1 and k2 ---

  if (cmd.equals("x")){
    long delimiter, delimiter_1, delimiter_2, delimiter_3;
    float subcomdtype,sub1,sub2,sub3;

    delimiter = cmd_content_str.indexOf("%");
    delimiter_1 = cmd_content_str.indexOf("%", delimiter + 1);
    delimiter_2 = cmd_content_str.indexOf("%", delimiter_1 +1);
    delimiter_3 = cmd_content_str.indexOf("%", delimiter_2 +1);

    subcomdtype=cmd_content_str.substring(0,delimiter).toFloat();
    sub1=cmd_content_str.substring(delimiter+1,delimiter_1).toFloat();
    sub2=cmd_content_str.substring(delimiter_1+1,delimiter_2).toFloat();
    sub3=cmd_content_str.substring(delimiter_2+1,delimiter_3).toFloat();

    Print_info(String(subcomdtype)+" "+String(sub1)+" "+String(sub2)+" "+String(sub3));
    //if command type is 1, change signal to pwm duty coefficient
    if (subcomdtype==1){
      Ks2d=sub1;
      Ks2d_offset=sub2;
    }

    //if command type is 2, change pid coefficient
    if (subcomdtype==2){
      speed_pid_p=sub1;
      speed_pid_i=sub2;
      speed_pid_d=sub3;
      myPID.SetTunings(speed_pid_p, speed_pid_i, speed_pid_d);
    }

    //if command type is 3, change target speed reference before plant
    if (subcomdtype==3){
      Speed_ref=sub1;
    }

    //if command type is 1, change acceleration calculation coefficient
    if (subcomdtype==4){
      acce_angle_k=sub1;
      acce_omega_k=sub2;
    }

    //if command type is 1, change acceleration calculation coefficient
    if (subcomdtype==5){
      delta_angle_omega_threahold=sub1;
      Print_info("ok ,updated angle_omega_buffer");
    }
  }


}





//-------------------------------------------------
//----------- facilitated calculation subfuncs
//-------------------------------------------------




void setup()
{

  serial_com_setup();
  PID_initial();
  PWM_initial();
  Encoder_interrupt_initial();
  Sync_initial();
  Angle_calibration();



}




void loop()
{


  // 1. process upper stream command
  if (Serial.available() > 0) {
    Upper_Stream_Comman_handler();
  }

 // 2. keep sampling , but do it outside call back func, in order to save accurate counter accumulation.



    if (FLAG_update_action)
   {
      FLAG_update_action=false;
      Sync_processing();
      //Sync_processing_test_angle();
      //Sync_processing_test_speed();
   }





}
