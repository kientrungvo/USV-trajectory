
// -------------------------------CODE for ESP8266 Nodemcu-------------------------------
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

// MODE CHOICE
  const int pin_mode = 14;        // D5
  bool auto_mode = 0;
  bool test_mode = false;
  int select_auto_mode = 1;

// MOTOR VARIABLES
  const int pin_motor_1 = 12;     // D6
  const int pin_motor_2 = 15;     // D8
  const int pin_test = 4;         // D2 
  int throttle;
  int turn;
  int discrete_control;
  int discrete_moment;
  int thrust;
  int moment;
  int speed_1;
  int speed_2;
  int test_speed = 0;

// BATTERY VOLTAGE
  float voltage_bat = 0;

// OBSTACLE AVOIDANCE
  int obstacle_scenario = 0;
  int obstacle_side = 0;
  int front_detect = 0;

  float distance_left = 10;
  float distance_right = 10;
  float distance_front = 10;
  float distance_front_left = 10;
  float distance_front_right = 10;

  const float side_avoid_limit = 0.05;
  const float side_avoid_track_radius = 0.15;
  const float side_avoid_detect_radius = 0.5;

  const float front_avoid_limit = 0.1;
  const float front_avoid_detect_radius = 0.5;
  
  const float front_side_avoid_detect_radius = 0.4;

// DEPTH SURVEY
  float depth = 0;
  float depth_confidence = 0;

// UDP WIFI
  const char* ssid = "MERCURY_2.4G_0122";
  const char* password = "hch4pw7n";
  //const char* ssid = "Dung Minh1906";
  //const char* password = "dungminh";
  // const char* ssid = "TAIH";
  // const char* password = "hmt12345";
  // const char* ssid = "Linh";
  // const char* password = "sinhnhatcuatao";
  // const char* ssid = "POCO F3";
  // const char* password = "12345789";
  // const char* ssid = "TP-LINK_3D77A0";
  // const char* password = "";
  // const char* ssid = "KTHK 207";
  // const char* password = "hmt123456789xyz";
  // const char* ssid = "HUAWEI_ADDD";
  // const char* password = "YG0A94HM7B2";
  //const char* ssid = "Hieu Thao 2,4g";
  //const char* password = "0944012555";
  //const char* ssid = "Hieu Thao 5g";
  //const char* password = "0944012555";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back


// TRACKING PARAMETERS
  float distance_12 = 0;
  float zigzag_step = 2.5;
  int step_no = 0;
  
  int current_target = 0;
  bool switch_target = 0;
  double arrival_start_time = 0;
  bool end_tracking = 0;
  bool reach_A = 0;
  bool get_initial = 0;
  
  double target_lat = 0;
  double target_lng = 0;
  double target_lat1 = 0;
  double target_lng1 = 0;
  double target_lat_1 = 0;
  double target_lng_1 = 0;
  double target_lat_2 = 0;
  double target_lng_2 = 0;
  double target_lat_3 = 0;
  double target_lng_3 = 0;
  double target_lat_4 = 0;
  double target_lng_4 = 0;
  double usv_lat = 0;
  double usv_lng = 0;
  double initial_lat = 0;
  double initial_lng = 0;
  bool valid = 0;
  double temp;
  int temp_3;
  int i;

// CONTROL PARAMETER
  float usv_yaw = 0;
  float usv_yaw_prev = 0;
  
  float fix_lat_error = 0;
  float fix_lng_error = 0;
  float yaw_error = 0;
  float desire_yaw = 0;
  float distance = 0;
  float k_yaw_error = 0;
  float k_distance = 0;
  bool arrival = 0;
  bool arrival_prev = 0;
  bool arrival_prev_prev = 0;
  float temp2;
  float fuzzy_low = 0;
  float fuzzy_mid = 0;
  float fuzzy_high = 0;

  const float pi = 3.14159265359;
  const float k_fix_lat = 113894;
  const float k_fix_lng = 104225;
  float a = 2;
  float b = 20;
  float k_moment = 175/pi;
  const float k_moment_2 = 300/pi;
  
  int cruise = 190;
  float arrival_radius = 3;
  float look_ahead = 5;

  IPAddress laptop(192,168,1,38); 
  IPAddress controller(192,168,1,52);
  IPAddress observer(192,168,1,51);
  IPAddress observer_2(192,168,1,61);
  IPAddress observer_3(192,168,1,71);

  IPAddress gateway(192, 168, 1, 1);  
  IPAddress subnet(255, 255, 0, 0);
  IPAddress primaryDNS(8, 8, 8, 8);   //optional
  IPAddress secondaryDNS(8, 8, 4, 4); //optional

  // Motot pin
  Servo f_1;
  Servo f_2;

//----------------------------------------------------------------------------    SETUP   -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 
void setup() {
  Serial.begin(115200);

  // Configures static IP address
  if (!WiFi.config(controller, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Udp.begin(4210);

  // PWM
  //f_1.attach(pin_motor_1,1000,2000); 
  pinMode(pin_motor_1, OUTPUT);     
  f_2.attach(pin_motor_2,1000,2000);
  //pinMode(pin_test, OUTPUT);   

  analogWriteFreq(15000);
}

//-----------------------------------------------------------------------------   LOOP    -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***-----------------------------------------------------------------------------
void loop(){

  // ----------------------------------------------- READING ----------------------------------------------- 

  // --------------------- UDP packet from Observers ---------------------
  receive_udp();

  // --------- Observer 3 ---------
  for (int i= 1; i<5; i++)
  {
    if (Udp.remoteIP()==observer_3)
    {
      distance_front_left = parse_target_lat(1);
      distance_front_right = parse_target_lat(16);
      depth = parse_target_lat(31);
      depth_confidence = parse_integer_3_digit(46);

      break;
    }
    receive_udp();
  }

  // --------- Observer 2 ---------
  for (int i= 1; i<5; i++)
  {
    if (Udp.remoteIP()==observer_2)
    {     
      distance_front = parse_target_lat(1);
      distance_left = parse_target_lat(16);
      distance_right = parse_target_lat(31);

      break;
    }
    receive_udp();
  }

  // --------- Observer 1 ---------
  for (int i= 1; i<5; i++)
  {
    if (Udp.remoteIP()==observer)
    {     
      usv_lat = parse_target_lat(6);
      usv_lng = parse_target_lng(6+2);
      usv_yaw = parse_integer_3_digit(58);
      valid = incomingPacket[44] - 48;
      voltage_bat = parse_target_lat(74);

      break;
    }
    receive_udp();
  }

  // --------------------- UDP packet from GCS ---------------------
  for (int i= 1; i<5; i++)
  { 
    if (Udp.remoteIP() == laptop)
    {
      Serial.print("Laptop:  ");
      Serial.print(incomingPacket);
      Serial.print("; ");

      if (incomingPacket[0] == 63)
      {
        test_mode = true;
        test_speed = parse_integer_3_digit(2);
      }
      Serial.print("Test mode:  ");
      Serial.print(test_mode);
      Serial.print("; ");
  
      if (incomingPacket[0] == 94)
      {
        Udp.beginPacket(laptop,1403);
        udp_send(usv_lat,2,9);
        udp_send(usv_lng,3,9);
        Udp.endPacket();
      }
      
      // ------------------------------------------ SETTING UP MODES ------------------------------------------
      // MODE 0: MANUAL MODE
      if (incomingPacket[0] == 35)
      {
        auto_mode = 0;
        discrete_control = parse_integer_3_digit(3);
        discrete_moment = parse_integer_3_digit(8);
        
        //desire_yaw = parse_integer_3_digit(8);
        //if (desire_yaw > 180)
        //  desire_yaw = map_float(desire_yaw, 180, 360, -180, 0);
        //desire_yaw = map_float(desire_yaw, -180, 180, -pi, pi);
      }
  
      // MODE 1: SINGLE TARGET MODE
      if (incomingPacket[0] == 64)
      {
        auto_mode = 1;
        select_auto_mode = 1;

        end_tracking = 0;
        reach_A = 0;
        
        target_lat = parse_target_lat(2);
        target_lng = parse_target_lng(2);

        initial_lat = usv_lat;
        initial_lng = usv_lng; 
      }
      
      // MODE 2: ZIGZAC MODE
      if (incomingPacket[0] == 33)
      {
        auto_mode = 1;
        select_auto_mode = 2;
        
        // Reset zigzag
        end_tracking = 0;
        reach_A = 0;
        arrival_start_time = millis();
        current_target = 1;
        
        target_lat_1 = parse_target_lat(2);
        target_lng_1 = parse_target_lng(2);
  
        target_lat_2 = parse_target_lat(29);
        target_lng_2 = parse_target_lng(29);
  
        target_lat_3 = parse_target_lat(56);
        target_lng_3 = parse_target_lng(56);
  
        target_lat_4 = parse_target_lat(83);
        target_lng_4 = parse_target_lng(83);
  
        target_lat = target_lat_1;
        target_lng = target_lng_1;

        initial_lat = usv_lat;
        initial_lng = usv_lng;  
  
        // CALCULATE ZIGZAG STEP
        distance_12 = get_distance (target_lat_1, target_lat_2, target_lng_1, target_lng_2);
        step_no = get_step_no(distance_12, zigzag_step);
      }

      // MODE 3: AB LINE
      if (incomingPacket[0] == 62)
      {
        auto_mode = 1;
        select_auto_mode = 3;
        arrival = 0;
        reach_A = 0;
        arrival_start_time = millis();

        target_lat_1 = parse_target_lat(2);
        target_lng_1 = parse_target_lng(2);
  
        target_lat_2 = parse_target_lat(29);
        target_lng_2 = parse_target_lng(29);

        target_lat = target_lat_1;
        target_lng = target_lng_1;

        initial_lat = usv_lat;
        initial_lng = usv_lng;
      }
  
      // SETTING PARAMETERS
      if (incomingPacket[0] == 37)
      {
        a = parse_integer_3_digit(3);
        b = parse_integer_3_digit(8);
        k_moment = parse_integer_3_digit(13)/pi;
        cruise = parse_integer_3_digit(18);
        arrival_radius = parse_integer_3_digit(23);
        zigzag_step = parse_integer_3_digit(28)/10;
        look_ahead = parse_integer_3_digit(33);
      }
      break;
    }
    receive_udp();
  }

  Serial.print("Auto = ");
  Serial.print(auto_mode);
  Serial.print("; "); 

  Serial.print("Packet = ");
  Serial.print(incomingPacket);
  Serial.print("; "); 

  // ------------------------------------------ TRACKING LOGIC ------------------------------------------
  // ------------ CHOOSE TARGET ------------

  // -------- MODE 2: ZIGZAG --------
  if ((auto_mode>0)&&(select_auto_mode == 2))
  { 
    if (current_target >= ((step_no+1)*2))
      end_tracking = true;
    Serial.print("end_tracking =  ");
    Serial.print(end_tracking);
    Serial.print("; ");  
      
    if ((arrival_prev>0)&&(arrival_prev_prev<1))
      arrival_start_time = millis();
  
    if (((millis()-arrival_start_time) > 100)&&(arrival_prev>0))
    {
      arrival_start_time = millis();
      switch_target = true;
    }

    // Jigsaw path option
//    if (((current_target%4)==0)||((current_target%4)==4))
//    {
//      arrival_start_time = millis();
//      switch_target = true;
//    }
    // end of Jigsaw path option
  
    if ((switch_target>0)&&(end_tracking<1))
    {
      switch_target = 0;
      current_target = current_target +1;
      
      initial_lat = target_lat;
      initial_lng = target_lng;
    }
    Serial.print("current_target =  ");
    Serial.print(current_target);
    Serial.print("; "); 
    
    target_lat = get_target_list(current_target, step_no, target_lat_1, target_lat_2, target_lat_3, target_lat_4);
    target_lng = get_target_list(current_target, step_no, target_lng_1, target_lng_2, target_lng_3, target_lng_4);
    
    switch_target = 0;
  }

  // -------- MODE 3: AB LINE --------
  if ((auto_mode>0)&&(select_auto_mode == 3))
  {
    if ((reach_A<1)&&(arrival<1))
    {
      target_lat = target_lat_1;
      target_lng = target_lng_1;
    }
    
    if ((reach_A<1)&&(arrival>0))
    {
      reach_A = 1; 
    }
    
    if ((reach_A>0)&&(arrival<1))
    {
      initial_lat = target_lat_1;
      initial_lng = target_lng_1;

      target_lat = target_lat_2;
      target_lng = target_lng_2;
    }
    
    if ((reach_A>0)&&(arrival<1))
    {
      end_tracking = 1;
    }
  }
  
// ----------------------------------------------- GET YAW ERROR, DISTANCE -----------------------------------------------
//
  Serial.print("target_lat =  ");
  Serial.print(target_lat,6);
  Serial.print("; ");
  
  Serial.print("target_lng =  ");
  Serial.print(target_lng,6);
  Serial.print("; "); 

  Serial.print("yaw =  ");
  Serial.print(usv_yaw,0);
  Serial.print("; "); 
  usv_yaw = map_float(usv_yaw, -180, 180, -pi, pi);
  
//  if ((usv_yaw<(pi/180)*3.5)&&(usv_yaw>(-pi/180)*3.5))
//  {
//    usv_yaw = usv_yaw_prev;
//  }
  
  temp2 = map_float(usv_yaw, -pi, pi, -180, 180);  
  Serial.print("yaw =  ");
  Serial.print(temp2,0);
  Serial.print("; "); 

  fix_lat_error = k_fix_lat * (target_lat - usv_lat); 
  fix_lng_error = k_fix_lng * (target_lng - usv_lng);
  
  // ------------ SET DESIRE YAW ------------
  if (auto_mode > 0)
  {
    temp2 = atan2(fix_lng_error, fix_lat_error);                                                                     // range -pi ~ pi
    desire_yaw = get_look_ahead_yaw(usv_lat, usv_lng, initial_lat, initial_lng, target_lat, target_lng);             // range -pi ~ pi
    if (abs(temp2-desire_yaw)>(pi/2))
    {
      desire_yaw = temp2;
    }
  }

  desire_yaw = wrap_pi(desire_yaw);
  temp2 = map_float(desire_yaw, -pi, pi, -180, 180);      
  Serial.print("yaw_d: ");
  Serial.print(temp2, 0);
  Serial.print("; "); 

  yaw_error = desire_yaw - usv_yaw;                             // range -2pi ~ 2pi

// ------------FIX YAW ERROR------------
  yaw_error = wrap_pi(yaw_error);                               // range -pi ~ pi
  temp2 = map_float(yaw_error, -pi, pi, -180, 180);      
  Serial.print("yaw_e: ");
  Serial.print(temp2, 0);
  Serial.print("; "); 

// ------------GET DISTANCE------------
  distance = pow( (pow(fix_lat_error,2) + pow(fix_lng_error,2)), 0.5);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(", ");

// ----------------------------------------------- CONTROL FACTORS -----------------------------------------------    
  k_yaw_error = 1 / (1 + a*pow(yaw_error,2));                   // range 1 ~ 0
  k_distance = pow(distance,2)/(pow(distance,2) + b);           // range 0 ~ 1
  Serial.print("k_yaw_e: ");
  Serial.print(k_yaw_error, 3);
  Serial.print("; "); 
  Serial.print("k_dist: ");
  Serial.print(k_distance, 3);
  Serial.print("; ");
  fuzzy_low = fuzzy_member(1, yaw_error, -pi/8, 0, pi/8);
  fuzzy_mid = fuzzy_member(2, yaw_error, -pi/8, 0, pi/8);
  fuzzy_high = fuzzy_member(3, yaw_error, -pi/8, 0, pi/8);

  // ------------TARGET REACHED------------
  arrival = 0;
  if (distance < arrival_radius)
  {
    arrival = true;
  }
  Serial.print("arrival: ");
  Serial.print(arrival, 1);
  Serial.print("; "); 

// ----------------------------------------------- THRUST AND MOMENT -----------------------------------------------
  // MODE 0: Manual
  if (auto_mode < 1)
  {
    if (discrete_control == 10)
      thrust = 180;
    if ((discrete_control == 1)||(discrete_control == 100))
      thrust = 0;
      
    moment = 0;
    if (discrete_moment == 1)
      moment = 90;
    if (discrete_moment == 100)
      moment = -90;
    
    //moment = k_moment * yaw_error;

    if ((discrete_control!=10)&&(discrete_control!=100)&&(discrete_control!=1))
    {
      thrust = 0;
      moment = 0;
    }
  }
  else
  {
    // (MODE 1: Single target) + (MODE 2: Zigzag) + (MODE 3: AB line)
    if ((arrival > 0)||(end_tracking > 0))
    {
      thrust = 0;
      moment = 0;
    }
    else
    {
      //thrust = k_yaw_error * k_distance * cruise;                   // range 0 ~ cruise
      thrust = k_distance * cruise;
      moment = k_moment * yaw_error;
      // moment = ((fuzzy_low * k_moment_2) + (fuzzy_mid * k_moment) + (fuzzy_high * k_moment_2)) * yaw_error;
    }
  }
    
    
// ----------------------------- Control PWM -----------------------------
  Serial.print("thrust: ");
  Serial.print(thrust);
  Serial.print("; ");
  Serial.print("moment: ");
  Serial.print(moment);
  Serial.print("; ");

  speed_1 = thrust; // PWM output, range 0-255
  speed_2 = -moment;
  speed_2 = map_float(speed_2,-130,130,0,180);
  if (speed_1>120) speed_1 = 120;
  if (speed_2>190) speed_2 = 190;
  if (speed_1<0) speed_1 = 0;
  if (speed_2<0) speed_2 = 0;
  Serial.print("Motor: ");
  Serial.print(speed_1);
  Serial.print(" ");
  Serial.print(speed_2);
  Serial.print("; ");

  // PWM
  if (speed_1<80)
    digitalWrite(pin_motor_1, LOW);
  else
    digitalWrite(pin_motor_1, HIGH);
  f_2.write(speed_2);
  analogWrite(pin_test, 50);
  
// ---------------- UDP SEND LOG DATA TO PACKET SENDER ----------------

  Udp.beginPacket(laptop, 4210);

  if(test_mode == true)
  {
    Udp.write("Test speed: ");
    udp_send(test_speed, 3, 0);
  }

  // Mode
  Udp.write("Auto: ");
  udp_send(auto_mode, 1, 0);
  Udp.write("Mode: ");
  udp_send(select_auto_mode, 1, 0);

  // Thrust Moment
  Udp.write("Thrust, Moment: ");
  udp_send(thrust,3,0);
  udp_send(moment,3,0);

  // Motor
  Udp.write("Motor: ");
  udp_send(speed_1, 3, 0);
  udp_send(speed_2, 3 ,0);

  // Compass
  temp2 = map_float(usv_yaw, -pi, pi, -180, 180);
  Udp.write("Yaw");
  udp_send(temp2,3,0);

  // Desire yaw
  temp2 = map_float(desire_yaw, -pi, pi, -180, 180);
  Udp.write("Desire yaw: ");
  udp_send(temp2, 3, 0);

  // Yaw error
  temp2 = map_float(yaw_error, -pi, pi, -180, 180);
  Udp.write("Yaw error: ");
  udp_send(temp2, 3, 0);

  // Distance
  Udp.write("Distance: ");
  udp_send(distance, 4, 0);

  //Arrival
  Udp.write("Arrival: ");
  udp_send(arrival, 1, 0);
  if (select_auto_mode == 2)
  {
    // Target list
    Udp.write("List: ");
    udp_send(step_no, 3,0);
    udp_send(current_target,3,0);
    // End zigzag
    Udp.write("End: ");
    udp_send(end_tracking, 1, 0);
  }
  if (select_auto_mode == 3)
  {
    Udp.write("Reach A: ");
    udp_send(reach_A, 1, 0);
  }

  // USV coordinates
  Udp.write("USV: ");
  udp_send(usv_lat,2,9);
  udp_send(usv_lng,3,9);
  Udp.write("Valid: ");
  udp_send(valid,1,0);

  // TARGET
  Udp.write("Initial: ");
  udp_send(initial_lat, 2, 9);
  udp_send(initial_lng, 3, 9);

  Udp.write("Target: ");
  udp_send(target_lat, 2, 9);
  udp_send(target_lng, 3, 9);

  // k_yaw
  Udp.write("k_yaw: ");
  udp_send(k_yaw_error, 1, 3);

  // k_distance
  Udp.write("k_distance: ");
  udp_send(k_distance, 1, 3);

  /*
  // Fuzzy
  Udp.write("fuzz_h: ");
  udp_send(fuzzy_high, 1, 3);
  Udp.write("fuzz_m: ");
  udp_send(fuzzy_mid, 1, 3);
  Udp.write("fuzz_l: ");
  udp_send(fuzzy_low, 1, 3);
  
  // k_moment_fuzzy
  temp2 = ((fuzzy_low * k_moment_2) + (fuzzy_mid * k_moment) + (fuzzy_high * k_moment_2))*pi;
  Udp.write("k_moment_fuzzy: ");
  udp_send(temp2, 3, 0);
  */

  // Tracking parameters
  Udp.write("a: ");
  udp_send(a, 2, 0);
  Udp.write("b: ");
  udp_send(b, 2, 0);
  
  Udp.write("k_moment: ");
  temp2 = k_moment*pi;
  udp_send(temp2, 3, 0);
  Udp.write("cruise: ");
  udp_send(cruise, 3, 0);
  
  Udp.write("arrival_radius: ");
  udp_send(arrival_radius, 1, 1);
  Udp.write("zigzag_step: ");
  udp_send(zigzag_step, 1, 1);

  Udp.write("look_ahead: ");
  udp_send(look_ahead, 2, 0);

  // Battery voltage
  Udp.write("Voltage: ");
  udp_send(voltage_bat, 2, 2);

  // Obstacle
  Udp.write("obstacle_scenario: ");
  udp_send(obstacle_scenario, 1, 0);
  Udp.write("obstacle_side: ");
  udp_send(obstacle_side, 1, 0);
  Udp.write("front: ");
  udp_send(distance_front, 2, 1);
  Udp.write("left: ");
  udp_send(distance_left, 2, 1);
  Udp.write("right: ");
  udp_send(distance_right, 2, 1);
  Udp.write("front_left: ");
  udp_send(distance_front_left, 2, 1);
  Udp.write("front_right: ");
  udp_send(distance_front_right, 2, 1);

  // Depth survey
  Udp.write("depth: ");
  udp_send(depth, 2, 3);
  Udp.write("depth_confidence: ");
  udp_send(depth_confidence, 3, 0);
  
  // END OF PACKET
  Udp.write(terminate_char);
  Udp.endPacket();

// ---------------- UDP SEND LOG DATA TO CONTROL INTERFACE ----------------
  Udp.beginPacket(laptop, 1403);

  // Mode
  udp_send(auto_mode, 1, 0);
  udp_send(select_auto_mode, 1, 0);

  // Thrust Moment
  udp_send(thrust,3,0);
  udp_send(moment,3,0);

  // Motor
  udp_send(speed_1, 3, 0);
  udp_send(speed_2, 3 ,0);

  // Compass
  temp2 = map_float(usv_yaw, -pi, pi, -180, 180);
  udp_send(temp2,3,0);

  // Desire yaw
  temp2 = map_float(desire_yaw, -pi, pi, -180, 180);
  udp_send(temp2, 3, 0);

  // Yaw error
  temp2 = map_float(yaw_error, -pi, pi, -180, 180);
  udp_send(temp2, 3, 0);

  // Distance
  udp_send(distance, 4, 0);

  //Arrival
  udp_send(arrival, 1, 0);
  udp_send(step_no, 3,0);
  udp_send(current_target,3,0);
  udp_send(end_tracking, 1, 0);
  udp_send(reach_A, 1, 0);

  // USV coordinates
  udp_send(usv_lat,2,9);
  udp_send(usv_lng,3,9);
  udp_send(valid,1,0);

  // TARGET
  udp_send(target_lat, 2, 9);
  udp_send(target_lng, 3, 9);

  // k_yaw
  udp_send(k_yaw_error, 1, 3);

  // k_distance
  udp_send(k_distance, 1, 3);

  // Tracking parameters
  udp_send(a, 2, 0);
  udp_send(b, 2, 0);
  
  temp2 = k_moment*pi;
  udp_send(temp2, 3, 0);
  udp_send(cruise, 3, 0);
  
  udp_send(arrival_radius, 1, 1);
  udp_send(zigzag_step, 1, 1);

  udp_send(look_ahead, 2, 0);

  // Battery voltage
  udp_send(voltage_bat, 2, 2);

  // Obstacle
  udp_send(obstacle_scenario, 1, 0);
  udp_send(obstacle_side, 1, 0);
  udp_send(distance_front, 2, 1);
  udp_send(distance_left, 2, 1);
  udp_send(distance_right, 2, 1);
  udp_send(distance_front_left, 2, 1);
  udp_send(distance_front_right, 2, 1);

  // Depth survey
  udp_send(depth, 2, 3);
  udp_send(depth_confidence, 3, 0);
  
  // END OF PACKET
  Udp.write(terminate_char);
  Udp.endPacket();

// ---------------------END LOOP---------------------
  Serial.println();
  arrival_prev_prev = arrival_prev;
  arrival_prev = arrival;
  usv_yaw_prev = usv_yaw;
  usv_yaw = map_float(usv_yaw, -pi, pi, -180, 180);
}

//----------------------------------------------------------------------------- FUNCTIONS ----------------------------------------------------------------------------- 
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 

void udp_send (double data, int top_digit, int lowest_decimal)
{
  if (data<0)
  {
    data = abs(data);
    Udp.write(45);
  }
  else Udp.write(43);
  
  int string_size = top_digit + lowest_decimal + 1;
  char udp_string [string_size];
  temp = data;
  
  for (int i = top_digit; i>=1; i--)
  {
   udp_string[top_digit-i] = temp/pow(10,(i-1));
   temp = temp - udp_string[top_digit-i] * pow(10,(i-1));
   udp_string[top_digit-i] = udp_string[top_digit-i] +48;
  }

  udp_string[top_digit] = 46;

  for (int i = 1; i <= lowest_decimal; i++)
  {
   udp_string[top_digit+i] = temp/pow(10,-i);
   temp = temp - udp_string[top_digit+i] * pow(10,-i);
   udp_string[top_digit+i] = udp_string[top_digit+i] +48;
  }

  for (int i=0; i<= (string_size-1); i++)
  {
    Udp.write(udp_string[i]);
  }
   
  Udp.write(end_char);
}

float map_float( float val, float min1, float max1, float min2, float max2)
{
  float val_2;
  if (val<min1)
    val = min1;
  if (val>max1)
    val = max1; 
  val_2 = min2 + (val-min1)*(max2-min2)/(max1-min1);
  return val_2;
}

double parse_target_lat (int _start_char_no)
{     
      double _target_lat;
      int _k = 1;
      _target_lat = 0;
      _k = _start_char_no+0;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+0]-48)*1E1;
      _k = _start_char_no+1;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+1]-48)*1E0;
      _k = _start_char_no+3;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+3]-48)*1E-1;
      _k = _start_char_no+4;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+4]-48)*1E-2;
      _k = _start_char_no+5;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+5]-48)*1E-3;
      _k = _start_char_no+6;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+6]-48)*1E-4;
      _k = _start_char_no+7;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+7]-48)*1E-5;
      _k = _start_char_no+8;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+8]-48)*1E-6;
      _k = _start_char_no+9;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+9]-48)*1E-7;
      _k = _start_char_no+10;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+10]-48)*1E-8;
      _k = _start_char_no+11;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+11]-48)*1E-9;

      return _target_lat;    
}



double parse_target_lng (int _start_char_no)
{
      double _target_lng;
      int _k = 2;
      _target_lng = 0;
      _k = _start_char_no + 13;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E2;
      _k = _start_char_no+14;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E1;
      _k = _start_char_no+15;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E0;
      _k = _start_char_no+17;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-1;
      _k = _start_char_no+18;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-2;
      _k = _start_char_no+19;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-3;
      _k = _start_char_no+20;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-4;
      _k = _start_char_no+21;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-5;
      _k = _start_char_no+22;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-6;
      _k = _start_char_no+23;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-7;
      _k = _start_char_no+24;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-8;
      _k = _start_char_no+25;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-9;

      return _target_lng;
}

float get_distance (double _target_lat_1, double _target_lat_2, double _target_lng_1, double _target_lng_2)
{
  float _distance;
  _distance = pow(pow(k_fix_lat*(_target_lat_2 - _target_lat_1) , 2) + pow(k_fix_lng*(target_lng_1 - target_lng_2) , 2) ,0.5);
  return _distance;
}

int get_step_no (float _distance, float _step_size)
{
  int _step_no;
  _step_no = _distance/_step_size;
  return _step_no;
}

double get_target (int i, int _step_no, double _target_1, double _target_2)
{
  double _target;
  double _step;

  _step = (_target_2 - _target_1)/_step_no;
  _target = _target_1 + _step*i;

  return _target;
}

double get_target_list (int i, int _step_no, double _target_1, double _target_2, double _target_3, double _target_4)
{
  double _target_list = 0;
  int k_int = 0;
  int mod_4 = 0;
  k_int = i/4;
  mod_4 = i%4;

  if (mod_4 == 0)
    _target_list = get_target((2*k_int), _step_no, _target_1, _target_2);

  if (mod_4 == 1)
    _target_list = get_target((2*k_int+1), _step_no, _target_1, _target_2);

  if (mod_4 == 2)
    _target_list = get_target((2*k_int), _step_no, _target_4, _target_3);

  if (mod_4 == 3)
    _target_list = get_target((2*k_int+1), _step_no, _target_4, _target_3);
  
  return _target_list;
}

float parse_integer_3_digit (int _start_char_no)
{
  float _val;
  int _k;
  _val = 0;
  _k = _start_char_no;
  _val = _val + (incomingPacket[_k]-48)*1E2;
  _k = _start_char_no+1;
  _val = _val + (incomingPacket[_k]-48)*1E1;
  _k = _start_char_no+2;
  _val = _val + (incomingPacket[_k]-48)*1E0;

  if((int)incomingPacket[_start_char_no - 1] == 45) 
    _val=-_val;
  
  return _val;
}
float fuzzy_member (int _level, float _in, float _in_1, float _in_2, float _in_3)
{
  float _member_deg;
  
  if (_level == 1)
  {
    if (_in < _in_1)
      _member_deg  = 1;
    if ((_in_1 <= _in)&&(_in <= _in_2))
      _member_deg = map_float (_in, _in_1, _in_2, 1, 0); 
    if (_in_2 < _in)
      _member_deg = 0;
  }
  
  if (_level == 2)
  {
    if ((_in_1 <= _in)&&(_in <= _in_2))
      _member_deg = map_float (_in, _in_1, _in_2, 0, 1); 
    if ((_in_2 < _in)&&(_in <= _in_3))
      _member_deg = map_float (_in, _in_2, _in_3, 1, 0);
    if ((_in < _in_1)||(_in > _in_3))
      _member_deg = 0;
  }
    
  if (_level == 3)
  {
    if (_in < _in_2)
      _member_deg = 0; 
    if ((_in_2 <= _in)&&(_in <= _in_3))
      _member_deg = map_float (_in, _in_2, _in_3, 0, 1);
    if (_in > _in_3)
      _member_deg = 1;
      
  }
  return _member_deg;
}

void receive_udp()
{
  int packetSize = Udp.parsePacket();
  int len = Udp.read(incomingPacket, 255);
  if (len > 0)
    incomingPacket[len] = 0;
}

float wrap_pi(float _angle)
{
  while (_angle > pi)
    _angle = _angle - 2*pi;
  while (_angle < -pi)
    _angle = _angle + 2*pi;
  return _angle;
}

float delta_2(float _a, float _b)
{
  float _delta_2;
  _delta_2 = pow((pow(_a,2)+pow(_b,2)),0.5);
  return _delta_2;
}

float get_look_ahead_yaw(double _usv_lat, double _usv_lng, double _target_lat_A, double _target_lng_A, double _target_lat_B, double _target_lng_B)
{
  double _a_AB;
  double _b_AB;
  double _c_AB;
  double _a_MH;
  double _b_MH;
  double _c_MH;
  double _x_H;
  double _y_H;
  double _det;
  float _desire_yaw;

  _b_MH = k_fix_lat * (_target_lat_B - _target_lat_A); 
  _a_MH = k_fix_lng * (_target_lng_B - _target_lng_A);

  _a_AB = -_b_MH;
  _b_AB = _a_MH;

  _c_AB = _a_AB * (k_fix_lng * _target_lng_A) + _b_AB * (k_fix_lat * _target_lat_A);
  _c_MH = _a_MH * (k_fix_lng * _usv_lng) + _b_MH * (k_fix_lat * _usv_lat);

  _det = _a_AB*_b_MH - _b_AB*_a_MH;
  _x_H = (_c_AB*_b_MH - _b_AB*_c_MH)/_det;
  _y_H = (_a_AB*_c_MH - _c_AB*_a_MH)/_det;

  _x_H = _x_H + (_a_MH/delta_2(_a_MH,_b_MH))*look_ahead;
  _y_H = _y_H + (_b_MH/delta_2(_a_MH,_b_MH))*look_ahead;

  _desire_yaw = atan2( _x_H - (k_fix_lng*usv_lng), _y_H - (k_fix_lat*usv_lat));

  return _desire_yaw;
}
