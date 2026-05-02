// #include <Arduino.h>
// /////////////////////
// /// For Micro ROS ///
// /////////////////////
// #include <micro_ros_platformio.h>
// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <rmw_microros/rmw_microros.h>

// // library IMU
// #include <REG.h>
// #include <wit_c_sdk.h>

// #define ANGLE_UPDATE	0x04
// static volatile char s_cDataUpdate = 0;

// static void AutoScanSensor(void);
// static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
// static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
// static void Delayms(uint16_t ucMs);
// const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};

// static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
// {
//   Serial6.write(p_data, uiSize);
//   Serial6.flush();
// }
// static void Delayms(uint16_t ucMs)
// {
//   delay(ucMs);
// }
// static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
// {
// 	int i;
//     for(i = 0; i < uiRegNum; i++)
//     {
//         switch(uiReg)
//         {
//             case Yaw:
//                 s_cDataUpdate |= ANGLE_UPDATE;
//                 break;
//         }
// 		uiReg++;
//     }
// }

// static void AutoScanSensor(void)
// {
// 	int i, iRetry;
	
// 	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
// 	{
// 		Serial6.begin(c_uiBaud[i]);
//     Serial6.flush();
// 		iRetry = 2;
// 		s_cDataUpdate = 0;
// 		do
// 		{
// 			WitReadReg(AX, 3);
// 			delay(200);
//       while (Serial6.available())
//       {
//         WitSerialDataIn(Serial6.read());
//       }
// 			if(s_cDataUpdate != 0)
// 			{
// 				Serial.print(c_uiBaud[i]);
// 				Serial.print(" baud find sensor\r\n\r\n");
// 				return ;
// 			}
// 			iRetry--;
// 		}while(iRetry);		
// 	}
// 	Serial.print("can not find sensor\r\n");
// 	Serial.print("please check your connection\r\n");
// }

// /*
//  * TODO : include your desired msg header file
// */
// #include <auv_interfaces/msg/sensor.h>

// /*
//  * Helper functions to help reconnect
// */
// #define EXECUTE_EVERY_N_MS(MS, X)  do { \
//     static volatile int64_t init = -1; \
//     if (init == -1) { init = uxr_millis();} \
//     if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
//   } while (0)

// enum states {
//   WAITING_AGENT,
//   AGENT_AVAILABLE,
//   AGENT_CONNECTED,
//   AGENT_DISCONNECTED
// } state;

// /*
//  * Declare rcl object
// */
// rclc_support_t support;
// rcl_init_options_t init_options;
// rcl_node_t node;
// rcl_timer_t timer;
// rclc_executor_t executor;
// rcl_allocator_t allocator;

// /*
//  * TODO : Declare your 
//  * publisher & subscription objects below
// */
// rcl_publisher_t pub_sensor;


// /*
//  * TODO : Define your necessary Msg
//  * that you want to work with below.
// */
// auv_interfaces__msg__Sensor sensor_msg;

// void run_control_loop();

// /*
//  * TODO : Define your subscription callbacks here
//  * leave the last one as timer_callback()
// */
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
//   (void) last_call_time;
//   if (timer != NULL) {

//     run_control_loop();
//     /*
//        TODO : Publish anything inside here
       
//        For example, we are going to echo back
//        the int16array_sub data to int16array_pub data,
//        so we could see the data reflect each other.
//        And also keep incrementing the int16_pub
//     */

//     rcl_publish(&pub_sensor, &sensor_msg, NULL);
//   }
// }

// /*
//    Create object (Initialization)
// */
// bool create_entities()
// {
//   /*
//      TODO : Define your
//      - ROS node name
//      - namespace
//      - ROS_DOMAIN_ID
//   */
//   const char * node_name = "teensy_node";
//   const char * ns = "";
//   const int domain_id = 0;
  
//   /*
//    * Initialize node
//    */
//   allocator = rcl_get_default_allocator();
//   init_options = rcl_get_zero_initialized_init_options();
//   rcl_init_options_init(&init_options, allocator);
//   rcl_init_options_set_domain_id(&init_options, domain_id);
//   rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
//   rclc_node_init_default(&node, node_name, ns, &support);

  
//   /*
//    * TODO : Init your publisher and subscriber 
//    */
//   rclc_publisher_init(
//     &pub_sensor,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(auv_interfaces, msg, Sensor),
//     "sensor_pub", &rmw_qos_profile_default);

//   /*
//    * Init timer_callback
//    * TODO : change timer_timeout
//    * 50ms : 20Hz
//    * 20ms : 50Hz
//    * 10ms : 100Hz
//    */
//   const unsigned int timer_timeout = 50;
//   rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

//   /*
//    * Init Executor
//    * TODO : make sure the num_handles is correct
//    * num_handles = total_of_subscriber + timer
//    * publisher is not counted
//    * 
//    * TODO : make sure the name of sub msg and callback are correct
//    */
//   unsigned int num_handles = 1;
//   executor = rclc_executor_get_zero_initialized_executor();
//   rclc_executor_init(&executor, &support.context, num_handles, &allocator);
//   rclc_executor_add_timer(&executor, &timer);

//   return true;
// }
// /*
//  * Clean up all the created objects
//  */
// void destroy_entities()
// {
//   rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
//   (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

//   rcl_timer_fini(&timer);
//   rclc_executor_fini(&executor);
//   rcl_init_options_fini(&init_options);
//   rcl_node_fini(&node);
//   rclc_support_fini(&support);
//   /*
//    * TODO : Make sue the name of publisher and subscriber are correct
//    */
//   rcl_publisher_fini(&pub_sensor, &node);
  
// }

// void setup() {
//   /*
//    * TODO : select either of USB or WiFi 
//    * comment the one that not use
//    */
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

//   Serial6.begin(115200);
// 	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
// 	WitSerialWriteRegister(SensorUartSend);
// 	WitRegisterCallBack(SensorDataUpdata);
//   WitDelayMsRegister(Delayms);
// 	AutoScanSensor();

//   /*
//    * TODO : Initialze the message data variable
//    */
//   auv_interfaces__msg__Sensor__init(&sensor_msg);

//   /*
//    * Setup first state
//    */
//   state = WAITING_AGENT;

// }

// void run_control_loop() {
//   while (Serial6.available())
//   {
//     WitSerialDataIn(Serial6.read());
//   }

//   if(s_cDataUpdate & ANGLE_UPDATE)
//   {
//     sensor_msg.yaw = sReg[Yaw] / 32768.0f * 180.0f;
//     sensor_msg.pitch = sReg[Roll] / 32768.0f * 180.0f;
//     sensor_msg.roll = -sReg[Pitch] / 32768.0f * 180.0f;
//     s_cDataUpdate &= ~ANGLE_UPDATE;
//   }
// }

// void loop() {
//   /*
//    * Try ping the micro-ros-agent (HOST PC), then switch the state 
//    * from the example
//    * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
//    * 
//    */
//   switch (state) {
//     case WAITING_AGENT:
//       EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
//       break;
//     case AGENT_AVAILABLE:
//       state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//       if (state == WAITING_AGENT) {
//         destroy_entities();
//       };
//       break;
//     case AGENT_CONNECTED:
//       EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
//       if (state == AGENT_CONNECTED) {
//         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//       }
//       break;
//     case AGENT_DISCONNECTED:
//       destroy_entities();
//       state = WAITING_AGENT;
//       break;
//     default:
//       break;
//   }
// }