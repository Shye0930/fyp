#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>

#define LED_PIN 23
#define LED_PIN_TEST 18
#define MOTOR_PWM_PIN 18 // PWM pin for motor control
bool led_test_state = false;

bool motor_state = false;

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t obstacle_sub;
rcl_publisher_t motor_pub; // New publisher for motor state
std_msgs__msg__Bool obstacle_msg;
std_msgs__msg__Bool motor_state_msg; // Message for the new publisher

void obstacle_callback(const void *msgin) {
  const std_msgs__msg__Bool *obstacle_msg = (std_msgs__msg__Bool *)msgin;
  // Set the motor state based on the obstacle detection
  

  if(obstacle_msg->data){
    motor_state = true;
    digitalWrite(MOTOR_PWM_PIN, LOW);
  }else{
    motor_state = false;
    digitalWrite(MOTOR_PWM_PIN, HIGH);
  }

   
  // Print the obstacle detection status
  Serial.print("Obstacle detected: ");
  Serial.println(obstacle_msg->data ? "true" : "false");
}

bool create_entities()
{
  const char *node_name = "esp32_obstacle_node";
  const char *ns = "";
  const int domain_id = 0;

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  // Initialize the subscription for obstacle detection
  rclc_subscription_init(
    &obstacle_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/obstacle_detected",
    &rmw_qos_profile_default
  );

  // Initialize the publisher for motor state
  rclc_publisher_init_default(
    &motor_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/motor_vibrating"
  );

  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &obstacle_sub, &obstacle_msg, &obstacle_callback, ON_NEW_DATA);

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&obstacle_sub, &node);
  rcl_publisher_fini(&motor_pub, &node); // Clean up the new publisher
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
 
void setup() {
  Serial.begin(115200);
  Serial.println("Setting up");

  //set_microros_transports();
  set_microros_wifi_transports("ssid", "password", "hostmachine", 8888);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_TEST, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(MOTOR_PWM_PIN, HIGH);
  obstacle_msg.data = false;
  motor_state_msg.data = false; // Initialize the new message
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // Publish the motor state
        motor_state_msg.data = motor_state;
        rcl_publish(&motor_pub, &motor_state_msg, NULL);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // Update LED and motor based on current state
  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }


}