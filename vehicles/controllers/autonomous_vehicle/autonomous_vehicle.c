/*
 * Description:  Autonomous vehicle controller example with Linux SocketCAN SIL telemetry
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include "can_telemetry.h"

// Standardowe nagłówki Linux SocketCAN do bezpośredniego czytania rozkazów AEB
#include <sys/socket.h>
#include <linux/can.h>
#include <unistd.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#define AEB_TEST_RIG 1

enum { X, Y, Z };

#define TIME_STEP 50
#define UNKNOWN 99999.99

#define KP 0.25
#define KI 0.006
#define KD 2

bool PID_need_reset = false;
#define FILTER_SIZE 3

bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

double global_obstacle_dist = 200.0; 
uint8_t current_aeb_brake_intensity = 0; 

void print_help() {
  printf("You can drive this car!\n");
}

void set_autodrive(bool onoff) {
  if (autodrive == onoff) return;
  autodrive = onoff;
}

void set_speed(double kmh) {
  if (kmh > 250.0) kmh = 250.0;
  speed = kmh;
  wbu_driver_set_cruising_speed(kmh);
}

void set_steering_angle(double wheel_angle) {
  if (wheel_angle - steering_angle > 0.1) wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1) wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  if (wheel_angle > 0.5) wheel_angle = 0.5;
  else if (wheel_angle < -0.5) wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}

void change_manual_steer_angle(int inc) {
  set_autodrive(false);
  double new_manual_steering = manual_steering + inc;
  if (new_manual_steering <= 25.0 && new_manual_steering >= -25.0) {
    manual_steering = new_manual_steering;
    set_steering_angle(manual_steering * 0.02);
  }
}

void check_keyboard() {
  int key = wb_keyboard_get_key();
  switch (key) {
    case WB_KEYBOARD_UP: set_speed(speed + 5.0); break;
    case WB_KEYBOARD_DOWN: set_speed(speed - 5.0); break;
    case WB_KEYBOARD_RIGHT: change_manual_steer_angle(+1); break;
    case WB_KEYBOARD_LEFT: change_manual_steer_angle(-1); break;
    case 'A': set_autodrive(true); break;
  }
}

int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}

double process_camera_image(const unsigned char *image) {
  if (image == NULL) return UNKNOWN;
  int num_pixels = camera_height * camera_width;
  const unsigned char REF[3] = {95, 187, 203};
  int sumx = 0;
  int pixel_count = 0;
  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++;
    }
  }
  if (pixel_count == 0) return UNKNOWN;
  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}

double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;
  if (first_call || new_value == UNKNOWN) {
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i) old_value[i] = 0.0;
  } else {
    for (i = 0; i < FILTER_SIZE - 1; ++i) old_value[i] = old_value[i + 1];
  }
  if (new_value == UNKNOWN) return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i) sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}

double process_sick_data(const float *sick_data, double *obstacle_dist) {
  if (sick_data == NULL) return UNKNOWN;
  const int HALF_AREA = 20;
  int sumx = 0;
  int collision_count = 0;
  int x;
  *obstacle_dist = 0.0;
  for (x = sick_width / 2 - HALF_AREA; x < sick_width / 2 + HALF_AREA; x++) {
    float range = sick_data[x];
    if (range < 20.0) {
      sumx += x;
      collision_count++;
      *obstacle_dist += range;
    }
  }
  if (collision_count == 0) return UNKNOWN;
  *obstacle_dist = *obstacle_dist / collision_count;
  return ((double)sumx / collision_count / sick_width - 0.5) * sick_fov;
}

void update_display() {
  const double NEEDLE_LENGTH = 50.0;
  wb_display_image_paste(display, speedometer_image, 0, 0, false);
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed)) current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  if (coords == NULL) return;
  const double speed_ms = wb_gps_get_speed(gps);
  gps_speed = speed_ms * 3.6;
  memcpy(gps_coords, coords, sizeof(gps_coords));
}

double applyPID(double yellow_line_angle) {
  static double oldValue = 0.0;
  static double integral = 0.0;
  if (PID_need_reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    PID_need_reset = false;
  }
  if (signbit(yellow_line_angle) != signbit(oldValue)) integral = 0.0;
  double diff = yellow_line_angle - oldValue;
  if (integral < 30 && integral > -30) integral += yellow_line_angle;
  oldValue = yellow_line_angle;
  return KP * yellow_line_angle + KI * integral + KD * diff;
}

int main(int argc, char **argv) {
  wbu_driver_init();

  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "Sick LMS 291") == 0) enable_collision_avoidance = true;
    else if (strcmp(name, "display") == 0) enable_display = true;
    else if (strcmp(name, "gps") == 0) has_gps = true;
    else if (strcmp(name, "camera") == 0) has_camera = true;
  }

  if (has_camera) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);
  }

  if (enable_collision_avoidance) {
    sick = wb_robot_get_device("Sick LMS 291");
    wb_lidar_enable(sick, TIME_STEP);
    sick_width = wb_lidar_get_horizontal_resolution(sick);
    sick_range = wb_lidar_get_max_range(sick);
    sick_fov = wb_lidar_get_fov(sick);
  }

  if (has_gps) {
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  }

  if (enable_display) {
    display = wb_robot_get_device("display");
    speedometer_image = wb_display_image_load(display, "speedometer.png");
  }

  if (has_camera) set_speed(40.0);
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);

  int can_socket = can_telemetry_init("vcan0");
  print_help();
  wb_keyboard_enable(TIME_STEP);

  while (wbu_driver_step() != -1) {
    check_keyboard();
    static int i = 0;

    if (can_socket >= 0) {
      struct can_frame rx_frame;
      while (recv(can_socket, &rx_frame, sizeof(struct can_frame), MSG_DONTWAIT) > 0) {
        if (rx_frame.can_id == 0x200 && rx_frame.can_dlc >= 1) {
          current_aeb_brake_intensity = rx_frame.data[0];
        }
      }
    }

    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0) {
      const unsigned char *camera_image = NULL;
      const float *sick_data = NULL;
      
      if (has_camera) camera_image = wb_camera_get_image(camera);
      if (enable_collision_avoidance) sick_data = wb_lidar_get_range_image(sick);

      if (autodrive && has_camera && camera_image != NULL) {
        double yellow_line_angle = filter_angle(process_camera_image(camera_image));
        double obstacle_dist = 200.0; 
        double obstacle_angle;
        
        if (enable_collision_avoidance && sick_data != NULL) {
          obstacle_angle = process_sick_data(sick_data, &obstacle_dist);
        } else {
          obstacle_angle = UNKNOWN;
        }

        global_obstacle_dist = (obstacle_angle != UNKNOWN) ? obstacle_dist : 200.0;

        #if AEB_TEST_RIG 

        if (current_aeb_brake_intensity > 0) {
          wbu_driver_set_cruising_speed(0.0); 
          wbu_driver_set_brake_intensity((double)current_aeb_brake_intensity / 100.0);
          printf("[CAN AEB INTERVENTION]: Executing emergency braking: %d%%\n", current_aeb_brake_intensity);
        } 

        #else 
        else {
          
          // Standardowy bieg autopilota (wykonuje się tylko, gdy AEB jest czyste)
          if (enable_collision_avoidance && obstacle_angle != UNKNOWN) {
            wbu_driver_set_brake_intensity(0.0);
            double obstacle_steering = steering_angle;
            if (obstacle_angle > 0.0 && obstacle_angle < 0.4)
              obstacle_steering = steering_angle + (obstacle_angle - 0.25) / obstacle_dist;
            else if (obstacle_angle > -0.4)
              obstacle_steering = steering_angle + (obstacle_angle + 0.25) / obstacle_dist;
            double steer = steering_angle;
            if (yellow_line_angle != UNKNOWN) {
              const double line_following_steering = applyPID(yellow_line_angle);
              if (obstacle_steering > 0 && line_following_steering > 0)
                steer = obstacle_steering > line_following_steering ? obstacle_steering : line_following_steering;
              else if (obstacle_steering < 0 && line_following_steering < 0)
                steer = obstacle_steering < line_following_steering ? obstacle_steering : line_following_steering;
            } else PID_need_reset = true;
            set_steering_angle(steer);
          } else if (yellow_line_angle != UNKNOWN) {
            wbu_driver_set_brake_intensity(0.0);
            set_steering_angle(applyPID(yellow_line_angle));
          } else {
            wbu_driver_set_brake_intensity(0.4);
            PID_need_reset = true;
          }
            
           #endif
        }
      }

      if (has_gps) {
        const double *coords = wb_gps_get_values(gps);
        if (coords != NULL) compute_gps_speed();
      }
      if (enable_display) update_display();

      if (can_socket >= 0) {
        double current_speed = wbu_driver_get_current_speed();
        if (isnan(current_speed) || current_speed < 0.0) current_speed = 0.0;

        uint16_t simulated_rpm = 800 + (uint16_t)(current_speed * 60.0);
        if (simulated_rpm > 6500) simulated_rpm = 6500;
        uint8_t simulated_pedal = (speed > 0.0) ? (uint8_t)((current_speed / speed) * 100.0) : 0;
        if (simulated_pedal > 100) simulated_pedal = 100;

        can_send_engine_data(can_socket, simulated_rpm, simulated_pedal);
        can_send_wheel_speeds(can_socket, current_speed, current_speed, current_speed, current_speed);

        struct can_frame tx_frame;
        tx_frame.can_id = 0x300;
        tx_frame.can_dlc = 3;

        tx_frame.data[0] = (uint8_t)current_speed;

        uint16_t dist_cm = (uint16_t)(global_obstacle_dist * 100.0);
        if (dist_cm > 20000) dist_cm = 20000; 

        tx_frame.data[1] = (dist_cm >> 8) & 0xFF;
        tx_frame.data[2] = dist_cm & 0xFF; 

        memset(&tx_frame.data[3], 0, 5);

        send(can_socket, &tx_frame, sizeof(struct can_frame), 0);
      }
    }

    ++i;
  }

  wbu_driver_cleanup();
  return 0;
}