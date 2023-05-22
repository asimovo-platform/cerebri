/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>

#include <stdio.h>

#include <synapse_zbus/channels.h>

#include "casadi/rover.h"
#include "parameters.h"

#define MY_STACK_SIZE 10240
#define MY_PRIORITY 4


enum control_mode_t {
    MODE_MANUAL=0,
    MODE_AUTO=1,
    MODE_CMD_VEL=2,
};
typedef enum control_mode_t control_mode_t;
char * mode_name[3] = {"manual", "auto", "cmd_vel"};

control_mode_t g_mode = {MODE_MANUAL};
bool g_armed = false;
static Odometry g_pose  = Odometry_init_zero;
static Twist g_cmd_vel = Twist_init_zero;
static BezierTrajectory g_bezier_trajectory = BezierTrajectory_init_zero;

static void handle_joy(Joy * joy) {
    // arming
    if (joy->buttons[7] == 1) {
        g_armed = true;
    } else if (joy->buttons[6] == 1) {
        g_armed = false;
    }

    // handle modes
    control_mode_t prev_mode = g_mode;
    if (joy->buttons[0] == 1) {
        g_mode =  MODE_MANUAL;
    } else if (joy->buttons[1] == 1) {
        if (g_bezier_trajectory.time_start != 0) {
            g_mode =  MODE_AUTO;
        } else {
            printf("no valid tranjectory\n");
        }
    } else if (joy->buttons[2] == 1) {
        g_mode =  MODE_CMD_VEL;
    }

    // notify on mode change
    if (g_mode != prev_mode) {
        printf("mode changed to %s!\n", mode_name[g_mode]);
    }

    // translate joystick to twist message
    if (g_mode == MODE_MANUAL) {
        g_cmd_vel.linear.x = joy->axes[1];
        g_cmd_vel.angular.z = joy->axes[3];
    }
}

static void listener_control_rover_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_joy) {
        handle_joy((Joy*)(chan->mutex));
    } else if (chan == &chan_in_odometry) {
        g_pose = *(Odometry*)(chan->message);
    } else if (chan == &chan_in_cmd_vel) {
        g_cmd_vel = *(Twist*)(chan->message);
    } else if (chan == &chan_in_bezier_trajectory) {
        g_bezier_trajectory = *(BezierTrajectory*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_control_rover, listener_control_rover_callback);

// computes rc_input from V, omega
void mixer() {

    // given cmd_vel, compute actuators
    double V = g_cmd_vel.linear.x;
    double omega = g_cmd_vel.angular.z;
    Actuators actuators = Actuators_init_zero;

    // casadi mem args
    casadi_int * iw = NULL;
    casadi_real * w = NULL;
    int mem = 0;

#ifdef CONFIG_CONTROL_ROVER_STEERING_ACKERMANN
    /* ackermann_steering:(L,omega,V)->(delta) */
    {

        double delta = 0;
        const casadi_real * args[3];
        casadi_real * res[1];
        args[0] = &wheel_base;
        args[1] = &omega;
        args[2] = &V;
        res[0] = &delta;
        ackermann_steering(args, res, iw, w, mem);
        double omega_fwd = V/wheel_radius;
        double turn_angle = 0;
        if (fabs(V) > 0.01) {
            turn_angle = delta;
        }
        actuators.position[0] = turn_angle;
        actuators.velocity[0] = omega_fwd;
    }
#endif

#ifdef CONFIG_CONTROL_ROVER_STEERING_DIFFERENTIAL
    /* differential_steering:(L,omega,w)->(Vw) */
    {
        double Vw = 0;
        const casadi_real * args[3];
        casadi_real * res[1];
        args[0] = &wheel_base;
        args[1] = &omega;
        args[2] = &wheel_separation;
        res[0] = &Vw;
        differential_steering(args, res, iw, w, mem);
        double omega_fwd = V/wheel_radius;
        double omega_turn = Vw/wheel_radius;
        actuators.velocity[0] = omega_fwd - omega_turn;
        actuators.velocity[1] = omega_fwd + omega_turn;
        actuators.velocity[2] = omega_fwd + omega_turn;
        actuators.velocity[3] = omega_fwd - omega_turn;
    }
#endif

    zbus_chan_pub(&chan_out_actuators, &actuators, K_NO_WAIT);
}

// computes thrust/steering in auto mode
void auto_mode() {
    // goal -> given position goal, find cmd_vel
    
    uint64_t time_start = g_bezier_trajectory.time_start;
    uint64_t time_stop = g_bezier_trajectory.time_stop;
    int64_t uptime = k_uptime_get()*1e6;
    int64_t T_nsec = time_stop - time_start;
    int64_t t_nsec = uptime - time_start;
    if (t_nsec > T_nsec) {
      // stop
      g_cmd_vel.linear.x = 0;
      g_cmd_vel.angular.z = 0;
      return;
    }
    if (t_nsec < 0) {
      t_nsec = 0;
    }
    double t = t_nsec*1e-9;
    double T = T_nsec*1e-9;
    double x, y, psi, V, omega = 0;
    double e[3] = {}; // e_x, e_y, e_theta
  
    double PX[6], PY[6];
    for (int i=0; i<6; i++) {
        PX[i] = g_bezier_trajectory.x[i];
        PY[i] = g_bezier_trajectory.y[i];
    }

    // casadi mem args
    casadi_int * iw = NULL;
    casadi_real * w = NULL;
    int mem = 0;

    /* bezier6_rover:(t,T,PX[1x6],PY[1x6],L)->(x,y,psi,V,omega) */
    {
        const casadi_real * args[5];
        casadi_real * res[5];
        args[0] = &t;
        args[1] = &T;
        args[2] = PX;
        args[3] = PY;
        args[4] = &wheel_base;
        res[0] = &x;
        res[1] = &y;
        res[2] = &psi;
        res[3] = &V;
        res[4] = &omega;
        bezier6_rover(args, res, iw, w, mem);
    }

    /* se2_error:(p[3],r[3])->(error[3]) */
    {
        const casadi_real * args[2];
        casadi_real * res[1];

        double p[3], r[3];

        // vehicle position
        p[0] = g_pose.pose.pose.position.x;
        p[1] = g_pose.pose.pose.position.y;
        p[2] = 2*atan2(
                g_pose.pose.pose.orientation.z,
                g_pose.pose.pose.orientation.w);

        // reference position
        r[0] = x;
        r[1] = y;
        r[2] = psi;

        // call function
        args[0] = p;
        args[1] = r;
        res[0] = e;
        se2_error(args, res, iw, w, mem);
    }

    // compute twist
    g_cmd_vel.linear.x = V + wheel_radius*gain_along_track*e[0];
    g_cmd_vel.angular.z = omega + gain_cross_track*e[1] + gain_heading*e[2];
}

void control_entry_point(void * p1, void * p2, void * p3) {

    while (true) {
        // auto
        if (g_mode == MODE_AUTO) {
            auto_mode();
        }

        mixer();

        // sleep to set control rate at 50 Hz
        k_usleep(1e6/50);
    }
}

K_THREAD_DEFINE(control_thread, MY_STACK_SIZE,
    control_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
