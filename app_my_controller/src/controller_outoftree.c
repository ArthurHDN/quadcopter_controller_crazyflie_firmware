#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "stabilizer_types.h"

#define DEBUG_MODULE "COOT"
#include "debug.h"



// Global state variable used in the
// firmware as the only instance and in bindings
// to hold the default values
typedef struct {
  struct quat prev_target_attitude;
  struct vec prev_varphi;
  struct quat prev_attitude;
  struct quat nominal_attitude;
  struct vec nominal_angular_velocity;
  float xi3_hat;
  float xi4_hat;
  float t;
} controllerMyController_t;

static controllerMyController_t g_self = {
  .xi3_hat = 0.0f,
  .xi4_hat = 0.0f,
  .t = 0.0f,
};

void controllerMyControllerReset(controllerMyController_t* self) {
  self->xi3_hat = 0.0f;
  self->xi4_hat = 0.0f;
  self->t = 0.0f;
}

void controllerMyControllerInit(controllerMyController_t* self) {
  // copy default values (bindings), or does nothing (firmware)
  *self = g_self;
  controllerMyControllerReset(self);
}

bool controllerMyControllerTest(controllerMyController_t* self) {
  return true;
}

static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

static struct mat33 CRAZYFLIE_INERTIA_INV =
  {{{60441.72109333f, -2880.22052559f, -1308.31543485f},
       {-2880.22052559f, 60782.19836313f, -3663.28321758f},
       {-1308.31543485f, -3663.28321758f, 34386.89067934f}}};


// // time constant of rotational rate control
// static float tau_rp_rate = 0.015;
// static float tau_yaw_rate = 0.0075;

// // minimum and maximum thrusts
// static float coll_min = 1;
// static float coll_max = 18;
// // if too much thrust is commanded, which axis is reduced to meet maximum thrust?
// // 1 -> even reduction across x, y, z
// // 0 -> z gets what it wants (eg. maintain height at all costs)
// static float thrust_reduction_fairness = 0.25;

// // minimum and maximum body rates
// static float omega_rp_max = 30;
// static float omega_yaw_max = 10;
// static float heuristic_rp = 12;
// static float heuristic_yaw = 5;

// static struct mat33 K_varphi =
//     {{{10.0f, 0.0f, 0.0f},
//       {0.0f, 10.0f, 0.0f},
//       {0.0f, 0.0f, 10.0f}}};

// static struct mat33 K_o =
//     {{{3.7776f, -0.0162f, -0.0653f},
//       {-0.0201f, 5.7169f, -0.0021f},
//       {-0.0814f, -0.0021f, 5.7087f}}};
static struct mat33 K_o =
    {{{5.0f, 0.0f, 0.0f},
      {0.0f, 5.0f, 0.0f},
      {0.0f, 0.0f, 5.0f}}};

static struct mat33 K_w =
    {{{10.3813f, -0.0024f, -0.0098f},
      {-0.0024f, 10.5941f, -0.0003f},
      {-0.0098f, -0.0003f, 10.5928f}}};

// static struct mat33 K_varphi =
//     {{{5.0f, 0.0f, 0.0f},
//       {0.0f, 5.0f, 0.0f},
//       {0.0f, 0.0f, 1.0f}}};

// static struct mat33 K_o =
//     {{{0.37776f, -0.00162f, -0.00653f},
//       {-0.00201f, 0.57169f, -0.00021f},
//       {-0.00814f, -0.00021f, 0.57087f}}};

// static struct mat33 K_w =
//     {{{1.03813f, -0.00024f, -0.00098f},
//       {-0.00024f, 1.05941f, -0.00003f},
//       {-0.00098f, -0.00003f, 1.05928f}}};

static float k_varphi_e = 2.5f;
static float k_e_o = 6.0e-3f;
static float k_e_w = 6.0e-3f;
static float epsilon_o = 2.0e-3f;
static float rho_3 = 2.5e-5f;
static float rho_4 = 2.5e-5f;

static float xi_r = 0.0005f;
static float saturate_xi3_hat = 0.0f;
static float saturate_xi4_hat = 0.0f;
static float robust_controller = 1.0f;
static float att_thrust = 1.0f;

static inline struct vec doublelnquat(struct quat q) {
  // float phi = quat2angle(q);
  // struct vec n = quat2axis(q);
  // struct vec z = vscl(phi, n);
  struct vec q_imag = mkvec(q.x,q.y,q.z);
  float q_imag_norm = vmag(q_imag);
  if (q_imag_norm == 0.0f){
    return vzero();
  }
  float q_real = q.w;
  float q_norm = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  struct vec z = vscl(
    2.0f,
    vscl(
      acosf(q_real/q_norm)/q_imag_norm, q_imag
    )
  );
  return z;
}

static inline struct vec angular_velocity(struct quat q1, struct quat q2, float dt) {
  struct vec w = vscl(
    (2.0f/dt),
    mkvec(
      q1.w*q2.x - q1.x*q2.w - q1.y*q2.z + q1.z*q2.y,
      q1.w*q2.y + q1.x*q2.z - q1.y*q2.w - q1.z*q2.x,
      q1.w*q2.z - q1.x*q2.y + q1.y*q2.x - q1.z*q2.w
    )
  );
  return w;
}

static inline struct quat qqmul2(struct quat q1, struct quat q2) {
  struct vec q1_vec = quatimagpart(q1);
  struct vec q2_vec = quatimagpart(q2);
  float w = q1.w*q2.w - vdot(q1_vec,q2_vec);
  struct vec v = vadd3(
    vscl(q1.w, q2_vec),
    vscl(q2.w, q1_vec),
    vcross(q1_vec, q2_vec)
  );
  return quatvw(v,w);
}

static inline struct vec qvrot2(struct vec v, struct quat q) {
  struct quat v_quat = qqmul2(qinv(q), qqmul2(quatvw(v,0.0f), q));
  return quatimagpart(v_quat);
}

// RATE_1000_HZ
// RATE_500_HZ
// RATE_250_HZ
// RATE_100_HZ
// RATE_50_HZ
// RATE_25_HZ
#define UPDATE_RATE RATE_250_HZ

void controllerMyController(controllerMyController_t* self, control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {

  static float control_thrust;

  // define this here, since we do body-rate control at 100Hz below the following if statement
  float omega[3] = {0};
  omega[0] = radians(sensors->gyro.x);
  omega[1] = radians(sensors->gyro.y);
  omega[2] = radians(sensors->gyro.z);

  if (!RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
    return;
  }
  float dt = (float)(1.0f/UPDATE_RATE);
  self->t += dt;

  control_thrust = att_thrust;

  ////////////////////
  // REFERENCE
  ///////////////////
  struct quat target_attitude = mkquat(
    setpoint->velocity.x,
    setpoint->velocity.y,
    setpoint->velocity.z,
    setpoint->attitudeRate.yaw
  );
  if (target_attitude.x == 0.0f && target_attitude.y == 0.0f && target_attitude.z == 0.0f && target_attitude.w == 0.0f){
    target_attitude = qeye();
  } else {
    target_attitude = qnormalize(target_attitude);
  }
  struct vec w_d = vzero();
  if (self->prev_target_attitude.x == self->prev_target_attitude.x){
    w_d = angular_velocity(self->prev_target_attitude, target_attitude, dt);
  }
  self->prev_target_attitude = target_attitude;

  ////////////////////
  // STATES
  ///////////////////
  struct quat attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);
  struct vec w = vzero();
  if (self->prev_attitude.x == self->prev_attitude.x){
    w = angular_velocity(self->prev_attitude, attitude, dt);
  }
  self->prev_attitude = mkquat(attitude.x, attitude.y, attitude.z, attitude.w);

  if ((robust_controller == 0.0f) || (self->nominal_attitude.x != self->nominal_attitude.x)){
    self->nominal_attitude = mkquat(attitude.x, attitude.y, attitude.z, attitude.w);
    self->nominal_angular_velocity = mkvec(w.x, w.y, w.z);
  } else if (robust_controller == 2.0f) {
    self->nominal_angular_velocity = vzero();
    self->nominal_attitude = qeye();
  }
  
  
  ////////////////////
  // NOMINAL CONTROL
  ///////////////////
  struct quat attitude_error = qqmul2(qinv(target_attitude),self->nominal_attitude);
  struct vec z_o = quatimagpart(attitude_error); // z_o.z = 0.0f;
  // varphi = w_d_B
  struct vec varphi = qvrot2(w_d, attitude_error);
  // dvarphidt
  struct vec dvarphidt = vzero();
  if (self->prev_varphi.x == self->prev_varphi.x){
    dvarphidt = vscl(1.0f/dt, vsub(varphi, self->prev_varphi));
  }
  self->prev_varphi = mkvec(varphi.x, varphi.y, varphi.z);
  // z_w = w - varphi
  struct vec z_w = vsub(self->nominal_angular_velocity, varphi);
  // tau_L = -K_o*z_o - K_w*z_w
  struct vec tau_L = vadd(
    vneg( mvmul(K_o, z_o) ),
    vneg( mvmul(K_w, z_w) )
  ); // tau_L.z = 0.0f;
  // tau = w x Jw + Jdphidt + Jtau_L
  struct vec bar_tau = vadd3(
    vcross(self->nominal_angular_velocity, mvmul(CRAZYFLIE_INERTIA,self->nominal_angular_velocity)),
    mvmul(CRAZYFLIE_INERTIA, dvarphidt),
    mvmul(CRAZYFLIE_INERTIA, tau_L)
  );
  // DEBUG_PRINT("z_o.z = %f\n", (double)z_o.z);


  ////////////////////
  // ROBUST CONTROL
  ///////////////////
  // // oe = o¯o∗
  struct quat o_e = qqmul2(qinv(self->nominal_attitude), attitude);
  // // ωe = ω− ¯ω_B
  struct vec w_e = vsub(w, qvrot2(self->nominal_angular_velocity,o_e));
  // // eo = I{oe}
  struct vec e_o = quatimagpart(o_e); // e_o.z = 0.0f;
  // // φe = −kφeeo
  struct vec varphi_e = vneg(vscl(k_varphi_e, e_o));
  // // eω = ωe −φe
  struct vec e_w = vsub(w_e, varphi_e);
  // // τ e = −keoeo − keωeω − ˆζ3tanh(ˆζ3eω/ϵ) − ˆζ4eω.
  struct vec tau_e = vneg(vadd4(
    vscl(k_e_o, e_o),
    vscl(k_e_w, e_w),
    vscl(self->xi3_hat, mkvec(
      tanhf(self->xi3_hat*e_w.x/epsilon_o),
      tanhf(self->xi3_hat*e_w.y/epsilon_o),
      tanhf(self->xi3_hat*e_w.z/epsilon_o)
    )),
    vscl(self->xi4_hat, e_w)
  ));

  // float xi3_hat = 0.0f;//self->xi3_hat;
  // float xi4_hat = self->xi4_hat;//self->xi4_hat;

  // // struct vec aux = vscl(self->xi4_hat, e_w);

  // struct vec aux = vzero();
  // aux.x += e_w.x;
  // aux.y += e_w.y;
  // aux.z += e_w.z;
  // aux.x *= xi4_hat;
  // aux.y *= xi4_hat;
  // aux.z *= xi4_hat;

  // // if (self->prev_aux.x == self->prev_aux.x){
  // //   if (vmag2(e_w) != 0.0f && xi4_hat != 0.0f && vmag2(aux) == 0.0f ){
  // //     aux = self->prev_aux;
  // //   }
  // // }
  // // self->prev_aux = aux;

  

  // struct vec tau_e = vneg(vadd4(
  //   vscl(k_e_o, e_o),
  //   vscl(k_e_w, e_w),
  //   vscl(xi3_hat, mkvec(
  //     tanhf(xi3_hat*e_w.x/epsilon_o),
  //     tanhf(xi3_hat*e_w.y/epsilon_o),
  //     tanhf(xi3_hat*e_w.z/epsilon_o)
  //   )),
  //   // vzero(),
  //   aux
  // ));

  

  // // DEBUG_PRINT("%f, %f\n", (double)self->xi3_hat,(double)self->xi4_hat);
  // DEBUG_PRINT(
  //   "%f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n",
  //   (double)self->xi4_hat,
  //   (double)aux.x,(double)aux.y, (double)aux.z,
  //   (double)e_w.x,(double)e_w.y, (double)e_w.z,
  //   (double)w_e.x,(double)w_e.y, (double)w_e.z,
  //   (double)varphi_e.x,(double)varphi_e.y, (double)varphi_e.z
  // );

  ////////////////////
  // PRACTICAL CONTROL
  ///////////////////
  struct vec bar_tau_B = mvmul(CRAZYFLIE_INERTIA,qvrot2(mvmul(CRAZYFLIE_INERTIA_INV,bar_tau), o_e));
  struct vec tau = vadd(bar_tau_B, tau_e);
  if (robust_controller == 0.0f) {
    tau = bar_tau;
  } else if (robust_controller == 2.0f) {
    tau = tau_e;
  }

  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torque[0] =  0.0f;
    control->torque[1] =  0.0f;
    control->torque[2] =  0.0f;
    controllerMyControllerReset(self);
  } else {
    control->thrustSi = control_thrust * CF_MASS; // force to provide control_thrust

    // float tau_rp_max = 3e-20f;
    // float tau_yaw_max = 1e-20f;

    // // scale the commands to satisfy rate constraints
    // float scaling = 1;
    // scaling = fmax(scaling, fabsf(tau.x) / tau_rp_max);
    // scaling = fmax(scaling, fabsf(tau.y) / tau_rp_max);
    // scaling = fmax(scaling, fabsf(tau.z) / tau_yaw_max);

    // tau.x /= scaling;
    // tau.y /= scaling;
    // tau.z /= scaling;


    control->torqueX = tau.x;
    control->torqueY = tau.y;
    control->torqueZ = tau.z;
  }
  control->controlMode = controlModeForceTorque;

  ////////////////////
  // INTEGRATE
  ///////////////////
  float kappa = self->xi3_hat*self->xi3_hat + self->xi4_hat*self->xi4_hat - xi_r*xi_r;

  if (kappa < 0 || 2*self->xi3_hat*rho_3*vmag(e_w) + 2*self->xi4_hat*rho_4*vmag2(e_w) <= 0) {
      self->xi3_hat = self->xi3_hat + dt*0.5f*rho_3*vmag(e_w);
      self->xi4_hat = self->xi4_hat + dt*0.5f*rho_4*vmag2(e_w);
  } else {
      self->xi3_hat = self->xi3_hat + dt*0.5f*(
          rho_3*vmag(e_w) - rho_3*(self->xi3_hat*self->xi3_hat*rho_3*vmag(e_w) + self->xi3_hat*self->xi4_hat*rho_4*vmag2(e_w))/(rho_3*self->xi3_hat*self->xi3_hat+rho_4*self->xi4_hat*self->xi4_hat)
      );
      self->xi4_hat = self->xi4_hat + dt*0.5f*(
          rho_4*vmag2(e_w) - rho_4*(self->xi3_hat*self->xi4_hat*rho_3*vmag(e_w) + self->xi4_hat*self->xi4_hat*rho_4*vmag2(e_w))/(rho_3*self->xi3_hat*self->xi3_hat+rho_4*self->xi4_hat*self->xi4_hat)
      );
  }

  // self->xi3_hat = self->xi3_hat + dt*0.5f*rho_3*vmag(e_w);
  if (self->xi3_hat > saturate_xi3_hat){
    self->xi3_hat = saturate_xi3_hat;
  }
  // self->xi4_hat = self->xi4_hat + dt*0.5f*rho_4*vmag2(e_w);
  if (self->xi4_hat > saturate_xi4_hat){
    self->xi4_hat = saturate_xi4_hat;
  }

  struct quat dnominal_atitudedt = qqmul2(
    self->nominal_attitude,
    mkquat(
      self->nominal_angular_velocity.x,
      self->nominal_angular_velocity.y,
      self->nominal_angular_velocity.z,
      0.0f
    )
  );
  self->nominal_attitude = mkquat(
    self->nominal_attitude.x + dt*0.5f*dnominal_atitudedt.x,
    self->nominal_attitude.y + dt*0.5f*dnominal_atitudedt.y,
    self->nominal_attitude.z + dt*0.5f*dnominal_atitudedt.z,
    self->nominal_attitude.w + dt*0.5f*dnominal_atitudedt.w
  );
  self->nominal_attitude = qnormalize(self->nominal_attitude);

  self->nominal_angular_velocity = vadd(self->nominal_angular_velocity,vscl(dt, vadd(
    vneg(mvmul(CRAZYFLIE_INERTIA_INV, vcross(self->nominal_angular_velocity, mvmul(CRAZYFLIE_INERTIA,self->nominal_angular_velocity)))),
    mvmul(CRAZYFLIE_INERTIA_INV, bar_tau)
  )));  
}



////////////////////////////////////
// OutOfTree function calls
///////////////////////////////////


void controllerOutOfTreeInit(void) {
  controllerMyControllerInit(&g_self);
}


void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  controllerMyController(&g_self, control, setpoint, sensors, state, stabilizerStep);
}


bool controllerOutOfTreeTest(void) {
  return controllerMyControllerTest(&g_self);
}


PARAM_GROUP_START(ctrlAtt)
PARAM_ADD(PARAM_FLOAT, xi_r, &xi_r)
PARAM_ADD(PARAM_FLOAT, saturate_xi3_hat, &saturate_xi3_hat)
PARAM_ADD(PARAM_FLOAT, saturate_xi4_hat, &saturate_xi4_hat)
PARAM_ADD(PARAM_FLOAT, robust_controller, &robust_controller)
PARAM_ADD(PARAM_FLOAT, att_thrust, &att_thrust)
PARAM_ADD(PARAM_FLOAT, k_varphi_e, &k_varphi_e)
PARAM_ADD(PARAM_FLOAT, k_e_o, &k_e_o)
PARAM_ADD(PARAM_FLOAT, k_e_w, &k_e_w)
PARAM_ADD(PARAM_FLOAT, epsilon_o, &epsilon_o)
PARAM_ADD(PARAM_FLOAT, rho_3, &rho_3)
PARAM_ADD(PARAM_FLOAT, rho_4, &rho_4)
PARAM_GROUP_STOP(ctrlAtt)
