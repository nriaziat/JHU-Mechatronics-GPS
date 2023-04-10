#include "IRCam.h"
#include "ParticleFilter.h"
#include <random>

robotPose pf_pose;
particle pf_particles[50];
IRCam cam;
ParticleFilter pf;
int led_pin = LED_BUILTIN;
bool led_state = true;
bool got_odometry = false;
char msg1Buf[9];
char msg2Buf[9];
char msg3Buf[9];
float dt;
robotPose initial_guess;
std::default_random_engine gen;
std::uniform_real_distribution<double> norm_dist(0, 3);
int i = 0;
float radius = 20;
float dist, dtheta;
uint out_Ix[4], out_Iy[4];


void requestEvent();
void receiveEvent(int howMany);
void printParticlesToProcessing();

void setup() {
  cam.begin();
  pinMode(led_pin, OUTPUT);
  Wire1.begin(8);
  Wire1.onRequest(requestEvent);  // register event
  Wire1.onReceive(receiveEvent);  // register event
  // Serial.begin(9600);
  // initial_guess.x = 180;
  // initial_guess.y = 60;
  // initial_guess.theta = M_PI / 2.f;
  // pf.begin(initial_guess);
  pf.begin();
}

void loop() {
  // Toggle LED to indicate loop rate
  digitalWrite(led_pin, led_state);
  led_state = !led_state;

  // Get IR camera reading and pass to particle filter
  uint n = cam.getIx(out_Ix);
  cam.getIy(out_Iy);
  pf.inputIRCam(out_Ix, out_Iy, n);
  pf_pose = pf.getPoseEstimate(OUTPUT_MODE::MEAN);

  // printParticlesToProcessing();
  dtostrf(pf_pose.x, 0, 2, msg1Buf);
  dtostrf(pf_pose.y, 0, 2, msg2Buf);
  dtostrf(pf_pose.theta, 0, 2, msg3Buf);
}

void requestEvent() {
  Wire1.write(msg1Buf);
  Wire1.write(",");
  Wire1.write(msg2Buf);
  Wire1.write(",");
  Wire1.write(msg3Buf);
}

void receiveEvent(int howMany) {
  char* buf;
  Wire1.readBytes(buf, 4);
  float dist = atof(buf);
  Wire1.readBytes(buf, 4);
  float angle = atof(buf);
  pf.inputOdometry(dist, angle);
  got_odometry = true;
}

void odometry(robotPose* pose, float dist, float dtheta) {
  std::default_random_engine generator;
  std::normal_distribution<double> norm(0, 0.01);
  std::normal_distribution<double> dist_norm(0, 0.5);
  // float alpha = dtheta;
  // float alpha_prime = alpha + alpha * norm(generator) + dist * dist_norm(generator);
  // float beta = 0;
  // float beta_prime = beta + beta * norm(generator) + dist * dist_norm(generator);
  // float d_prime = dist + dist * dist_norm(generator) + (alpha + beta) * norm(generator);
  float d_hat = dist + dist_norm(generator);
  float theta_hat = dtheta + norm(generator);
  pose->x += d_hat / theta_hat * sin(pose->theta + dtheta) - d_hat / theta_hat * sin(pose->theta);
  if (pose->x > 244) {
    pose->x = 244;
  }
  if (pose->x < 0) {
    pose->x = 0;
  }
  pose->y += -d_hat / theta_hat * cos(pose->theta + dtheta) + d_hat / theta_hat * cos(pose->theta);
  pose->theta += theta_hat;
  if (pose->y > 122) {
    pose->y = 122;
  }
  if (pose->y < 0) {
    pose->y = 0;
  }

  // pose->x += d_prime * cos(pose->theta + alpha_prime);
  // pose->y += d_prime * sin(pose->theta + alpha_prime);
  // pose->theta += alpha_prime + beta_prime;
  pose->theta = atan2(sin(pose->theta), cos(pose->theta));
}

void printParticlesToProcessing(){
    // pf.getMostLikelyParticles(50, pf_particles);
    for (size_t i = 0; i < 50; i++) {
      Serial.print(i + 1);
      Serial.print(",");
      Serial.print(pf_particles[i].pose.x);
      Serial.print(",");
      Serial.print(pf_particles[i].pose.y);
      Serial.print(",");
      Serial.print(pf_particles[i].pose.theta);
      Serial.print(",");
      Serial.print(pf_particles[i].weight);
      Serial.print(",");
    }
    // Serial.print("0,");
    // Serial.print(state.x);
    // Serial.print(",");
    // Serial.print(state.y);
    // Serial.print(",");
    // Serial.print(state.theta);
    // Serial.print(",1,");
    // Serial.print("51,");
    // Serial.print(pf_pose.x);
    // Serial.print(",");
    // Serial.print(pf_pose.y);
    // Serial.print(",");
    // Serial.print(pf_pose.theta);
    // Serial.print(",1,");
    // Serial.println();

    Serial.print("Estimated Point: X: ");
    Serial.print(pf_pose.x);
    Serial.print(", Y: ");
    Serial.print(pf_pose.y);
    Serial.print(", Theta: ");
    Serial.println(pf_pose.theta);
    // float dx = pf_pose.x - state.x;
    // float dy = pf_pose.y - state.y;
    // float error = sqrt(dx*dx + dy*dy);
    // Serial.print("Computed error: ");
    // Serial.println(error);
}