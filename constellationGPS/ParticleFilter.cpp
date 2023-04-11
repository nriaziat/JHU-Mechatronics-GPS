#include "ParticleFilter.h"
#include "IRCam.h"
#include <random>
#include <stdlib.h>


double normal_pdf(float mu, float sigma, float x);
double normal_cdf(float mu, float sigma, float low_x, float high_x);
int sort_desc(const void *cmp1, const void *cmp2);
double normalize_angle(double a);
void normalize_angle(float *a);
void generateMap(point* map);
void rotate2D(point *in, float theta);

ParticleFilter::ParticleFilter(float height){
  m_height = height;
}

ParticleFilter::ParticleFilter() {
  ParticleFilter(15.4);
}

void ParticleFilter::begin() {
  generateMap(m_map);
  m_n = generateParticles();
}

void ParticleFilter::begin(robotPose guess) {
  generateMap(m_map);
  m_n = generateParticles(guess);
}

size_t ParticleFilter::generateParticles() {
  std::default_random_engine generator;
  std::normal_distribution<double> norm_theta(0, 1);

  float increment = roundf(10.f * sqrt(float(x_max * y_max) / float(sizeof(particles)/sizeof(particle)))) / 10.f;
  size_t i = 0;
  for (float x = 0; x < x_max; x += increment) {
    for (float y = 0; y < y_max; y += increment) {
      particles[i].pose.x = x;
      particles[i].pose.y = y;
      particles[i].pose.theta = norm_theta(generator);
      particles[i].weight = 1.0;
      i++;
    }
  }
  return i;
}

size_t ParticleFilter::generateParticles(robotPose guess) {
  std::default_random_engine generator;
  std::normal_distribution<double> norm_x(guess.x, 15);
  std::normal_distribution<double> norm_y(guess.y, 15);
  std::normal_distribution<double> norm_theta(guess.theta, 0.5);
  unsigned int n = float(sizeof(particles)) / float(sizeof(particle));
  for (size_t i=0; i<n; i++){
    particles[i].pose.x = norm_x(generator);
    particles[i].pose.y = norm_y(generator);
    particles[i].pose.theta = norm_theta(generator);
    particles[i].weight = 1;
  }
  return n;
}

void ParticleFilter::inputIRCam(const uint *Ix, const uint *Iy, size_t n_points) {
  // Serial.println("Getting sensor probabilities");
  getSensorProbability(Ix, Iy, n_points);
  // Serial.println("Normalizing weights");
  normalizeWeights();
  float n_eff_ratio;
  do { 
    float sq_sum_weight = 0;
    float sum_weight = 0;
    for (size_t i = 0; i < m_n; i++){
      sq_sum_weight += particles[i].weight * particles[i].weight;
      sum_weight += particles[i].weight;
    }
    n_eff_ratio =  (sum_weight*sum_weight / sq_sum_weight) / float(m_n);
    if (n_eff_ratio < 0.5)
      resample();
  } while (n_eff_ratio < 0.5);
  computeMeanEstimate();
}

void ParticleFilter::normalizeWeights() {
  double eta = 0;
  for (size_t i = 0; i < m_n; i++) {
    eta += particles[i].weight;
  }
  if (fabs(eta) > 1e-30) {
    for (size_t i = 0; i < m_n; i++) {
      particles[i].weight /= eta;
      particles[i].weight *= double(m_n);
    }
  }
}

void ParticleFilter::getSensorProbability(const uint *Ix, const uint *Iy, size_t n_points) {
  double sigma = 20;
  float x_sens, y_sens, range, bearing, x, y, min_dist, dist;

  for (size_t i = 0; i < m_n; i++) {
    if (particles[i].pose.x < 0 || particles[i].pose.y < 0){
      particles[i].weight = 0;
    } 
    else {
      // LIKELIHOOD FIELD MODEL
      for (size_t j = 0; j < n_points; j++) {
        x_sens = (float(Ix[j]) - 512.f)/ float(px_scaler);
        y_sens = (float(Iy[j]) - 384.f) / float(px_scaler);
        range = sqrt(x_sens*x_sens + y_sens*y_sens);
        bearing = atan2(y_sens, x_sens);
        x = particles[i].pose.x + range * cos(particles[i].pose.theta + bearing);
        y = particles[i].pose.y + range * sin(particles[i].pose.theta + bearing);
        min_dist = 10000000;
        for (size_t k=0; k < n_map; k++){
          dist = sqrt((x-m_map[k].x)*(x-m_map[k].x) + (y-m_map[k].y)*(y-m_map[k].y));
          if (dist < min_dist){
            min_dist = dist;
          }
        }

        particles[i].weight *= normal_pdf(0, sigma, min_dist);
      }
    }
  }
}

sensorReading ParticleFilter::sensorModel(robotPose state) {
  const float width = tan(0.29) * 244.0;
  const float height = tan(0.2) * 244.0;
  const float d = (width * width + height * height);
  auto centroid_x = state.x;
  auto centroid_y = state.y;
  sensorReading output;
  output.n = 0;
  if (centroid_x > x_max || centroid_x < 0 || centroid_y > y_max || centroid_y < 0)
    return output;

  // point X, Y, Z, W;
  // X.x = centroid_x + width;
  // X.y = centroid_y + height;
  // Y.x = centroid_x - width,
  // Y.y = centroid_y + height;
  // Z.x = centroid_x + width,
  // Z.y = centroid_y - height;
  // W.x = centroid_x - width,
  // W.y = centroid_y - height;
  size_t j = 0;
  for (size_t i = 0; i < n_map; i++) {
    point P;
    P.x = m_map[i].x - centroid_x;
    P.y = m_map[i].y - centroid_y;
    rotate2D(&P, -state.theta);
    // point center;
    // center.x = centroid_x;
    // center.y = centroid_y;
    if (P.x * P.x + P.y * P.y < d) {
      // if ( PointInRectangle( center, width, height, P) ){
      if (px_scaler * P.x + 512 > 0 and px_scaler * P.y + 384 > 0) {
        output.Ix[j] = px_scaler * P.x + 512;
        output.Iy[j] = px_scaler * P.y + 384;
        j++;
        if (j > 3) {
          break;
        }
      }
    }
  }
  output.n = j;
  if (output.n > n_map) {
    Serial.print("Sensor model sees too many points (overflow): ");
    Serial.println(output.n);
  }
  return output;
}

void ParticleFilter::inputOdometry(float dist, float dtheta) {
  // Odometry for Holonomic Robots
  // Serial.println("Inputting Odometry");
  std::default_random_engine generator;
  std::normal_distribution<double> norm(0, 0.1);
  std::normal_distribution<double> dist_norm(0, 50);
  for (size_t i = 0; i < m_n; i++) {
    
    float d_hat = dist + dist_norm(generator);
    float theta_hat = dtheta + norm(generator);

    particles[i].pose.x += d_hat / theta_hat * sin(particles[i].pose.theta + dtheta) - d_hat/theta_hat * sin(particles[i].pose.theta);
    particles[i].pose.y += -d_hat / theta_hat * cos(particles[i].pose.theta + dtheta) + d_hat/theta_hat * cos(particles[i].pose.theta);
    particles[i].pose.theta += theta_hat;
    normalize_angle(&particles[i].pose.theta);
  }
}

void ParticleFilter::inputOdometry(float dist, float dtheta1, float dtheta2) {
  // Odometry for Non Holonomic Robots
  std::default_random_engine generator;
  std::normal_distribution<double> norm(0, 0.1);
  std::normal_distribution<double> dist_norm(0, 50);
  for (size_t i = 0; i < m_n; i++) {
    float alpha = dtheta1;
    float alpha_prime = alpha + alpha * norm(generator) + dist * dist_norm(generator);
    float beta = dtheta2;
    float beta_prime = beta + beta * norm(generator) + dist * dist_norm(generator);
    float d_prime = dist + dist * dist_norm(generator) + (alpha + beta) * norm(generator);

    particles[i].pose.x += d_prime * cos(particles[i].pose.theta + alpha_prime);
    particles[i].pose.y += d_prime * sin(particles[i].pose.theta + alpha_prime);
    particles[i].pose.theta += alpha_prime + beta_prime;

    normalize_angle(&particles[i].pose.theta);
  }
}

void ParticleFilter::resample() {
  qsort(particles, m_n, sizeof(particle), sort_desc);
  particle *new_particles = (particle *)malloc(sizeof(particle) * m_n);
  if (new_particles == NULL) {
    Serial.println("COULD NOT MALLOC");
  }
  memcpy(new_particles, particles, sizeof(particle) * m_n);
  double probabilities[m_n];
  double rand_weight;
  double sum_weight = 0;

  for (size_t i = 0; i < m_n; i++) {
    probabilities[i] = sum_weight += particles[i].weight;  // compute running sum
    // Serial.println(probabilities[i]);
  }

  std::default_random_engine generator;
  std::uniform_real_distribution<double> uniform(0, sum_weight);

  for (size_t i = 0; i < m_n; i++) {
    // binary search for successor (next higher value)
    rand_weight = uniform(generator);
    size_t L = 0;
    size_t R = m_n;
    while (L < R) {
      size_t m = floor((L + R) / 2.f);
      if (probabilities[m] > rand_weight) {
        R = m;
      } else {
        L = m + 1;
      }
    }
    particles[i] = new_particles[R];
    //////////////
  }
  normalizeWeights();
  free(new_particles);
}

robotPose ParticleFilter::getPoseEstimate(OUTPUT_MODE mode) {
  qsort(particles, m_n, sizeof(particle), sort_desc);
  if (mode == OUTPUT_MODE::MEAN){
    return out_pose;
  } else { 
    return particles[0].pose;
  }
}

void ParticleFilter::computeMeanEstimate() {
    out_pose.x = 0;
    out_pose.y = 0;
    out_pose.theta = 0;
    for (size_t i = 0; i < m_n; i++){
      out_pose.x += particles[i].pose.x;
      out_pose.y += particles[i].pose.y;
      out_pose.theta += particles[i].pose.theta;
    }
    out_pose.x /= float(m_n);
    out_pose.y /= float(m_n);
    out_pose.theta /= float(m_n);
}

void ParticleFilter::getMostLikelyParticles(size_t n, particle *out_particles) {
  qsort(particles, m_n, sizeof(particle), sort_desc);
  for (size_t i = 0; i < n; i++) {
    out_particles[i] = particles[i];
  }
}

int sort_desc(const void *cmp1, const void *cmp2) {
  particle a = *((particle *)cmp1);
  particle b = *((particle *)cmp2);
  return a.weight > b.weight ? -1 : (a.weight < b.weight ? 1 : 0);
}


double normal_pdf(float mu, float sigma, float x) {
  return 1.0 / (double(sigma) * 2.5066282746) * exp(-0.5 * pow((double(x) - double(mu)) / double(sigma), 2));
}

double normal_cdf(float mu, float sigma, float low_x, float high_x) {
  float z2 = (high_x - mu) / (sigma * M_SQRT2);
  float z1 = (low_x - mu) / (sigma * M_SQRT2);

  return 0.5 * (erf(z2 * M_SQRT2) - erf(z1 * M_SQRT2));
}

double normalize_angle(double a) {
  return atan2(sin(a), cos(a));
}

void normalize_angle(float *a) {
  *a = atan2(sin(*a), cos(*a));
}

void rotate2D(point *in, float theta){
  point p = *in;
  in->x = cos(theta) * p.x - sin(theta) * p.y;
  in->y = sin(theta) * p.x + cos(theta) * p.y;
}

void generateMap(point* map){
  // map[0].x = 10;
  // map[0].y = 10;
  // map[1].x = 10;
  // map[1].y = 70;
  // map[2].x = 50;
  // map[2].y = 50;
  // map[3].x = 100;
  // map[3].y = 100;
  // map[4].x = 120;
  // map[4].y = 10;
  // map[5].x = 150;
  // map[5].y = 90;
  // map[6].x = 200;
  // map[6].y = 30;
  // map[7].x = 30;
  // map[7].y = 30;
  // map[8].x = 45;
  // map[8].y = 110;
  // map[9].x = 190;
  // map[9].y = 50;
  // map[10].x = 175;
  // map[10].y = 10;
  // map[11].x = 20;
  // map[11].y = 80;
  // map[12].x = 160;
  // map[12].y = 40;
  // map[13].x = 120;
  // map[13].y = 60;
  map[0].x = 10;
  map[0].y = 90;
  map[1].x = 40;
  map[1].y = 90;
  map[2].x = 70;
  map[2].y = 30;
  map[3].x = 100;
  map[3].y = 30;
  map[4].x = 160;
  map[4].y = 60;
  map[5].x = 160;
  map[5].y = 30;
  map[6].x = 220;
  map[6].y = 90;  
  map[7].x = 220;
  map[7].y = 60;

}
