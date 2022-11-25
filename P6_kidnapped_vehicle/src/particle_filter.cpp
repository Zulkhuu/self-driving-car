/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <map>

#include "helper_functions.h"

using std::string;
using std::vector;

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
    
  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  //TODO: this should be an init parameter or, at the least, a constant.
  particles.resize(num_particles);
  weights.resize(num_particles);

  std::normal_distribution<double> init_dist_x(x, std[0]);
  std::normal_distribution<double> init_dist_y(y, std[1]);
  std::normal_distribution<double> init_dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++) {
    particles[i].id = i;
    particles[i].x = init_dist_x(gen);
    particles[i].y = init_dist_y(gen);
    particles[i].theta = init_dist_theta(gen);
    particles[i].weight = 1.0;
    weights[i] = 1.0;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::normal_distribution<double> motion_noise_x(0.0, std_pos[0]);
  std::normal_distribution<double> motion_noise_y(0.0, std_pos[1]);
  std::normal_distribution<double> motion_noise_theta(0.0, std_pos[2]);

  const double c1 = (velocity / yaw_rate);
  const double yaw_rate_abs = fabs(yaw_rate);
  const double yaw_rate_change = yaw_rate * delta_t;

  for(auto& p : particles) {
    if (yaw_rate_abs < 0.0001) { // Avoid zero division, assume yaw_rate negligible
      p.x += velocity * cos(p.theta) * delta_t;
      p.y += velocity * sin(p.theta) * delta_t;
    } else {
      p.x += c1 * (sin(p.theta + yaw_rate_change) - sin(p.theta));
      p.y += c1 * (cos(p.theta) - cos(p.theta + yaw_rate_change));
      p.theta += yaw_rate_change;
    }

    p.x += motion_noise_x(gen);
    p.y += motion_noise_y(gen);
    p.theta += motion_noise_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
 
  for(auto& obs : observations) {
    int id = -1;
    double min_distance = 5000; // sensor_range(50)*100
    for(const auto& pred : predicted) {
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      if (distance < min_distance) {
        min_distance = distance;
        id = pred.id;
      }
    }
    obs.id = id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  const double sig_x = std_landmark[0];
  const double sig_y = std_landmark[1];

  double weights_sum = 0.0;
  for (auto& p : particles) {
    // Find landmarks that are in sensor range from the current particle's position
    vector<LandmarkObs> predicted;
    for(const auto& lm : map_landmarks.landmark_list) {
      double distance = dist(p.x, p.y, lm.x_f, lm.y_f);
      if(distance <= sensor_range)
        predicted.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
    }
    
    // Convert the observation measurements to map coordinate system
    vector<LandmarkObs> map_observations;
    for(const auto& obs : observations) {
      double x = (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y) + p.x;
      double y = (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y) + p.y;
      map_observations.push_back(LandmarkObs{obs.id, x, y});
    }

    // Find data association using nearest neighbor
    dataAssociation(predicted, map_observations);

    // Set weight for each particle using distance between observations and map landmarks
    p.weight = 1.0;
    for(const auto& obs : map_observations) {
      for(const auto& pred : predicted) {
        if (obs.id == pred.id) {
          p.weight *= multiv_prob(sig_x, sig_y, obs.x, obs.y, pred.x, pred.y);
        }
      }
    }
    weights_sum += p.weight;
  }

  // Normalize weights
  for(int i=0; i<num_particles; i++){
    particles[i].weight /= weights_sum;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    
  vector<Particle> new_particles;
  new_particles.resize(num_particles);

  std::discrete_distribution<> weighted_dist(weights.begin(), weights.end());
  for (int i = 0; i < num_particles; i++) {
    new_particles[i] = particles[weighted_dist(gen)];
  }

  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}