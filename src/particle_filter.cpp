/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Visualization for debugging - print initial point
  //std::cout << "Initial data:\n\tx: " << x << "\t y: " << y << "\t theta: " << theta << "\n\n";
  
  // Choose number of particles
  num_particles = 100;
  particles.reserve(num_particles); // Set size of yet-to-be-populated array
  weights.resize(num_particles);
  
  // From lesson 14.4
  // Create random noise distributions around each dimension
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
  
  // Initialize each particle based on noisy measurement data
  for(int i = 0; i < num_particles; ++i) {
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].id = 0; // No initial information for this parameter
    particles[i].weight = 1.0f; // Redundant with member "weights"
    weights[i] = 1.0f;
	  
    // Visualization for debugging - print all particles
    //std::cout << "P[" << i << "]:\tx: " << particles[i].x << "\t y: " << particles[i].y << "\t theta: " << particles[i].theta << "\n";
  }
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  
  // From lesson 14.4
  // Create random noise distributions around each dimension
  // Use mean = 0 since this is additive (faster than generating a distribution for every particle's coordinates)
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(0.0f, std_pos[0]);
  std::normal_distribution<double> dist_y(0.0f, std_pos[1]);
  std::normal_distribution<double> dist_theta(0.0f, std_pos[2]);
  
  // From lesson 12.3
  // Apply bicycle motion model
  if(fabs(yaw_rate) > 0.001) {
    // Use calculus to update position
    for(int i = 0; i < num_particles; ++i) {
      particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate*delta_t;
      
      // Add motion model uncertainty
      particles[i].x += dist_x(gen);
      particles[i].y += dist_y(gen);
      particles[i].theta += dist_theta(gen);
    }
  }
  else {
    // Use geometry to update position
    for(int i = 0; i < num_particles; ++i) {
      particles[i].x += (velocity*delta_t)*(cos(particles[i].theta));
      particles[i].y += (velocity*delta_t)*(sin(particles[i].theta));
      // no need to update theta, raw rate too small (theta_t = theta_0)
      
      // Add motion model uncertainty
      particles[i].x += dist_x(gen);
      particles[i].y += dist_y(gen);
      particles[i].theta += dist_theta(gen);
    }
  }
}

/*void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Unused

}*/

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  
  // From https://discussions.udacity.com/t/implementing-data-association/243745/2
  
  int num_landmarks = map_landmarks.landmark_list.size();
  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];
  // Visualization for debugging - print the number of landmarks
  //std::cout << "Number of landmarks: " << num_landmarks << "\n\n";
  
  int num_observations = observations.size();
  std::vector<double> obs_weights;
  obs_weights.resize(num_observations);

  // Visualization for debugging - print the number of observations
  //std::cout << "Number of observations: " << num_observations << "\n\n";
  
  double obs_x, obs_y, dist_x, dist_y, dist, min_dist_x, min_dist_y, min_dist;
  max_weight = 0.0f;
  
  // Assign and observation to a landmark
  for(int p_idx = 0; p_idx < num_particles; ++p_idx ){
    // Perform the update for each particle
    // Use temp variables for readability
    double px = particles[p_idx].x;
    double py = particles[p_idx].y;
    double theta = particles[p_idx].theta;
    
    // reset weights vector for this particle
    std::fill(obs_weights.begin(), obs_weights.end(), 0.0f);
    
    for(int o_idx = 0; o_idx < num_observations; ++o_idx) {
      // Transform each observation to the map coordinate system
      obs_x = px + observations[o_idx].x*cos(theta) - observations[o_idx].y*sin(theta);
      obs_y = py + observations[o_idx].x*sin(theta) + observations[o_idx].y*cos(theta);
      
      // Re-initialize the minimum distance for this observation
      min_dist_x = 1e10f;
      min_dist_y = 1e10f;
      min_dist = 1e10f;
      
      for (int l_idx = 0; l_idx < num_landmarks; ++l_idx) {
        // Find the distance for each observation and compare it to the minimum distance
        dist_x = map_landmarks.landmark_list[l_idx].x_f - obs_x;
        dist_y = map_landmarks.landmark_list[l_idx].y_f - obs_y;
        dist = sqrt(dist_x*dist_x + dist_y*dist_y);
        
        if (dist < min_dist) {
          // Update minimum distance and choose this landmark
          min_dist_x = dist_x;
          min_dist_y = dist_y;
          min_dist = dist;
          observations[o_idx].id = l_idx;
          
          // Visualization for debugging - print that this logic was evaluated
          //std::cout << "Updated an observation's landmark\n";
          //std::cout << "P: " << p_idx << "\tO: " << o_idx << "\t L: " << l_idx << "\t";
          //std::cout << "D: " << dist << "\n";
        }
      }
      // Still inside FOR loop on observation o_idx, update the weight of a single observation
      obs_weights[o_idx] = (1/(2*M_PI*sig_x*sig_y))*exp(-(min_dist_x*min_dist_x)/(2*sig_x*sig_x) - (min_dist_y*min_dist_y)/(2*sig_y*sig_y));
      
      // Visualization for debugging - print intermediate weights
      //std::cout << "Particle #" << p_idx << ",\tObs #" << o_idx << ",\tWeight " << obs_weights[o_idx] << "\n";
    }
    // Still inside FOR loop on particle p_idx, update the total particle weight
    weights[p_idx] = 1.0f;
    for (int i = 0; i < num_observations; ++i) {
      //particles[p_idx].weight *= obs_weights[i];
      weights[p_idx] *= obs_weights[i];
    }
    particles[p_idx].weight = weights[p_idx];
    if (weights[p_idx] > max_weight) {
      max_weight = weights[p_idx];
    }
    // Visualization for debugging - print particle weights
    //std::cout << "Particle #" << p_idx << ": " << particles[p_idx].weight << "\n";
    //std::cout << "Particle #" << p_idx << ": " << weights[p_idx] << "\n";
    
  }
  
  
}

void ParticleFilter::resample() {
  
  // From http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  std::vector<Particle> resampled_particles;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
	
  for(int n=0; n<num_particles; ++n) {
    resampled_particles.push_back(particles[d(gen)]);
  }
  
  particles = resampled_particles;
  
  /* Resample wheel algorithm from lesson 13.20 seems to get stuch in while loop
     weights may be too small 1e-100, 1e-300 for this algorithm
     */

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
