# Kidnapped Vehicle Particle Filter Project
This project is originally forked from https://github.com/udacity/CarND-Kidnapped-vehicle-Project. This repository includes starter code, that is used herein.

## Particle Filter Initialization
The strategy for the initial population of particles is to use the first GPS measurement (`x, y, theta` coordinates), and to randomly choose points around that measurement.  The selection is based on generating a random offset, based on a Gaussian probability distribution.

Additionally, with no observation information available initially, the particle weights are all set equal to `1.0`.

```C++

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
  /* Write to a file in order to post process in Matlab
  std::ofstream outFile("p.m", std::ofstream::out);
   */
  for(int i = 0; i < num_particles; ++i) {
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].id = 0; // No initial information for this parameter
    particles[i].weight = 1.0f; // Redundant with member "weights"
    weights[i] = 1.0f;
    pf_step = 0; // used for keeping track of iteration for post-processing only
	  
    /* Write to a file in order to post process in Matlab
    if (i == 0){
      outFile << "P{" << pf_step+1 << "} = [" << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ";...\n";
    } else if (i == (num_particles - 1)) {
      outFile << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << "];\n\n";
    } else {
      outFile << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << ";...\n";
    }
     */
    
  }
  /* Write to a file in order to post process in Matlab
  outFile.close();
   */
  is_initialized = true;

```
## Particle Motion Prediction
The prediction step applies a bicycle motion model (`v, theta, theta_dot` coordinates) to each of the particles, based on the control inputs.  The prediction algorithm using yaw rate (`theta_dot`) is not well defined (div-by-zero) when the yaw rate is very small.  Therefore, there is a method for the particle update for very small `theta_dot` and a different algorithm for not-so-small `theta_dot`, i.e. turning.

Uncertainty (noise) in the control signals are taken into account by adding a Gaussian noise term to each of the coordinates, after the motion update.  The basic assumption is that the uncertainty in each coordinate is independent of the other coordinates.

```C++
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
```

## Particle Probability Evaluation

In order to evaluate each particle, and update the probability to keep it in the resampling step, the observations (in car coordinate system) are compared the map landmarks (in map coordinate system).  In order to properly compare the two quantities, the observations for each particle must be translated into map coordinates.  Next, the Euclidean distance of each observation to the nearest landmark is calculated.  Finally, the Euclidean distances for all observations for a given particle are used to calculated the weight (relative probability) of that particle in the resample step.

```C++
// Assign an observation to a landmark
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
    // At the end of FOR loop for a given observation, update the weight of that observation
    obs_weights[o_idx] = (1/(2*M_PI*sig_x*sig_y))*exp(-(min_dist_x*min_dist_x)/(2*sig_x*sig_x) - (min_dist_y*min_dist_y)/(2*sig_y*sig_y));
      
    // Visualization for debugging - print intermediate weights
    //std::cout << "Particle #" << p_idx << ",\tObs #" << o_idx << ",\tWeight " << obs_weights[o_idx] << "\n";
  }
  // At the end of FOR loop on for a given particle, update the total weight of that particle
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
```

## Resampling Particles

Initially I implemented the resampling wheel method as demonstrated by Sebastian Thrun in the lesson material.  This method tended to spend far too much real-time stuck in the while loop step.  This may be due to the extremely small values for the particle weights, without using normalization.  Later, I implemented the method identified here: http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution.

The resampling takes a surprisingly small amount of code to implement.

```C++
std::vector<Particle> resampled_particles;
std::random_device rd;
std::mt19937 gen(rd());
std::discrete_distribution<> d(weights.begin(), weights.end());
	
for(int n=0; n<num_particles; ++n) {
  resampled_particles.push_back(particles[d(gen)]);
}
  
particles = resampled_particles;
```

## Results

The video below was generated by post-processing data in Matlab.  The size of the particles (blue circles) in the second frame is proportional to the number of times a particular particle appears in the total set.  The larger the particle, the more likely the algorithm considers this position to be correct.

[![Whoops, there should be a picture here!](https://img.youtube.com/vi/oENYCav-mQU/0.jpg)](https://youtu.be/oENYCav-mQU)

*Particle filter animation*

## Reflections

This project was an exercise in understanding C++ code and putting smaller functions into a larger program.  The actual particle filter design portion was surprisingly simple.  The only tunable parameter is the number of particles.  I chose a value of 100 initially, and this is able to pass the grader criteria, so I didn't need further tuning.

I was disappointed that there was no visualization tool provided as in the Kalman filter projects, so I wrote my own.  Including the time to write the [visualization script in Matlab](data/plot_p_and_gt.m), using Matlab toolboxes I had never used, this project took less than 10 hours to complete.
