/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *  Edited on: Nov 17 2017
 *      Author: Andrei Feklistov
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
    
    // set number of particles
    num_particles = 30;

    // extract standard deviations
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    // prepare random generator and x, y, theta ditributions to randomly pick from
    std::default_random_engine gen;
    std::normal_distribution<> dist_x(x, std_x);
    std::normal_distribution<> dist_y(y, std_y);
    std::normal_distribution<> dist_theta(theta, std_theta);
    
    // generate all initial particles
    for (int i = 0; i < num_particles; i++)
    {
        Particle particle;

        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1;

        particles.push_back(particle);
        weights.push_back(particle.weight);
    }

    // set flag to true
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
{
	// Add measurements to each particle and add random Gaussian noise.
	
    // extract standard deviations
    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    // prepare random generator and x, y, theta ditributions to randomly pick from
    std::default_random_engine gen;
    std::normal_distribution<> dist_x(0, std_x);
    std::normal_distribution<> dist_y(0, std_y);
    std::normal_distribution<> dist_theta(0, std_theta);

    // iterate through particles
    for (int i = 0; i < num_particles; i++)
    {
        double theta = particles[i].theta;

        // predict x, y and theta. avoid division by zero
        if (fabs(yaw_rate) > 0.001) 
        {
            particles[i].x += (velocity / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta));
            particles[i].y += (velocity / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }
        else 
        {
            particles[i].x += velocity * delta_t * cos(theta);
            particles[i].y += velocity * delta_t * sin(theta);
            // no change for theta
        }

        // add noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }   
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        const std::vector<LandmarkObs> &observations, const Map &map_landmarks) 
{
	// Update the weights of each particle using a mult-variate Gaussian distribution. 
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Particles are located
	//   according to the MAP'S coordinate system. We need to transform between the two systems.

    // standard deviations
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];

    // for debug tool in simulator (blue lines from particle to landmarks) set three vectors
    std::vector<int> associations;
    int indx; // for saving landmark's index
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    // iterate trough particles
    for (int i = 0; i < num_particles; i++)
    {
        // preapare variables for multivariate Gaussian calculations
        double normalization_term = 1 / (2 * M_PI * std_x * std_y);
        double multivar_gauss = 1;
        
        // go through each observation
        for (int j = 0; j < observations.size(); j++)
        {
            // transform observed vehicle's x and y to map's x and y
            double x_obs_to_map, y_obs_to_map;
            x_obs_to_map = particles[i].x + cos(particles[i].theta) * observations[j].x - sin(particles[i].theta) * observations[j].y;
            y_obs_to_map = particles[i].y + cos(particles[i].theta) * observations[j].y + sin(particles[i].theta) * observations[j].x;

            // use observations only in sensor range
            double obs_range = dist(particles[i].x, particles[i].y, x_obs_to_map, y_obs_to_map);
            if (obs_range <= sensor_range)
            {
                // assosiate observation with closest landmark
                double min_dist = 1000000000; // set some big number for a start
                double mu_x, mu_y; // for later use, mu_x and mu_y (it is the closest landmark's x and y)

                for (int k = 0; k < map_landmarks.landmark_list.size(); k++)
                {
                    double x_landmark = map_landmarks.landmark_list[k].x_f;
                    double y_landmark = map_landmarks.landmark_list[k].y_f;

                    // find closest landmark
                    double distance = dist(x_obs_to_map, y_obs_to_map, x_landmark, y_landmark);
                    if (distance < min_dist)
                    {
                        min_dist = distance;
                        mu_x = map_landmarks.landmark_list[k].x_f;
                        mu_y = map_landmarks.landmark_list[k].y_f;

                        // save index for associations vector
                        indx = map_landmarks.landmark_list[k].id_i;
                    }
                }

                // calculate multivariate Gaussian probability
                // prepare exponent part
                double exponent = pow((x_obs_to_map - mu_x), 2) / (2 * pow(std_x, 2)) + pow((y_obs_to_map - mu_y), 2) / (2 * pow(std_y, 2));

                // calculate mvG
                multivar_gauss *= normalization_term * exp(-exponent);

                // save observation's landmark id and measured x and y
                associations.push_back(indx);
                sense_x.push_back(x_obs_to_map);
                sense_y.push_back(y_obs_to_map);
            }
        }

        // update particle's weight
        particles[i].weight = multivar_gauss;
        weights[i] = multivar_gauss;

        SetAssociations(particles[i], associations, sense_x, sense_y);
    }
}

void ParticleFilter::resample() 
{
	// Resample particles with replacement with probability proportional to their weight. 
	
    // prepare random generator and ditribution to randomly pick from
    std::default_random_engine gen;
    std::discrete_distribution<int> weighted_particles(weights.begin(), weights.end());

    // store resampled particles here
    std::vector<Particle> resampled;

    // resample according to weights
    for (int i = 0; i < num_particles; i++)
    {
        int index = weighted_particles(gen);
        resampled.push_back(particles[index]);
    }

    particles = resampled;
}

void ParticleFilter::SetAssociations(Particle &particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	// particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
