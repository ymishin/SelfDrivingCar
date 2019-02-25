#include "particle_filter.h"

static const double eps = 1e-5;

const unsigned int seed = 777;
std::mt19937 gen(seed);

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  num_particles_ = 100;

  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  particles_.reserve(num_particles_);
  for (unsigned i = 0; i < num_particles_; ++i) {
    particles_.push_back(Particle(i, dist_x(gen), dist_y(gen), dist_theta(gen)));
  }
  
  weights_.reserve(num_particles_);
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  std::normal_distribution<double> dist_x(0.0, std_pos[0]);
  std::normal_distribution<double> dist_y(0.0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (auto &p : particles_) {
    if (yaw_rate > eps) {
      p.x_ += velocity / yaw_rate * (sin(p.theta_ + yaw_rate * delta_t) - sin(p.theta_)) + dist_x(gen);
      p.y_ += velocity / yaw_rate * (cos(p.theta_)  - cos(p.theta_ + yaw_rate * delta_t)) + dist_y(gen);
    } 
    else {
      p.x_ += velocity * delta_t * cos(p.theta_) + dist_x(gen);
      p.y_ += velocity * delta_t * sin(p.theta_) + dist_x(gen);
    }
    p.theta_ += yaw_rate * delta_t + dist_theta(gen);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  
  weights_.clear();

  for (auto &p : particles_) {
    
    p.weight_ = 1.;
    
    for (const auto &obs : observations) {
      
      // Transform to map coordinate system
      double x_map = p.x_ + cos(p.theta_) * obs.x - sin(p.theta_) * obs.y;
      double y_map = p.y_ + sin(p.theta_) * obs.x + cos(p.theta_) * obs.y;

      // Find nearest landmark
      unsigned closest_landmark_ind = -1;
      double min_dist = std::numeric_limits<double>::infinity();
      double curr_dist;
      for (unsigned i = 0; i < map_landmarks.landmark_list.size(); ++i) {
        const auto &lm = map_landmarks.landmark_list[i];
        if (dist(p.x_, p.y_, lm.x_f, lm.y_f) > sensor_range) continue;
        curr_dist = dist(x_map, y_map, lm.x_f, lm.y_f);
        if (curr_dist < min_dist) {
          min_dist = curr_dist;
          closest_landmark_ind = i;
        }
      }

      // Update particle weight
      if (closest_landmark_ind == -1) {
        p.weight_ *= multivProb(std_landmark[0], std_landmark[1], x_map, y_map,
                                std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
      }
      else {
        const auto &lm = map_landmarks.landmark_list[closest_landmark_ind];
        p.weight_ *= multivProb(std_landmark[0], std_landmark[1], x_map, y_map, lm.x_f, lm.y_f);
      }
    }

    weights_.push_back(p.weight_);
  }
}

void ParticleFilter::resample() {
  
  std::discrete_distribution<int> dist_res(weights_.begin(), weights_.end());

  std::vector<Particle> resampled_particles;
  resampled_particles.reserve(num_particles_);
  for (unsigned i = 0; i < num_particles_; ++i) {
    resampled_particles.push_back(particles_[dist_res(gen)]);
  }
  particles_.swap(resampled_particles);
}

void ParticleFilter::setAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations_ = associations;
  particle.sense_x_ = sense_x;
  particle.sense_y_ = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations_;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x_;
  } else {
    v = best.sense_y_;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

double ParticleFilter::multivProb(double sig_x, double sig_y, double x_obs, double y_obs, double mu_x, double mu_y) {

  double gauss_norm = 1. / (2. * M_PI * sig_x * sig_y);
  double exponent = ((x_obs - mu_x) * (x_obs - mu_x) / (2. * sig_x * sig_x))
                  + ((y_obs - mu_y) * (y_obs - mu_y) / (2. * sig_y * sig_y));

  return gauss_norm * exp(-exponent);
}