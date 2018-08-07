// covariance_matrix.h
// Computes incremental means, variances, and covariances.

// Ryker Dial
// Date Created: October 3, 2017
// Last Modified: October 3, 2017

#ifndef COVARIANCE_MATRIX
#define COVARIANCE_MATRIX

#include <vector>

class covarianceMatrix {

	private:
		uint64_t n_; // Sample Counter
		double avg_x_, avg_y_, avg_z_;
		double old_avg_x_, old_avg_y_, old_avg_z_;
		double sum_var_x_, sum_var_y_, sum_var_z_;
		double var_x_, var_y_, var_z_;
		double cov_xy_, cov_yz_, cov_xz_;

	public:

		covarianceMatrix()
			: n_(0), avg_x_(0), avg_y_(0), avg_z_(0), old_avg_x_(0), old_avg_y_(0), old_avg_z_(0),
			  sum_var_x_(0), sum_var_y_(0), sum_var_z_(0), var_x_(0), var_y_(0), var_z_(0),
			  cov_xy_(0), cov_yz_(0), cov_xz_(0)
		{}

		void push(double x, double y, double z) {
			++n_;

			// Source for Average and Variance Calculations:
			// http://people.ds.cam.ac.uk/fanf2/hermes/doc/antiforgery/stats.pdf

			// ***** Calculate Cumulative Moving Average ***** //
			// Save the old averages for the variance calculations
			old_avg_x_ = avg_x_;
			old_avg_y_ = avg_y_;
			old_avg_z_ = avg_z_;

			// Calculate new averages
			avg_x_ = avg_x_ + (x - avg_x_)/n_;
			avg_y_ = avg_y_ + (y - avg_y_)/n_;
			avg_z_ = avg_z_ + (z - avg_z_)/n_;
			// *********** //

			// ***** Calculate Variances ***** //
			// Calculate sums, where sum = n*sigma_n^2
			sum_var_x_ = sum_var_x_ + (x-old_avg_x_)*(x-avg_x_);
			sum_var_y_ = sum_var_y_ + (y-old_avg_y_)*(y-avg_y_);
			sum_var_z_ = sum_var_z_ + (z-old_avg_z_)*(z-avg_z_);

			// Calculate new variances
			var_x_ = sum_var_x_/n_;
			var_y_ = sum_var_y_/n_;
			var_z_ = sum_var_z_/n_;
			// ********** //

			// ***** Calculate Covariances ****** //
			// Source: https://knaap.com/data/recursive_covariance.pdf
			if(n_ > 1) {
				cov_xy_ = (n_-1)/(1.0*n_)*cov_xy_ + 1.0/(n_-1)*(x-avg_x_)*(y-avg_y_);
				cov_yz_ = (n_-1)/(1.0*n_)*cov_yz_ + 1.0/(n_-1)*(y-avg_y_)*(z-avg_z_);
				cov_xz_ = (n_-1)/(1.0*n_)*cov_xz_ + 1.0/(n_-1)*(x-avg_x_)*(z-avg_z_);
			}
			// ********** //
		}

		// Get covariance matrix in row-major ordering
		std::vector<double> getCovMatrix() {
			std::vector<double> cov_matrix_;
			cov_matrix_.reserve(9);
			cov_matrix_.push_back(var_x_);
			cov_matrix_.push_back(cov_xy_);
			cov_matrix_.push_back(cov_xz_);
			cov_matrix_.push_back(cov_xy_);
			cov_matrix_.push_back(var_y_);
			cov_matrix_.push_back(cov_yz_);
			cov_matrix_.push_back(cov_xz_);
			cov_matrix_.push_back(cov_yz_);
			cov_matrix_.push_back(var_z_);

			return cov_matrix_;
		}
};

#endif