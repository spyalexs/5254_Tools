#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"


class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		MyPotentialFunction(Eigen::Vector2d Goal, double d_star, double Zetta, double Q_star, double Eta, std::vector<amp::Obstacle2D> Obs);

		// Returns the potential function value (height) for a given 2D point. 
        virtual double operator()(const Eigen::Vector2d& q) const override {
            return q[0] * q[0] + q[1] * q[1];
        }

		virtual Eigen::Vector2d getGradient(const Eigen::Vector2d& q) const override;

		Eigen::Vector2d getClosestPointToObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obs) const;

		Eigen::Vector2d findClosestPoint(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d goal) const;



	private: 
		Eigen::Vector2d goal;
		double d_star;
		double zetta;
		double q_star;
		double eta;
		std::vector<amp::Obstacle2D> obstacles;
};

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {};

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;

		MyPotentialFunction *func;
	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};
