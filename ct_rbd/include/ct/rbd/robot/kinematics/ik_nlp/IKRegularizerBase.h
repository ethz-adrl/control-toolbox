/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

class IKRegularizerBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	IKRegularizerBase() = default;
	virtual ~IKRegularizerBase() = default;

	virtual Eigen::MatrixXd computeRegularizer(const Eigen::VectorXd jointVal) = 0;
};

