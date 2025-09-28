/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_cost.cc
 */

#include "planning/open_space/ilqr_smoother/al_cost.h"

void ALCost::SetConstraint(const std::shared_ptr<Constrain>& constrain) {
  constrains_.push_back(constrain);
}

void ALCost::EvaluateDerivative(
    const Eigen::Ref<const Eigen::VectorXd>& xu,
    const Eigen::Ref<const Eigen::VectorXd>& xu_ref) {
  // jac 6 * 1
  Eigen::VectorXd jac = 2 * weight_ * (xu - xu_ref);
  // hess 6 * 6
  Eigen::MatrixXd hess = 2 * weight_;
  for (const auto& constrain : constrains_) {
    constrain->Evaluate(xu);
    constrain->Jacobian(xu);
    jac += 2.0 * constrain->GetJac().transpose() *
           (constrain->GetLambda() + constrain->GetI() * constrain->GetEval());
    hess += 2.0 * constrain->GetJac().transpose() * constrain->GetI() *
            constrain->GetJac();
  }
  dx_ = jac.topRows(4);
  du_ = jac.bottomRows(2);
  ddx_ = hess.topLeftCorner(4, 4);
  ddu_ = hess.bottomRightCorner(2, 2);
}

double ALCost::CalculateConstraintsCost(
    const Eigen::Ref<const Eigen::VectorXd>& xu) {
  double cost = 0.0;
  for (const auto& constrain : constrains_) {
    constrain->Evaluate(xu);
    constrain->Jacobian(xu);
    cost += (constrain->GetLambda().transpose() * constrain->GetEval() +
             constrain->GetEval().transpose() * constrain->GetI() *
                 constrain->GetEval())(0, 0);
  }
  return cost;
}

void ALCost::UpdateDualVar(const Eigen::Ref<const Eigen::VectorXd>& xu) {
  for (const auto& constrain : constrains_) {
    constrain->UpdateDual(xu);
  }
}
