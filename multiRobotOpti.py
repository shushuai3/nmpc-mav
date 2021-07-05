import os
from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import scipy.linalg

def MultiRobotOptimizer(model, constraint, tp, steps):
    # initiate ocp
    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = steps

    # dimension
    nx = model.x.size()[0]
    ny = 1

    # nonlinear objective
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.W = np.diag([1])
    ocp.model.cost_y_expr = constraint.cost_y_expr

    # constraints of input and state
    ocp.constraints.lbu = np.array([constraint.v_min, constraint.v_min, constraint.v_min, constraint.v_min])
    ocp.constraints.ubu = np.array([constraint.v_max, constraint.v_max, constraint.v_max, constraint.v_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.lbx = np.array([constraint.xy_min, constraint.xy_min])
    ocp.constraints.ubx = np.array([constraint.xy_max, constraint.xy_max])
    ocp.constraints.idxbx = np.array([0, 1])

    # initiate references
    ocp.constraints.x0 = np.zeros(nx)
    ocp.cost.yref = np.zeros(ny)

    # solver settings
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4
    ocp.solver_options.tf = tp
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
    return acados_solver