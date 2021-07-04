import os
from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import scipy.linalg

def MultiRobotOptimizer(model, constraint, tp, steps):
    # initiate ocp
    ocp = AcadosOcp()
    ocp.model = model
    ocp.dims.N = steps

    # dimension number of state, input, trajectory, and terminal
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    # cost of Lagrange objective
    unscale = steps / tp
    Q = np.diag([1.0, 1.0, 1.0])
    R = np.diag([0.01, 0.01, 0.1, 0.1])
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)

    # cost of Mayer objective
    Qe = np.diag([1.0, 1.0, 1.0])
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W_e = Qe / unscale
    ocp.cost.Vx_e = np.zeros((ny_e, nx))
    ocp.cost.Vx_e[:nx, :nx] = np.eye(nx)

    # constraints of input and state
    ocp.constraints.lbu = np.array([constraint.v_min, constraint.v_min, constraint.v_min, constraint.v_min])
    ocp.constraints.ubu = np.array([constraint.v_max, constraint.v_max, constraint.v_max, constraint.v_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.lbx = np.array([constraint.xy_min, constraint.xy_min])
    ocp.constraints.ubx = np.array([constraint.xy_max, constraint.xy_max])
    ocp.constraints.idxbx = np.array([0, 1])

    # slack constraint
    nsbx = 2
    nsh = constraint.expr.shape[0]
    ns = nsh + nsbx
    ocp.cost.zl = 100 * np.ones((ns,))
    ocp.cost.zu = 100 * np.ones((ns,))
    ocp.cost.Zl = 1 * np.ones((ns,))
    ocp.cost.Zu = 1 * np.ones((ns,))
    ocp.constraints.lsbx = np.zeros([nsbx])
    ocp.constraints.usbx = np.zeros([nsbx])
    ocp.constraints.idxsbx = np.array(range(nsbx))

    # nonlinear constraint
    model.con_h_expr = constraint.expr
    ocp.constraints.lh = np.array(
        [
            constraint.obser_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            constraint.obser_max,
        ]
    )
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array(range(nsh))

    # initiate references
    ocp.constraints.x0 = np.zeros(nx)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # solver settings
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
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