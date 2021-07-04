from casadi import *
from acados_template import AcadosModel

def multiRobotModel():
    # define model & constraint
    model = AcadosModel()
    constraint = types.SimpleNamespace()

    # control inputs
    vix = SX.sym('vix')
    viy = SX.sym('viy')
    vjx = SX.sym('vjx')
    vjy = SX.sym('vjy')
    u = vertcat(vix, viy, vjx, vjy)

    # relative states
    xij = SX.sym('xij')
    yij = SX.sym('yij')
    psiij = SX.sym('psiij')
    x = vertcat(xij, yij, psiij)

    # xdot
    dxij = SX.sym('dxij')
    dyij = SX.sym('dyij')
    dyawij = SX.sym('dyawij')
    xdot = vertcat(dxij, dyij, dyawij)

    # xdot function
    f_expl = vertcat(cos(psiij)*vjx - sin(psiij)*vjy - vix,
            sin(psiij)*vjx + cos(psiij)*vjy - viy,
            0)

    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = []
    model.p = []
    model.name = 'multiRobot'

    # state and input bounds
    constraint.xy_min = -10
    constraint.xy_max = 10
    constraint.v_max = 2
    constraint.v_min = -2

    detO = -2*(-vix*vjx*sin(psiij) + viy*vjx*cos(psiij) - vix*vjy*cos(psiij) - viy*vjy*sin(psiij))*(-vjx*yij*cos(psiij) + vjy*yij*sin(psiij) + vix*yij + vjx*xij*sin(psiij) + vjy*xij*cos(psiij) - viy*xij)
    constraint.expr = vcat([0.021/(0.001+detO)])
    return model, constraint