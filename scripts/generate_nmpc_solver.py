import os
import sys
import numpy as np
import casadi as cs
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

def export_antenna_tracker_model() -> AcadosModel:
    model_name = 'antenna_tracker'

    # States
    az = cs.SX.sym('az')
    az_dot = cs.SX.sym('az_dot')
    el = cs.SX.sym('el')
    el_dot = cs.SX.sym('el_dot')
    x = cs.vertcat(az, az_dot, el, el_dot)

    # Controls
    u_az = cs.SX.sym('u_az')
    u_el = cs.SX.sym('u_el')
    u = cs.vertcat(u_az, u_el)

    # Dynamics
    # Model: acceleration is controlled by u, with gravity affecting elevation
    # u is broadly treated as an acceleration command since scale is arbitrary
    f_expl = cs.vertcat(
        az_dot,
        u_az,
        el_dot,
        u_el - 10.77 * cs.cos(el)
    )

    xdot = cs.vertcat(cs.SX.sym('az_d'), cs.SX.sym('az_dd'), cs.SX.sym('el_d'), cs.SX.sym('el_dd'))
    f_impl = xdot - f_expl

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model

def generate_c_code():
    ocp = AcadosOcp()
    ocp.model = export_antenna_tracker_model()

    Tf = 1.0  # Prediction horizon (seconds)
    N = 20    # Number of shooting intervals
    ocp.solver_options.N_horizon = N

    # Cost formulation
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    nx = 4
    nu = 2
    ny = nx + nu
    ny_e = nx

    # Weights: Q and R matrices
    Q = np.diag([50.0, 1.0, 50.0, 1.0])
    R = np.diag([0.01, 0.01])
    W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W = W
    ocp.cost.W_e = Q

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :] = np.eye(nu)

    ocp.cost.Vx_e = np.eye(nx)
    
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Constraints
    ocp.constraints.lbu = np.array([-1000.0, -1000.0])
    ocp.constraints.ubu = np.array([1000.0, 1000.0])
    ocp.constraints.idxbu = np.array([0, 1])

    # Elevation limits (0 to 90 deg -> 0 to pi/2 rad)
    # We add a small buffer for tracking stability
    ocp.constraints.lbx = np.array([-0.1])
    ocp.constraints.ubx = np.array([1.6])
    ocp.constraints.idxbx = np.array([2])

    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0])

    # Solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # Real Time Iteration
    ocp.solver_options.tf = Tf

    # Export dir
    export_dir = os.path.join(os.getcwd(), 'c_generated_code')
    
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json', build=True, generate=True)
    print("Acados NMPC C code generated successfully.")

if __name__ == '__main__':
    generate_c_code()
