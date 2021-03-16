#include "PrescribedMotion.h"

using namespace PBD;

PrescribedMotion::PrescribedMotion()
{
}

PrescribedMotion::~PrescribedMotion()
{
}

void PrescribedMotion::initPrescribedMotion(
    Real startTime, Real endTime, std::string traj[3],
    Real angVel, Vector3r rotAxis, 
    Vector3r suppPoint)
{
    m_startTime = startTime;
    m_endTime = endTime;

    for (int i = 0; i < 3; i++)
        m_prescribedTrajectory[i] = traj[i];

    m_angularVelocity = angVel;
    m_rotationAxis = rotAxis;
    m_supportPoint = suppPoint;
}

void PrescribedMotion::evaluateRigidBodyStep(Real h, Real delta_h, RigidBody& rb)
{
    const Vector3r xi = rb.getPosition();
    const Vector3r vi = rb.getVelocity();

    te_variable vars[] = {{"t", &h}, {"dt", &delta_h}, 
						  {"x", &xi[0]}, {"y", &xi[1]}, {"z", &xi[2]},
						  {"vx", &vi[0]}, {"vy", &vi[1]}, {"vz", &vi[2]}};
    const int num_vars = 8;
    int err = -1;

    Vector3r newPos = rb.getPosition();
    Vector3r newAngVel = rb.getAngularVelocity();

    if(evaluateVectorValue(m_prescribedTrajectory, vars, num_vars, newPos)) 
    {
        rb.setPosition(newPos);
        // TODO: Calculate current velocity and set it!
    }
}

void PrescribedMotion::evaluateParticleStep(Real h, Real delta_h, ParticleData& pd)
{

}

bool PrescribedMotion::evaluateVectorValue(std::string expr[3], te_variable vars[], int num_vars, Vector3r& vec_val)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        if (!evaluateRealValue(expr[i], vars, num_vars, vec_val[i]))
            return false;
    }
    
    return true;
}

bool PrescribedMotion::evaluateRealValue(std::string expr, te_variable vars[], int num_vars, Real& val)
{
    int err = -1;

    te_expr *comp_expr = te_compile(expr.c_str(), vars, num_vars, &err);
    if (comp_expr)
        val = static_cast<Real>(te_eval(comp_expr));

    if (err != 0) 
    {
        LOG_ERR << "PrescribedMotion: expression for prescribed motion is errorneous";
        return false;
    }

    return true;
}