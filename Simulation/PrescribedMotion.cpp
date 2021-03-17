#include "PrescribedMotion.h"
#include "Utils/Logger.h"
#include "RigidBody.h"
#include "ParticleData.h"

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

bool PrescribedMotion::isInTime(Real t)
{
    return (t >= m_startTime && t <= m_endTime);
}

void PrescribedMotion::evaluateRigidBodyStep(Real t, Real dt, RigidBody& rb)
{
    const Vector3r xi = rb.getPosition();
    const Vector3r vi = rb.getVelocity();

    te_variable vars[] = {{"t", &t}, {"dt", &dt}, 
						  {"x", &xi[0]}, {"y", &xi[1]}, {"z", &xi[2]},
						  {"vx", &vi[0]}, {"vy", &vi[1]}, {"vz", &vi[2]}};
    const int num_vars = 8;
    int err = -1;

    Vector3r newPos = rb.getPosition();

    if(evaluateVectorValue(m_prescribedTrajectory, vars, num_vars, newPos)) 
    {
        rb.getLastPosition() = rb.getOldPosition();
        rb.setOldPosition(xi);
        rb.setPosition(newPos);

        rb.getLastRotation() = rb.getOldRotation();
        rb.getOldRotation() = rb.getRotation();

        rb.setAngularVelocity(m_angularVelocity * m_rotationAxis);

        AngleAxisr rotation = AngleAxisr(m_angularVelocity * dt, m_rotationAxis);
        Quaternionr curr_rotation = rb.getRotation();
        curr_rotation *= Quaternionr(rotation);
        curr_rotation.normalize();
        rb.setRotation(curr_rotation);
        rb.rotationUpdated();
    }
}

void PrescribedMotion::evaluateParticleStep(Real t, Real dt, ParticleData& pd, unsigned int offset, unsigned int size)
{
    /*for (unsigned int i = offset; i < offset + size; i++)
    {
        const Vector3r xi = pd.getPosition(i);
        
        Eigen::ParametrizedLine<Real, 3> rotLine(m_supportPoint, m_rotationAxis.normalized());
        const Vector3r proj_xi = rotLine.projection(xi);
        const Vector3r ri = xi - rotLine.projection(xi);


    }*/
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
    if (expr == "")
        return true;
    
    int err = -1;
    te_expr *comp_expr = te_compile(expr.c_str(), vars, num_vars, &err);
    if (comp_expr) {
        val = static_cast<Real>(te_eval(comp_expr));
        te_free(comp_expr);
    }

    if (err != 0) 
    {
        LOG_ERR << "PrescribedMotion: expression for prescribed motion is errorneous";
        return false;
    }

    return true;
}