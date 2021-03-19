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

void PrescribedMotion::rigidBodyStep(Real t, Real dt, RigidBody& rb)
{
    const Vector3r xi = rb.getPosition();

    te_variable vars[] = {{"t", &t}, {"dt", &dt}, 
						  {"x", &xi[0]}, {"y", &xi[1]}, {"z", &xi[2]}};
    const int num_vars = 5;

    Vector3r newPos = rb.getPosition();

    if(!evaluateVectorValue(m_prescribedTrajectory, vars, num_vars, newPos)) 
        return;

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

    //AngleAxisr test_arr(curr_rotation);
    //LOG_DEBUG << "Current rotation angle: " << m_angularVelocity * dt;
    //LOG_DEBUG << "Rotated amount: " << test_arr.angle() << ", Axis: " << test_arr.axis();
}

void PrescribedMotion::particleStep(Real t, Real dt, ParticleData& pd, unsigned int offset, unsigned int size)
{
    // Move support vector of rotation axis
    const Vector3r supp_xi = m_supportPoint;

    te_variable vars[] = {{"t", &t}, {"dt", &dt}, 
						  {"x", &supp_xi[0]}, {"y", &supp_xi[1]}, {"z", &supp_xi[2]}};
    const int num_vars = 5;
    Vector3r newSupportPoint = m_supportPoint;

    if(!evaluateVectorValue(m_prescribedTrajectory, vars, num_vars, newSupportPoint))
        return;

    const Vector3r translation = newSupportPoint - m_supportPoint;

    //LOG_DEBUG << "Current Translation: " << translation;

    m_supportPoint = newSupportPoint;

    // Move particles along the rotation of the axis
    for (unsigned int i = offset; i < offset + size; i++)
    {
        // Particles have moved by the translation
        const Vector3r& oldPosition = pd.getPosition(i);
        Vector3r xi = oldPosition + translation;
        
        // Step 1: Compute the normal vector ri of the line intersecting in our point
        Eigen::ParametrizedLine<Real, 3> rotLine(m_supportPoint, m_rotationAxis.normalized());
        const Vector3r proj_xi = rotLine.projection(xi);
        const Vector3r ri = xi - proj_xi;

        // Step 2: Rotate ri around the axis
        Matrix3r rotation = AngleAxisr(m_angularVelocity * dt, m_rotationAxis).toRotationMatrix();
        const Vector3r ri_new = rotation * ri;

        // Step 3: Recover tangential component
        xi = proj_xi + ri_new;
        pd.setPosition(i, xi);

        // Calculate rough velocities:
        const Vector3r transl_vel = translation / dt;
        const Vector3r rot_vel = (m_angularVelocity * m_rotationAxis).cross(ri_new);
        const Vector3r vel = transl_vel + rot_vel;
        
        LOG_DEBUG << vel;

        pd.setVelocity(i, vel);
    }
}