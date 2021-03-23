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
    const Real startTime, const Real endTime, const std::string traj[3],
    const Real angVel, const Vector3r& rotAxis, 
    const Vector3r& suppPoint)
{
    m_startTime = startTime;
    m_endTime = endTime;

    for (int i = 0; i < 3; i++)
        m_prescribedTrajectory[i] = traj[i];

    m_angularVelocity = angVel;
    m_rotationAxis = rotAxis.normalized();
    m_supportPoint = suppPoint;
}

bool PrescribedMotion::isInTime(const Real t)
{
    return (t >= m_startTime && t <= m_endTime);
}

bool PrescribedMotion::evaluateVectorValue(const std::string expr[3], const te_variable vars[], const int num_vars, Vector3r& vec_val)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        if (!evaluateRealValue(expr[i], vars, num_vars, vec_val[i]))
            return false;
    }
    
    return true;
}

bool PrescribedMotion::evaluateRealValue(const std::string expr, const te_variable vars[], const int num_vars, Real& val)
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

void PrescribedMotion::rigidBodyStep(const Real t, const Real dt, RigidBody& rb)
{
    const Vector3r xi = rb.getPosition();
    const Eigen::Vector3d xi_double = xi.cast<double>();
    const double t_double = static_cast<double>(t);
    const double dt_double = static_cast<double>(dt);

    te_variable vars[] = {{"t", &t_double}, {"dt", &dt_double}, 
						  {"x", &xi_double[0]}, {"y", &xi_double[1]}, {"z", &xi_double[2]}};
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
}

void PrescribedMotion::particleStep(const Real t, const Real dt, ParticleData& pd, const unsigned int offset, const unsigned int size)
{
    // Move support vector of rotation axis
    const Vector3r supp_xi = m_supportPoint;
    const Eigen::Vector3d supp_xi_double = supp_xi.cast<double>();
    const double t_double = static_cast<double>(t);
    const double dt_double = static_cast<double>(dt);

    te_variable vars[] = {{"t", &t_double}, {"dt", &dt_double}, 
						  {"x", &supp_xi_double[0]}, {"y", &supp_xi_double[1]}, {"z", &supp_xi_double[2]}};
    const int num_vars = 5;
    Vector3r newSupportPoint = m_supportPoint;

    if(!evaluateVectorValue(m_prescribedTrajectory, vars, num_vars, newSupportPoint))
        return;

    const Vector3r translation = newSupportPoint - m_supportPoint;
    m_supportPoint = newSupportPoint;

    // Move particles along the rotation of the axis
    #pragma omp parallel if(size > MIN_PARALLEL_SIZE) default(shared)
	{
        #pragma omp for schedule(static) 
        for (unsigned int i = offset; i < offset + size; i++)
        {
            // Particles have moved by the translation
            const Vector3r& oldPosition = pd.getPosition(i);
            Vector3r xi = oldPosition + translation;
        
            // Step 1: Compute the normal vector ri of the line intersecting in our point
            Eigen::ParametrizedLine<Real, 3> rotLine(m_supportPoint, m_rotationAxis);
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

            pd.setVelocity(i, vel);
        }
    }
}