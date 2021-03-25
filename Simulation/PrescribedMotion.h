#ifndef __PRESCRIBED_MOTION_H__
#define __PRESCRIBED_MOTION_H__

#include "Common/Common.h"
#include "extern/tinyexpr/tinyexpr.h"

namespace PBD
{
    class RigidBody;
    class ParticleData;
    
    class PrescribedMotion
    {
        private:
            /** start of the prescribed motion */
		    Real m_startTime;
		    /** end of the prescribed motion */
		    Real m_endTime;

            /** string expression of prescribed motion */
            std::string m_prescribedTrajectory[3];
            /** angular velocity of prescribed rotation */
            Real m_angularVelocity;
            /** rotation axis (normalized) */
            Vector3r m_rotationAxis;
            /** support vector for the rotation axis. Note that
             * this is only needed for entities without a local
             * coordinate system */
            Vector3r m_supportPoint;

        protected:
            bool evaluateVectorValue(const std::string expr[3], const te_variable vars[], const int num_vars, Vector3r& vec_val);
            bool evaluateRealValue(const std::string expr, const te_variable vars[], const int num_vars, Real &val);

        public:
            PrescribedMotion();
            ~PrescribedMotion();

            void initPrescribedMotion(
                const Real startTime, const Real endTime, const std::string traj[3],
                const Real angVel, const Vector3r& rotAxis, 
                const Vector3r& suppPoint
            );

            bool isInTime(const Real t);

            void rigidBodyStep(const Real t, const Real dt, RigidBody& rb);
            void particleStep(const Real t, const Real dt, ParticleData& pd, const int offset, const int size);

            FORCE_INLINE const Real getStartTime() const
            {
                return m_startTime;
            }

            FORCE_INLINE const Real getEndTime() const
            {
                return m_endTime;
            }
    };
}

#endif