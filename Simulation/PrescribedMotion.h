#ifndef __PRESCRIBED_MOTION_H__
#define __PRESCRIBED_MOTION_H__

#include "Common/Common.h"
#include "extern/tinyexpr/tinyexpr.h"

using namespace std::placeholders;

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
            /** rotation axis */
            Vector3r m_rotationAxis;
            /** support vector for the rotation axis. Note that
             * this is only needed for entities without a local
             * coordinate system */
            Vector3r m_supportPoint;

        protected:
            bool evaluateVectorValue(std::string expr[3], te_variable vars[], int num_vars, Vector3r& vec_val);
            bool evaluateRealValue(std::string expr, te_variable vars[], int num_vars, Real &val);

        public:
            PrescribedMotion();
            ~PrescribedMotion();

            void initPrescribedMotion(
                Real startTime, Real endTime, std::string traj[3],
                Real angVel, Vector3r rotAxis, 
                Vector3r suppPoint
            );

            bool isInTime(Real t);

            void rigidBodyStep(Real t, Real dt, RigidBody& rb);
            void particleStep(Real t, Real dt, ParticleData& pd, unsigned int offset, unsigned int size);

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