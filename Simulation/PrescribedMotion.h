#ifndef __PRESCRIBED_MOTION_H__
#define __PRESCRIBED_MOTION_H__

#include "Common/Common.h"
#include "RigidBody.h"
#include "ParticleData.h"
#include "extern/tinyexpr/tinyexpr.h"

using namespace std::placeholders;

namespace PBD
{
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
            void evaluateRigidBodyStep(Real h, Real delta_h, RigidBody& rb);
            void evaluateParticleStep(Real h, Real delta_h, ParticleData& pd);

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

            std::function<void(Real, Real, RigidBody&)> getRigidBodyStep() { 
                return std::bind(
                    &PrescribedMotion::evaluateRigidBodyStep, this, _1, _2, _3
                ); 
            };
            std::function<void(Real, Real, ParticleData&)> getParticleStep() { 
                return std::bind(
                    &PrescribedMotion::evaluateParticleStep, this, _1, _2, _3
                ); 
            };
    };
}

#endif