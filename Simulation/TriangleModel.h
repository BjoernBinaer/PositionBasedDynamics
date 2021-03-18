#ifndef __TRIANGLEMODEL_H__
#define __TRIANGLEMODEL_H__

#include "Common/Common.h"
#include <vector>
#include "Simulation/RigidBody.h"
#include "Utils/IndexedFaceMesh.h"
#include "Simulation/ParticleData.h"
#include "Constraints.h"
#include "PrescribedMotion.h"

namespace PBD 
{	
	class TriangleModel
	{
		public:
			TriangleModel();
			virtual ~TriangleModel();

			typedef Utilities::IndexedFaceMesh ParticleMesh;

		protected:
			/** offset which must be added to get the correct index in the particles array */
			unsigned int m_indexOffset;
			/** Face mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;
			Real m_restitutionCoeff;
			Real m_frictionCoeff;

			std::vector<PrescribedMotion*> m_prescribedMotionVector;

		public:
			void updateConstraints();

			ParticleMesh &getParticleMesh();
			void cleanupModel();

			unsigned int getIndexOffset() const; 

			void initMesh(const unsigned int nPoints, const unsigned int nFaces, const unsigned int indexOffset, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs);
			void updateMeshNormals(const ParticleData &pd);

			void addPrescribedMotion(Real startTime, Real endTime, std::string traj[3], Real angVel, Vector3r rotAxis, Vector3r suppVec);
			bool checkForPrescribedMotion(Real t, ParticleData &pd);

			void applyCurrentPrescribedMotion(Real t, Real delta_t, ParticleData& pd);

			FORCE_INLINE Real getRestitutionCoeff() const
			{
				return m_restitutionCoeff;
			}

			FORCE_INLINE void setRestitutionCoeff(Real val)
			{
				m_restitutionCoeff = val;
			}

			FORCE_INLINE Real getFrictionCoeff() const
			{
				return m_frictionCoeff;
			}

			FORCE_INLINE void setFrictionCoeff(Real val)
			{
				m_frictionCoeff = val;
			}
	};
}

#endif