#include "TriangleModel.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;

TriangleModel::TriangleModel() :
	m_particleMesh()
{
	m_restitutionCoeff = static_cast<Real>(0.6);
	m_frictionCoeff = static_cast<Real>(0.2);
}

TriangleModel::~TriangleModel(void)
{
	cleanupModel();
}

void TriangleModel::cleanupModel()
{
	m_particleMesh.release();

	for (int i = 0; i < m_prescribedMotionVector.size(); i++)
		delete m_prescribedMotionVector[i];
	m_prescribedMotionVector.clear();
}

void TriangleModel::updateMeshNormals(const ParticleData &pd)
{
	m_particleMesh.updateNormals(pd, m_indexOffset);
	m_particleMesh.updateVertexNormals(pd);
}

TriangleModel::ParticleMesh &TriangleModel::getParticleMesh()
{
	return m_particleMesh;
}

void TriangleModel::initMesh(const unsigned int nPoints, const unsigned int nFaces, const unsigned int indexOffset, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs)
{
	m_indexOffset = indexOffset;
	m_particleMesh.release();

	m_particleMesh.initMesh(nPoints, nFaces * 2, nFaces);

	for (unsigned int i = 0; i < nFaces; i++)
	{
		m_particleMesh.addFace(&indices[3 * i]);
	}
	m_particleMesh.copyUVs(uvIndices, uvs);
	m_particleMesh.buildNeighbors();
}

unsigned int TriangleModel::getIndexOffset() const
{
	return m_indexOffset;
}

void TriangleModel::addPrescribedMotion(const Real startTime, const Real endTime, const std::string traj[3], const Real angVel, const Vector3r& rotAxis, const Vector3r& suppVec)
{
	PrescribedMotion* pm = new PrescribedMotion();
	pm->initPrescribedMotion(startTime, endTime, traj, angVel, rotAxis, suppVec);
	m_prescribedMotionVector.push_back(pm);
}

bool TriangleModel::checkForPrescribedMotion(const Real t, ParticleData &pd)
{
	const unsigned int offset = m_indexOffset;
	const unsigned int nParticles = m_particleMesh.numVertices();

	bool animated = std::find_if(m_prescribedMotionVector.begin(), m_prescribedMotionVector.end(), 
									[t] (PrescribedMotion* pm) { return pm->isInTime(t); })
									!= m_prescribedMotionVector.end();

	#pragma omp parallel if(nParticles > MIN_PARALLEL_SIZE) default(shared)
	{
        #pragma omp for schedule(static) 
		for (unsigned int i = offset; i < offset + nParticles; i++)
		{
			if (animated)
				pd.setParticleState(i, ParticleState::Animated);
			else if (pd.getMass(i) != 0.0)
				pd.setParticleState(i, ParticleState::Simulated);
			else 
				pd.setParticleState(i, ParticleState::Fixed);
		}
	}

	return animated;
}

void TriangleModel::applyCurrentPrescribedMotion(const Real t, const Real delta_t, ParticleData& pd)
{
	const unsigned int offset = m_indexOffset;
	const unsigned int nParticles = m_particleMesh.numVertices();

	// Get current 
	Real delta = std::numeric_limits<Real>::max();
	PrescribedMotion* current_pm = nullptr;

	for (const auto& pm : m_prescribedMotionVector)
	{
		if (pm->isInTime(t))
		{
			Real curr_delta = t - pm->getStartTime();
			if (curr_delta < delta)
			{
				delta = curr_delta;
				current_pm = pm;
			}
		}
	}

	if (current_pm)
		current_pm->particleStep(t, delta_t, pd, offset, nParticles);
}