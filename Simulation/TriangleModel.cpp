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

void TriangleModel::addPrescribedMotion(Real startTime, Real endTime, std::string traj[3], Real angVel, Vector3r rotAxis)
{
	PrescribedMotion* pm = new PrescribedMotion();
	pm->initPrescribedMotion(startTime, endTime, traj,
	angVel, rotAxis, Vector3r::Zero());
	m_prescribedMotionVector.push_back(pm);
}
