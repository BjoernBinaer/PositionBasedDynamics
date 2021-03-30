
#ifndef __VOLUME_INTEGRATION_H__
#define __VOLUME_INTEGRATION_H__

#include <vector>
#include <string>
#include <iostream>

#include "Common/Common.h"

namespace Utilities
{
	class VolumeIntegration
	{

	private:
		int A;
		int B;
		int C;

		// projection integrals 
		double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
		// face integrals 
		double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
		// volume integrals 
		double T0;
		double T1[3];
		double T2[3];
		double TP[3];

	public:

		VolumeIntegration(const unsigned int nVertices, const unsigned int nFaces, Vector3r * const vertices, const unsigned int* indices);

		/** Compute inertia tensor for given geometry and given density. 
		*/
		void compute_inertia_tensor(double density);

		/** Return mass of body. */
		double getMass() const { return m_mass; }
		/** Return volume of body. */
		double getVolume() const { return m_volume; }
		/** Return inertia tensor of body. */
		Eigen::Matrix3d const& getInertia() const { return m_theta; }
		/** Return center of mass. */
		Eigen::Vector3d const& getCenterOfMass() const { return m_r; }

	private:

		void volume_integrals();
		void face_integrals(unsigned int i);

		/** Compute various integrations over projection of face.
		*/
		void projection_integrals(unsigned int i);


		std::vector<Eigen::Vector3d> m_face_normals;
		std::vector<double> m_weights;
		unsigned int m_nVertices;
		unsigned int m_nFaces;
		std::vector<Eigen::Vector3d> m_vertices;
		const unsigned int* m_indices;

		double m_mass, m_volume;
		Eigen::Vector3d m_r;
		Eigen::Vector3d m_x;
		Eigen::Matrix3d m_theta;
	};
}

#endif 
