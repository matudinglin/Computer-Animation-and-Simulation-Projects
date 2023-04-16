#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

enum SkinningTypes { LBS, DQS };
const SkinningTypes skinningTypes = LBS;

Skinning::Skinning(int numMeshVertices, const double* restMeshVertexPositions,
	const std::string& meshSkinningWeightsFilename)
{
	this->numMeshVertices = numMeshVertices;
	this->restMeshVertexPositions = restMeshVertexPositions;

	cout << "Loading skinning weights..." << endl;
	ifstream fin(meshSkinningWeightsFilename.c_str());
	assert(fin);
	int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
	fin >> numWeightMatrixRows >> numWeightMatrixCols;
	assert(fin.fail() == false);
	assert(numWeightMatrixRows == numMeshVertices);
	int numJoints = numWeightMatrixCols;

	vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
	vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
	fin >> ws;
	while (fin.eof() == false)
	{
		int rowID = 0, colID = 0;
		double w = 0.0;
		fin >> rowID >> colID >> w;
		weightMatrixColumnIndices[rowID].push_back(colID);
		weightMatrixEntries[rowID].push_back(w);
		assert(fin.fail() == false);
		fin >> ws;
	}
	fin.close();

	// Build skinning joints and weights.
	numJointsInfluencingEachVertex = 0;
	for (int i = 0; i < numMeshVertices; i++)
		numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
	assert(numJointsInfluencingEachVertex >= 2);

	// Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
	meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
	meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
	for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
	{
		vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
		for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
		{
			int frameID = weightMatrixColumnIndices[vtxID][j];
			double weight = weightMatrixEntries[vtxID][j];
			sortBuffer[j] = make_pair(weight, frameID);
		}
		sortBuffer.resize(weightMatrixEntries[vtxID].size());
		assert(sortBuffer.size() > 0);
		sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
		for (size_t i = 0; i < sortBuffer.size(); i++)
		{
			meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
			meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
		}

		// Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
		// the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
	}
}

void Skinning::applySkinning(const RigidTransform4d* jointSkinTransforms, double* newMeshVertexPositions) const
{
	switch (skinningTypes)
	{
	case LBS:
		applyLBS(jointSkinTransforms, newMeshVertexPositions);
		break;
	case DQS:
		applyDQS(jointSkinTransforms, newMeshVertexPositions);
		break;
	default:
		break;
	}


}

void Skinning::applyLBS(const RigidTransform4d* jointSkinTransforms, double* newMeshVertexPositions) const
{
	for (int i = 0; i < numMeshVertices; ++i)
	{
		Vec4d restMeshVertexPos(restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2], 1.0);
		Vec4d newMeshVertexPos(0.0, 0.0, 0.0, 0.0);
		// compute LBS positions
		for (int j = 0; j < numJointsInfluencingEachVertex; ++j)
		{
			int vertexIdx = i * numJointsInfluencingEachVertex + j;
			newMeshVertexPos += meshSkinningWeights[vertexIdx] * jointSkinTransforms[meshSkinningJoints[vertexIdx]] * restMeshVertexPos;
		}
		// set results
		newMeshVertexPositions[3 * i + 0] = newMeshVertexPos[0];
		newMeshVertexPositions[3 * i + 1] = newMeshVertexPos[1];
		newMeshVertexPositions[3 * i + 2] = newMeshVertexPos[2];
	}
}

using Eigen::Quaterniond;
using Eigen::Matrix3d;
using Eigen::Vector3d;

void Skinning::applyDQS(const RigidTransform4d* jointSkinTransforms, double* newMeshVertexPositions) const
{
	for (int i = 0; i < numMeshVertices; ++i)
	{
		// q = q_0 + e * q_e
		// q_0: rotation
		// q_e: 0.5 * translation * rotation
		Quaterniond q_0(0.0, 0.0, 0.0, 0.0), q_e(0.0, 0.0, 0.0, 0.0);

		for (int j = 0; j < numJointsInfluencingEachVertex; ++j)
		{
			int vertexIdx = numJointsInfluencingEachVertex * i + j;

			// for each joint, form the dual queternion
			Matrix3d rotationj;
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					rotationj(r, c) = jointSkinTransforms[meshSkinningJoints[vertexIdx]][r][c];
			Quaterniond q_0j(rotationj);
			if (q_0j.w() < 0) q_0j.w() = -q_0j.w();


			Vec3d translationj = jointSkinTransforms[meshSkinningJoints[vertexIdx]].getTranslation();
			Quaterniond tj(0, translationj[0], translationj[1], translationj[2]);
			Quaterniond q_ej = tj * q_0j;
			q_ej.coeffs() *= 0.5;

			q_0j.normalized();
			q_ej.normalized();

			// compute q
			q_0.coeffs() += meshSkinningWeights[vertexIdx] * q_0j.coeffs();
			q_e.coeffs() += meshSkinningWeights[vertexIdx] * q_ej.coeffs();
		}

		// get unit dual quaternion
		q_0.normalized();
		q_e.normalized();

		// get rotation and translation from result dual quaternion
		Matrix3d rotation = q_0.toRotationMatrix();
		Quaterniond t = q_e * q_0.inverse();
		t.coeffs() *= 2.0;
		Vector3d translation(t.x(), t.y(), t.z());
		// compute results
		Vector3d restMeshVertexPos(restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2]);
		Vector3d newMeshVertexPos = rotation * restMeshVertexPos + translation;
		// set results 
		newMeshVertexPositions[3 * i + 0] = newMeshVertexPos[0];
		newMeshVertexPositions[3 * i + 1] = newMeshVertexPos[1];
		newMeshVertexPositions[3 * i + 2] = newMeshVertexPos[2];
	}
}



